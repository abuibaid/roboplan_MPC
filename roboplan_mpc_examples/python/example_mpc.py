#!/usr/bin/env python3

import queue
import sys
import time
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import tyro
import xacro
from pinocchio.visualize import ViserVisualizer

from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    JointTrajectory,
    Scene,
    loadJointLimitsConfig,
    loadUrdfSceneDescriptionFromXml,
)
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.mpc import MPC, MPCOptions
from roboplan.optimal_ik import (
    FrameTask,
    FrameTaskOptions,
    IterativeSolveOptions,
    Oink,
    PositionLimit,
)
from roboplan.visualization import plotJointTrajectory, visualizeJointTrajectory


# RoboPlan example models the MPC supports (no continuous joints).
MODELS_DIR = get_package_models_dir()
MODELS = {
    "ur5": SimpleNamespace(
        urdf_path=MODELS_DIR / "ur_robot_model" / "ur5_gripper.urdf",
        srdf_path=MODELS_DIR / "ur_robot_model" / "ur5_gripper.srdf",
        yaml_config_path=MODELS_DIR / "ur_robot_model" / "ur5_config.yaml",
        default_joint_group="arm",
        ee_names=["tool0"],
        base_link="base",
        # The model has a simple gripper on tool0; show the arm only.
        hidden_frames=["tool0"],
    ),
    "franka": SimpleNamespace(
        urdf_path=MODELS_DIR / "franka_robot_model" / "fr3.urdf",
        srdf_path=MODELS_DIR / "franka_robot_model" / "fr3.srdf",
        yaml_config_path=MODELS_DIR / "franka_robot_model" / "fr3_config.yaml",
        default_joint_group="fr3_arm",
        ee_names=["fr3_hand"],
        base_link="fr3_link0",
    ),
    "dual": SimpleNamespace(
        urdf_path=MODELS_DIR / "franka_robot_model" / "dual_fr3.urdf",
        srdf_path=MODELS_DIR / "franka_robot_model" / "dual_fr3.srdf",
        yaml_config_path=MODELS_DIR / "franka_robot_model" / "dual_fr3_config.yaml",
        default_joint_group="dual_fr3_arm",
        ee_names=["left_fr3_hand", "right_fr3_hand"],
        base_link="left_fr3_link0",
    ),
    "so101": SimpleNamespace(
        urdf_path=MODELS_DIR / "so101_robot_model" / "so101.urdf",
        srdf_path=MODELS_DIR / "so101_robot_model" / "so101.srdf",
        yaml_config_path=MODELS_DIR / "so101_robot_model" / "so101_config.yaml",
        default_joint_group="arm",
        ee_names=["gripper_link"],
        base_link="base_link",
    ),
}


def main(
    model: str = "ur5",
    max_time: float = 10.0,
    tolerance: float = 1e-3,
    host: str = "localhost",
    port: str = "8000",
    z_min: float | None = None,
):
    """
    Move a robot to random end-effector poses with the MPC and animate the trajectory.

    OInK converts each Cartesian target into a joint goal, once per target. The robot then
    follows the trajectory predicted by the MPC, as a joint trajectory controller would: at each
    control cycle, it is at the first predicted state. The MPC respects the joint position,
    velocity, acceleration, and torque limits, and optionally keeps the joints and the tool of the
    arm above a floor (environment constraints), but does not avoid other collisions.

    Parameters:
        model: The name of the model to use (ur5, franka, dual, or so101; the MPC does not
            support continuous joints yet).
        max_time: The maximum time (in seconds) to reach the goal.
        tolerance: The joint position (rad) and velocity (rad/s) error at which the goal is reached.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        z_min: The height (in meters) of a floor that the joints and the tool of the arm stay
            above, e.g. 0.0. The targets are then drawn near the floor. No floor if not given.
            One arm only (not dual).
    """
    model_data = MODELS.get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)
    group_name = model_data.default_joint_group
    base_frame = model_data.base_link
    ee_names = model_data.ee_names

    package_paths = [get_package_share_dir()]

    # Pre-process with xacro. This is not necessary for raw URDFs.
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "test_scene",
        loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths),
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)
    q_indices = scene.getJointGroupInfo(group_name).q_indices

    # Build a separate Pinocchio model (with mimic joints) for visualization. Until Pinocchio
    # and Coal have nanobind bindings, it cannot be taken from the scene.
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    for frame in getattr(model_data, "hidden_frames", []):
        for geom_model in (collision_model, visual_model):
            names = [
                geom.name
                for geom in geom_model.geometryObjects
                if model.frames[geom.parentFrame].name == frame
            ]
            for name in names:
                geom_model.removeGeometryObject(name)

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Set up the MPC, and OInK to convert Cartesian targets into joint goals.
    options = MPCOptions(group_name=group_name)
    # Environment constraints: the frames T2 ... T(n+1) stay above the floor.
    joint_names = scene.getJointGroupInfo(group_name).joint_names
    constrained_frames = joint_names[1:] + ee_names[:1]
    if z_min is not None:
        if len(ee_names) > 1:
            print("The floor (z_min) is supported for one arm only.")
            sys.exit(1)
        options.tip_frame = ee_names[0]
        options.environment_min = np.array([-np.inf, -np.inf, z_min])
        # The constraints are on the joint centres, so keep about a link radius to the floor.
        options.d_sec = 0.06
        viz.viewer.scene.add_box(
            "/environment/floor",
            color=(150, 150, 150),
            dimensions=(2.0, 2.0, 0.005),
            opacity=0.5,
            position=(0.0, 0.0, z_min - 0.0025),
        )
    mpc = MPC(scene, options)

    def above_floor(q_full):
        """Whether the constrained frames are above the floor, by the safety distance."""
        return z_min is None or all(
            scene.forwardKinematics(q_full, frame)[2, 3] >= z_min + options.d_sec
            for frame in constrained_frames
        )

    def random_target():
        """A random configuration; with a floor, one with the tool less than 0.1 m above it."""
        while True:
            q = scene.randomCollisionFreePositions()
            if z_min is None or (
                above_floor(q)
                and scene.forwardKinematics(q, ee_names[0])[2, 3] < z_min + 0.1
            ):
                return q

    dt = mpc.getTimeStep()
    anim_dt = 0.01

    # One frame task per end effector.
    oink = Oink(scene, group_name)
    frame_tasks = []
    for ee_name in ee_names:
        target = CartesianConfiguration()
        target.base_frame = base_frame
        target.tip_frame = ee_name
        target.tform = np.eye(4)
        # Damping keeps the steps well conditioned near singular configurations.
        frame_tasks.append(
            FrameTask(oink, scene, target, FrameTaskOptions(lm_damping=0.01))
        )
    position_limit = PositionLimit(oink)
    # The IK runs once per target, so it can take its time.
    ik_options = IterativeSolveOptions()
    ik_options.max_time = 1.0
    ik_options.max_restarts = 10
    ik_options.max_stagnant_iters = 50

    traj_queue = queue.Queue()
    cur_traj = None
    animate = False

    q_full = scene.randomCollisionFreePositions()
    while not above_floor(q_full):
        q_full = scene.randomCollisionFreePositions()
    scene.setJointPositions(q_full)
    viz.display(q_full)
    time.sleep(0.1)

    plan_button = viz.viewer.gui.add_button("Run MPC")

    @plan_button.on_click
    def run_mpc(_):
        nonlocal animate
        animate = False
        plan_button.disabled = True
        animate_button.disabled = True

        state = JointConfiguration()
        state.positions = q_full[q_indices]
        state.velocities = np.zeros(len(q_indices))

        # Random reachable end-effector poses, converted into a joint goal by OInK. Draw
        # other targets if the IK fails, or if the goal is below the floor.
        print("\nSolving IK for random target poses...")
        q_goal_full = None
        for _ in range(20):
            q_target = random_target()
            for frame_task, ee_name in zip(frame_tasks, ee_names):
                frame_task.setTargetFrameTransform(
                    scene.forwardKinematics(q_target, ee_name, base_frame)
                )
            try:
                q_goal_full = oink.solveIterativeIk(
                    q_full,
                    frame_tasks,
                    constraints=[position_limit],
                    options=ik_options,
                )
            except RuntimeError as e:
                print(f"No IK solution, drawing other targets: {e}")
                continue
            if above_floor(q_goal_full):
                break
            q_goal_full = None
        if q_goal_full is None:
            plan_button.disabled = False
            return
        goal = JointConfiguration()
        goal.positions = q_goal_full[q_indices]
        mpc.setGoal(goal)

        # Closed loop: solve, then move to the first predicted state.
        print("Running MPC...")
        traj = JointTrajectory()
        traj.joint_names = scene.getJointGroupInfo(group_name).joint_names
        # The bound vectors are views into the C++ object, so store copies.
        times = [0.0]
        positions = [state.positions.copy()]
        velocities = [state.velocities.copy()]
        solve_times = []
        first_prediction = None
        while times[-1] < max_time:
            t_start = time.time()
            prediction = mpc.solve(state)
            solve_times.append(time.time() - t_start)
            if first_prediction is None:
                first_prediction = prediction

            state.positions = prediction.positions[1]
            state.velocities = prediction.velocities[1]
            times.append(times[-1] + dt)
            positions.append(state.positions.copy())
            velocities.append(state.velocities.copy())

            if (
                np.max(np.abs(state.positions - goal.positions)) < tolerance
                and np.max(np.abs(state.velocities)) < tolerance
            ):
                break
        traj.times, traj.positions, traj.velocities = times, positions, velocities

        reached = times[-1] < max_time
        print(
            f"{'Reached' if reached else 'Did not reach'} the goal in {times[-1]:.2f} s, "
            f"solve time: mean {np.mean(solve_times) * 1000:.2f} ms, "
            f"max {np.max(solve_times) * 1000:.2f} ms"
        )

        # Show the start (green) and goal (red) end-effector positions.
        q_start_full = scene.toFullJointPositions(group_name, positions[0])
        q_goal_full = scene.toFullJointPositions(group_name, goal.positions)
        viz.display(q_start_full)
        for ee_name in ee_names:
            viz.viewer.scene.add_icosphere(
                f"/mpc/start/{ee_name}",
                radius=0.03,
                color=(0, 200, 0),
                position=scene.forwardKinematics(q_start_full, ee_name)[:3, 3],
            )
            viz.viewer.scene.add_icosphere(
                f"/mpc/goal/{ee_name}",
                radius=0.03,
                color=(200, 0, 0),
                position=scene.forwardKinematics(q_goal_full, ee_name)[:3, 3],
            )

        # Show the executed path (red) and the first prediction over the horizon (green).
        visualizeJointTrajectory(viz, scene, traj, ee_names, (100, 0, 0), "/mpc/path")
        visualizeJointTrajectory(
            viz, scene, first_prediction, ee_names, (0, 100, 0), "/mpc/prediction"
        )

        traj_queue.put(traj)
        plan_button.disabled = False
        animate_button.disabled = False

    animate_button = viz.viewer.gui.add_button("Animate trajectory")
    animate_button.disabled = True

    @animate_button.on_click
    def animate_trajectory(_):
        plan_button.disabled = True
        animate_button.disabled = True
        nonlocal animate
        animate = True

    # Main display and animation loop.
    fig = plt.figure()
    plt.ion()

    def process_plot_events():
        """Keeps the plot window responsive, if it is still open."""
        if plt.fignum_exists(fig.number):
            fig.canvas.flush_events()

    while True:
        if not traj_queue.empty():
            cur_traj = traj_queue.get()
            # Reopen the plot window if it was closed.
            if not plt.fignum_exists(fig.number):
                fig = plt.figure()
            plt.figure(fig.number)
            plotJointTrajectory(cur_traj, scene, group_name=group_name, velocities=True)
            fig.canvas.draw()
            plt.pause(0.1)
        elif animate and cur_traj is not None:
            print("Animating trajectory...")
            # Interpolate between the MPC steps, as a joint trajectory controller would.
            positions = np.array(cur_traj.positions)
            for t in np.arange(0.0, cur_traj.times[-1] + anim_dt, anim_dt):
                q = [np.interp(t, cur_traj.times, p) for p in positions.T]
                q_full = scene.toFullJointPositions(group_name, np.array(q))
                viz.display(q_full)
                process_plot_events()
                time.sleep(anim_dt)
            # The next run starts where this one ended.
            scene.setJointPositions(q_full)
            animate = False
            plan_button.disabled = False
            animate_button.disabled = False
            print("...done!")
        else:
            process_plot_events()
            time.sleep(0.02)


if __name__ == "__main__":
    tyro.cli(main)
