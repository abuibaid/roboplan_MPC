"""
Unit tests for the MPC in RoboPlan.
"""

import numpy as np
import pytest

from roboplan.core import (
    JointConfiguration,
    Scene,
    loadJointLimitsConfig,
    loadUrdfSceneDescription,
)
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.mpc import MPC, MPCOptions

GROUP_NAME = "arm"


@pytest.fixture
def test_scene() -> Scene:
    roboplan_models_dir = get_package_models_dir()
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    yaml_path = roboplan_models_dir / "ur_robot_model" / "ur5_config.yaml"
    package_paths = [get_package_share_dir()]

    description = loadUrdfSceneDescription(urdf_path, package_paths)
    scene = Scene("test_scene", description)
    scene.importJointLimitsFromConfig(loadJointLimitsConfig(yaml_path))
    scene.importSrdf(srdf_path.read_text())
    return scene


@pytest.mark.parametrize("solver", ["sqpmethod", "ipopt"])
def test_reaches_goal_within_limits(test_scene: Scene, solver: str) -> None:
    controller = MPC(test_scene, MPCOptions(group_name=GROUP_NAME, solver=solver))
    lower_q, upper_q = test_scene.getPositionLimitVectors(GROUP_NAME)
    min_dq, max_dq = test_scene.getVelocityLimitVectors(GROUP_NAME)
    min_ddq, max_ddq = test_scene.getAccelerationLimitVectors(GROUP_NAME)

    goal = JointConfiguration()
    goal.positions = np.array([0.8, -1.2, 1.3, -0.6, 1.2, 0.4])
    controller.setGoal(goal)

    state = JointConfiguration()
    state.positions = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
    state.velocities = np.zeros(6)

    # Closed loop over 6 s, following the predicted trajectory as a joint trajectory controller would.
    for _ in range(int(6.0 / controller.getTimeStep())):
        trajectory = controller.solve(state)
        assert len(trajectory.positions) == controller.getHorizon() + 1
        ddq = (trajectory.velocities[1] - state.velocities) / controller.getTimeStep()
        assert np.all(ddq >= min_ddq - 1e-6)
        assert np.all(ddq <= max_ddq + 1e-6)
        state.positions = trajectory.positions[1]
        state.velocities = trajectory.velocities[1]
        assert np.all(state.positions >= lower_q - 1e-6)
        assert np.all(state.positions <= upper_q + 1e-6)
        assert np.all(state.velocities >= min_dq - 1e-6)
        assert np.all(state.velocities <= max_dq + 1e-6)

    assert np.max(np.abs(state.positions - goal.positions)) < 1e-3
    assert np.linalg.norm(state.velocities) < 1e-3


def test_invalid_state(test_scene: Scene) -> None:
    controller = MPC(test_scene, MPCOptions(group_name=GROUP_NAME))
    state = JointConfiguration()
    state.positions = np.zeros(3)
    state.velocities = np.zeros(3)
    with pytest.raises(RuntimeError):
        controller.solve(state)
