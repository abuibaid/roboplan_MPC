# roboplan_MPC

Joint-space model predictive control (MPC) built on [RoboPlan](https://github.com/open-planning/roboplan), [Pinocchio](https://github.com/stack-of-tasks/pinocchio), and [CasADi](https://web.casadi.org/).

![MPC demo: UR5 and Franka FR3](media/mpc_demo.gif)

## How it works

![How it works](media/mpc_simple.svg)

1. **OInK** (RoboPlan) converts a target pose into a joint goal, once per target.
2. **The MPC** plans the joint accelerations for the next 1.5 s toward that goal, within the joint position, velocity, acceleration, and torque limits. **Pinocchio** computes the torque each planned acceleration requires (inverse dynamics), which the MPC keeps within the motor limits.
3. **The robot** follows the plan, e.g. through a joint trajectory controller. Every 0.1 s the MPC replans from the measured state.

By default each cycle runs one SQP iteration (CasADi `sqpmethod` with OSQP), warm started from the previous solution: about 2–10 ms per solve. IPOPT is available through `MPCOptions(solver="ipopt")`.

## Build and run

Requires [pixi](https://pixi.sh). RoboPlan, Pinocchio, and CasADi come from conda-forge.

```bash
pixi run build      # configure, build, and install into the pixi environment
pixi run test       # C++ and Python tests
pixi run example    # browser viewer at http://localhost:8080 (--model ur5|franka|dual|so101)
pixi run example --z-min 0.0   # with a floor at z = 0 (environment constraints)
```

The example is in [roboplan_mpc_examples/python/example_mpc.py](roboplan_mpc_examples/python/example_mpc.py), in the same layout as RoboPlan's `roboplan_examples`.

## Usage

```python
from roboplan.mpc import MPC, MPCOptions

mpc = MPC(scene, MPCOptions(group_name="arm"))
mpc.setGoal(goal)              # JointConfiguration, e.g. from OInK
trajectory = mpc.solve(state)  # JointTrajectory over the horizon, starting at the measured state
```

The same API is available in C++ (`roboplan::MPC`, `roboplan::MPCOptions`).

Environment constraints keep the joints and the tool of the arm (the frames T2 ... T(n+1)) within bounds in the world frame, e.g. above a table:

```python
options = MPCOptions(group_name="arm", tip_frame="tool0",
                     environment_min=[-inf, -inf, 0.0],   # z_min; ±inf: not constrained
                     d_sec=0.06)                           # safety distance [m]
```

The transformations T1 ... T(n+1) come from `RobotTransformation` (C++), the forward kinematics of the Pinocchio model on CasADi symbols, in place of DH parameters.

## Roadmap

### Done

- [x] Joint-space MPC in C++ with Python bindings: tracks a joint goal within the joint position, velocity, acceleration, and torque limits of a RoboPlan `Scene`.
- [x] Pinocchio as part of the MPC: inverse dynamics (RNEA) gives the torque of each planned acceleration, kept within the motor limits.
- [x] CasADi formulation (multiple shooting), solved with one warm-started SQP iteration per cycle (OSQP) or with IPOPT.
- [x] Robust solves: unbounded initial state, and a cold-start retry when a warm-started solve fails.
- [x] Cartesian targets through OInK (once per target), and a browser example on the UR5, Franka FR3, dual Franka FR3, and SO-101.
- [x] C++ and Python tests.
- [x] `RobotTransformation`: the transformations T1 ... T(n+1) of the robot from the Pinocchio model (URDF), on CasADi symbols.
- [x] Environment constraints: the joints and the tool stay within bounds in the world frame (e.g. a table), shown in the example with `--z-min`.

### Next

1. **Composable costs and constraints.** Build the optimal control problem from terms added with `addCost`, `addTerminalCost`, and `addConstraint` (as in RoboPlan's [#287](https://github.com/open-planning/roboplan/pull/287)) instead of a fixed problem.
2. **Collision avoidance.** Self-collision and static obstacles as distance constraints, with the robot links modelled as capsules (line segments), following [KoKoBot](https://github.com/ngafur/MPC_MultiRobot_Motion_Planning). The geometry comes from the RoboPlan `Scene`.
3. **Moving obstacles.** Predicted obstacle motion over the horizon as time-varying constraints, set at every cycle.
4. **Multiple robots.** Distributed MPC for robots sharing a workspace, where each robot plans with its neighbours' predicted trajectories, as in KoKoBot.
5. **ROS 2 integration.** A node that sends the predicted trajectory to a `joint_trajectory_controller`, tested in simulation and on a real UR5e.

## References

- **RoboPlan**: modern robot motion planning library, which this MPC builds on (scene, limits, OInK, example models). [github.com/open-planning/roboplan](https://github.com/open-planning/roboplan), [documentation](https://roboplan.readthedocs.io). Discussion of this MPC with the RoboPlan maintainers: [open-planning/roboplan#399](https://github.com/open-planning/roboplan/issues/399).
- **KoKoBot**: online motion control for a multi-robot system based on (distributed) model predictive control, by Nigora Gafur, RPTU Kaiserslautern-Landau. This MPC builds on its formulation, and its collision avoidance and multi-robot coordination are the next steps here. [github.com/ngafur/MPC_MultiRobot_Motion_Planning](https://github.com/ngafur/MPC_MultiRobot_Motion_Planning).
- **Pinocchio**: rigid-body dynamics, used for the inverse dynamics of the MPC. [github.com/stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio).
- **CasADi**: symbolic framework for the optimal control problem. [web.casadi.org](https://web.casadi.org).
- **OSQP**: QP solver of the SQP iterations. [osqp.org](https://osqp.org).
