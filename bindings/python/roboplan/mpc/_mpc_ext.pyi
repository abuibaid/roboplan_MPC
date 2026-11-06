from typing import Annotated

import numpy
from numpy.typing import NDArray
import roboplan.core._core_ext


class MPCOptions:
    """Options struct for the MPC."""

    def __init__(self, group_name: str = '', Ts: float = 0.1, N: int = 15, Q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] = ..., Qf: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] = ..., R: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] = ..., Rd: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] = ..., solver: str = 'sqpmethod', jit: bool = False) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used by the controller."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def Ts(self) -> float:
        """The time step of the prediction model, in seconds."""

    @Ts.setter
    def Ts(self, arg: float, /) -> None: ...

    @property
    def N(self) -> int:
        """The number of steps in the prediction horizon."""

    @N.setter
    def N(self, arg: int, /) -> None: ...

    @property
    def Q(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Diagonal weights on the joint state error [q; dq]. Defaults to 1 for positions and 0.1 for velocities if empty.
        """

    @Q.setter
    def Q(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def Qf(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Diagonal weights on the terminal joint state error. Defaults to 3 Q if empty.
        """

    @Qf.setter
    def Qf(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def R(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Diagonal weights on the inputs (joint accelerations). Defaults to 0.01 if empty.
        """

    @R.setter
    def R(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def Rd(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Diagonal weights on the input rates. Defaults to 0.1 if empty."""

    @Rd.setter
    def Rd(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def solver(self) -> str:
        """
        The NLP solver: 'sqpmethod' for one SQP iteration per call with OSQP (real-time iteration, fast), or 'ipopt' to solve to convergence on every call.
        """

    @solver.setter
    def solver(self, arg: str, /) -> None: ...

    @property
    def jit(self) -> bool:
        """
        If true, compiles the problem functions just in time. Requires a C compiler at runtime, and takes seconds to minutes when constructing; most of the solve time is spent in IPOPT itself, so the speedup is small.
        """

    @jit.setter
    def jit(self, arg: bool, /) -> None: ...

class MPC:
    """Joint-space model predictive control (MPC) for motion control."""

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: MPCOptions) -> None: ...

    def setGoal(self, goal: roboplan.core._core_ext.JointConfiguration) -> None:
        """Sets the desired joint state."""

    def solve(self, state: roboplan.core._core_ext.JointConfiguration) -> roboplan.core._core_ext.JointTrajectory:
        """
        Solves the MPC from the current joint state. Returns the predicted joint trajectory over the horizon, starting at the current state.
        """

    def getTimeStep(self) -> float:
        """Returns the time step of the prediction model, in seconds."""

    def getHorizon(self) -> int:
        """Returns the number of steps in the prediction horizon."""
