# The `roboplan.mpc` bindings, backed by `_mpc_ext`.
# Import core first to guarantee its types are registered before use.
from roboplan_common import add_dll_directories

_dll_directories = add_dll_directories()

import roboplan.core  # noqa: F401

from ._mpc_ext import *
from ._mpc_ext import __version__  # noqa: F401
