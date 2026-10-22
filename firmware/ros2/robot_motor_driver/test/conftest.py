"""Make the driver importable from the source tree.

colcon/ctest run these tests from the build directory, where the package may not
be on sys.path yet; inserting the parent directory keeps `pytest test/` working
from a plain checkout too, with or without an install space.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)
