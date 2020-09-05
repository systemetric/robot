"""Functions for use in the RoboCon test harness
"""

import contextlib
import sys
from io import StringIO

@contextlib.contextmanager
def captured_output():
    """Monkey patch stdout & stderr to collect output from a function"""
    new_out, new_err = StringIO(), StringIO()
    old_out, old_err = sys.stdout, sys.stderr
    try:
        sys.stdout, sys.stderr = new_out, new_err
        yield sys.stdout, sys.stderr
    finally:
        sys.stdout, sys.stderr = old_out, old_err