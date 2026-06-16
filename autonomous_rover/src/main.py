import os
import sys

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from mission_controller import main
else:
    from .mission_controller import main


if __name__ == "__main__":
    main()
