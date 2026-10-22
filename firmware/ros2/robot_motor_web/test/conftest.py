from pathlib import Path
import sys
here=Path(__file__).resolve()
sys.path[:0]=[str(here.parents[1]),str(here.parents[2]/'robot_motor_driver')]
