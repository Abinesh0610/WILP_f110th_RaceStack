import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bits/ABINESH_Packages/racer_ws/install/f1tenth_race_stack'
