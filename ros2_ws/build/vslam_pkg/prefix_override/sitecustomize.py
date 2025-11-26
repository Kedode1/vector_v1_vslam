import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohamed-kassem/vector_v1_vslam/ros2_ws/install/vslam_pkg'
