import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohammed-sameh/hatem_version/ros2_ws/install/rpi_controller_pkg'
