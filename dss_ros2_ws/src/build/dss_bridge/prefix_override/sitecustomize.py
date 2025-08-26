import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Project/DSS_ROS2/dss_ros2_ws/src/install/dss_bridge'
