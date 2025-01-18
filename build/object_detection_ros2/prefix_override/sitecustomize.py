import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/i880/programming/ros2/object_detection_ros2/install/object_detection_ros2'
