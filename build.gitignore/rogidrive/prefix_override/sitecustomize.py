import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yuki/ros2_ws/src/NHK2025/install/rogidrive'