import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dan/duckietown-ros2-ws/dt-core/packages/robots/duckiebot/dagu_car/install'
