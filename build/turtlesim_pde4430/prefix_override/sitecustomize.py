import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mdx-msc-bilal/Desktop/MDX ROS/CW1/install/turtlesim_pde4430'
