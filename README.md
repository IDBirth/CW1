# turtlesim_pde4430

Bilal Baslar
PDE4430
7th Nov 2025

PDE4430 TurtleSim package.
---
Terminal A
source /opt/ros/jazzy/setup.bash && ros2 run turtlesim turtlesim_node

Terminal B
$WS="Your work space"
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

---

## Task 1:
1) turtle_teleop_key
2) eometry_msgs/msg/ ... Twist
3) 10 per second
4) turtlesim/msg/Pose
5) '''
mdx-msc-bilal@Ubunt24:~/Desktop/MDX ROS/CW1$ ros2 topic echo /turtle1/color_sensor --once
r: 255
g: 255
b: 255
'''

## Task 2:
1) Image at images/T2-1.png
  Run:


2) Video(T2-2) inside images folder, run:
ros2 run turtlesim_pde4430 circular_move --ros-args -p radius:=3.0 -p speed:=1.5

3) Video(T2-3) inside images folder run:
ros2 run turtlesim_pde4430 figure_eight --ros-args \
  -p a:=3.0 -p b:=3.0 -p omega:=0.6 -p k_pos:=1.2 -p k_ang:=4.0 -p v_max:=2.0

4) Video T2-4, run:
ros2 run turtlesim_pde4430 double_sweep_roomba


5) Video T2-5, but one turttle was off that hour, run:
ros2 run turtlesim_pde4430 full_map_cleaner


## Task 3

1) VIDEO T3-1, run:

ros2 run turtlesim_pde4430 user_input


2) VIDEO T3-2, run:

ros2 run turtlesim_pde4430 goto_goal_loop --ros-args   -p k_lin:=1.2 -p k_ang:=5.0 -p v_max:=2.4 -p w_max:=3.2   -p tolerance:=0.12 -p slowdown_radius:=1.0