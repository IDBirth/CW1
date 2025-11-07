# turtlesim_pde4430

Bilal Baslar
PDE4430
7th Nov 2025

PDE4430 TurtleSim package.

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
---
'''

## Task 2:
1) DONE Code at /src/turtlesim_pde4430/turtlesim_pde4430/straight_move.py

And

Image at images/T2-1.png

2) DONE run:
ros2 run turtlesim_pde4430 circular_move --ros-args -p radius:=3.0 -p speed:=1.5

change the radius to make it bigger
Video(T2-2) inside images folder

3) DONE run:
ros2 run turtlesim_pde4430 figure_eight --ros-args \
  -p a:=3.0 -p b:=3.0 -p omega:=0.6 -p k_pos:=1.2 -p k_ang:=4.0 -p v_max:=2.0

4) DONE Video T2-4, run:
ros2 run turtlesim_pde4430 double_sweep_roomba


5) TBD!

## Task 3

1) DONE VIDEO T3-1, run:

ros2 run turtlesim_pde4430 user_input


2) DONe VIDEO T3-2, run:

ros2 run turtlesim_pde4430 goto_goal_loop --ros-args   -p k_lin:=1.2 -p k_ang:=5.0 -p v_max:=2.4 -p w_max:=3.2   -p tolerance:=0.12 -p slowdown_radius:=1.0