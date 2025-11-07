# turtlesim_pde4430

Bilal Baslar  
PDE4430  
7 Nov 2025

PDE4430 TurtleSim package.

---

## Setup

**Terminal A**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node

Terminal B

WS="your_workspace_path"
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

Task 1

    turtle_teleop_key

ros2 run turtlesim turtle_teleop_key

    geometry_msgs/msg/Twist

    10 Hz

    turtlesim/msg/Pose

    Color sensor (example)

ros2 topic echo /turtle1/color_sensor --once
# r: 255
# g: 255
# b: 255

Task 2

    Image: images/T2-1.png

    Circular move (Video T2-2)

ros2 run turtlesim_pde4430 circular_move --ros-args -p radius:=3.0 -p speed:=1.5

    Figure eight (Video T2-3)

ros2 run turtlesim_pde4430 figure_eight --ros-args \
  -p a:=3.0 -p b:=3.0 -p omega:=0.6 -p k_pos:=1.2 -p k_ang:=4.0 -p v_max:=2.0

    Double sweep roomba (Video T2-4)

ros2 run turtlesim_pde4430 double_sweep_roomba

    Full map cleaner (Video T2-5)

ros2 run turtlesim_pde4430 full_map_cleaner

Task 3

    User input (Video T3-1)

ros2 run turtlesim_pde4430 user_input

    Go to goal loop (Video T3-2)

ros2 run turtlesim_pde4430 goto_goal_loop --ros-args \
  -p k_lin:=1.2 -p k_ang:=5.0 -p v_max:=2.4 -p w_max:=3.2 \
  -p tolerance:=0.12 -p slowdown_radius:=1.0


Or create it from the terminal (in your repo root):

```bash
cat > README.md <<'EOF'
# turtlesim_pde4430

Bilal Baslar  
PDE4430  
7 Nov 2025

PDE4430 TurtleSim package.

---

## Setup

**Terminal A**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node

Terminal B

WS="your_workspace_path"
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

Task 1

    turtle_teleop_key

ros2 run turtlesim turtle_teleop_key

    geometry_msgs/msg/Twist

    10 Hz

    turtlesim/msg/Pose

    Color sensor (example)

ros2 topic echo /turtle1/color_sensor --once
# r: 255
# g: 255
# b: 255

Task 2

    Image: images/T2-1.png

    Circular move (Video T2-2)

ros2 run turtlesim_pde4430 circular_move --ros-args -p radius:=3.0 -p speed:=1.5

    Figure eight (Video T2-3)

ros2 run turtlesim_pde4430 figure_eight --ros-args \
  -p a:=3.0 -p b:=3.0 -p omega:=0.6 -p k_pos:=1.2 -p k_ang:=4.0 -p v_max:=2.0

    Double sweep roomba (Video T2-4)

ros2 run turtlesim_pde4430 double_sweep_roomba

    Full map cleaner (Video T2-5)

ros2 run turtlesim_pde4430 full_map_cleaner

Task 3

    User input (Video T3-1)

ros2 run turtlesim_pde4430 user_input

    Go to goal loop (Video T3-2)

ros2 run turtlesim_pde4430 goto_goal_loop --ros-args \
  -p k_lin:=1.2 -p k_ang:=5.0 -p v_max:=2.4 -p w_max:=3.2 \
  -p tolerance:=0.12 -p slowdown_radius:=1.0