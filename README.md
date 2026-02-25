# Description
Assignments for robotics motion planning and control module in the University of Birmingham (forward kinematics & inverse kinematics)

# Local Machine Installation
1. **Install ROS 2 Development environment**
    Install the **Development Tools** package:
    ```bash
    sudo apt install ros-dev-tools ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ament-cmake
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS 2** environment but, under some circumstances you may need to do this again:
    ```bash
    source /opt/ros/humble/setup.sh
    ```

2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/assignment1/src
   cd ~/assignment1  # not into src
   ```
3. **Clone the Repositories:**
   ```bash
    git clone https://github.com/HyPAIR/RMPC_Assignment1.git src
    ```
4. **Install the dependencies**
    ```bash
    vcs import src < src/franka.repos --recursive --skip-existing
    ```
5. **Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
6. **Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF --executor sequential
   ```
7. **Adjust Enviroment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

# Joint Position Controller with Gazebo

To visualize the arm and run the joint position controller, you can run with the following command.

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

# Forward Kinematics

To test your forward kinmatics solution, you can run with the following command.

```bash
cd ~/assignment1/src/assignment
python3 forword_kinematics.py
```

# Inverse Kinematics

To test your inverse kinmatics solution, you can run with the following command.

```bash
cd ~/assignment1/src/assignment
python3 inverse_kinematics.py
```
