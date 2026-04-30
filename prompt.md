I need you to build a ROS2 spacecraft attitude control simulator in 3 hours. This is a job application technical submission for an embedded software engineering position at a spacecraft company.



# Project overview:

A ROS2-based attitude control simulator with a 3D visualization, demonstrating a
full attitude control pipeline similar to real flight software architecture.



## Node architecture:

- `simulator` (C++): 
    - subscribes to `/vehicle_params` (custom message with inertia matrix)
    - subscribes to `/torque_commands` (`geometry_msgs/msg/Vector3`) to get torques to apply to body,
    - simulate the rotation at a fixed timestep
    - publishes `/current_attitude` (`geometry_msgs/msg/Vector3`) and `/current_rates` (`geometry_msgs/msg/Vector3`)

- `rate_controller` (C++):
    - subscribes to `/vehicle_params` (custom message with rate PD gains and FF)
    - subscribes to `/desired_rates` (`geometry_msgs/msg/Vector3`) and `/current_rates` (`geometry_msgs/msg/Vector3`)
    - runs PD control
    - publishes `/torque_commands` (`geometry_msgs/msg/Vector3`)

- `attitude_controller` (C++):
    - subscribes to `/vehicle_params` (custom message with attitude PID gains and FF)
    - subscribes to `/desired_attitude` (`geometry_msgs/msg/Vector3`) and `/current_attitude` (`geometry_msgs/msg/Vector3`)
    - computes attitude error
    - runs PID control
    - publishes `/desired_rates` (`geometry_msgs/msg/Vector3`)

- `visualizer` (Python):
    - subscribes to `/current_attitude` and `/current_rates`
    - renders a real-time 3D spacecraft (load 3d file, I can use any format)
    - use PyQtGraph + PyQt6, so it can render the spacecraft and also have 6 plots of attitude and rate (roll, pitch, yaw, roll rate, pitch rate, yaw rate) 

- `flight_controller` (Python):
    - publishes PID gains and desired attitude on demand via ROS2
    - simple interactive UI (sliders with tkinter) for:
        - Desired attitude as Euler angles (roll, pitch, yaw) relative to inertial frame
        - PID gains for attitude controller
        - PD gains for rate controller
        - Inertia tensor diagonal (Ixx, Iyy, Izz)

## Topics:

- `/vehicle_params` - custom with everything needed for the controllers and simulator
- `/desired_attitude` - `geometry_msgs/msg/Vector3`
- `/current_attitude` - `geometry_msgs/msg/Vector3`
- `/current_rates` - `geometry_msgs/msg/Vector3`
- `/desired_rates` - `geometry_msgs/msg/Vector3`
- `/torque_commands` - `geometry_msgs/msg/Vector3`




## Technical requirements:

- ROS2 Humble
- C++ nodes use `rclcpp`, Python nodes use `rclpy`
- Fixed timestep loop in simulator (100Hz target), ignore messages that arrive too late
- Rate and attitude controller runs at same frequency as simulator
- All nodes launch from a single launch file with default parameters set to start with something (e.g. inertia tensor, PID gains)


## What good looks like:

- Change desired attitude via sliders and watch the 3D spacecraft rotate to match
- Change PID gains and see the response change in real time
- Attitude error plot shows convergence
- Each node does one thing


## Code quality:

Proper ROS2 conventions
The C++ nodes should look like they could run on embedded hardware with minimal changes — no unnecessary dependencies, fixed timestep, deterministic behavior

Please build this step by step, starting with the package structure and CMakeLists, then the simulator node, then the controllers, then the visualizer and params node, and finally the launch file. Test each node before moving to the next.