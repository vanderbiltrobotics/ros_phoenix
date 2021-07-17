# ros_phoenix
This project contains a ROS2 package for interfacing with CTRE motor controllers. It provides components for controlling Victor SPXs, Talon SRXs, and Falcon 500s. It uses the SocketCAN driver for communicating with these devices over a CAN bus. This version takes advantage of ROS2 features such as composition and improved parameter system.

## Cloning and Building
1. Clone the package into a ROS2 workspace
```
$ cd <ros_ws>/src
$ git clone https://github.com/vanderbiltrobotics/ros_phoenix
$ cd ros_phoenix
$ git checkout foxy
```
2. Build the workspace and source the setup file
```
$ cd <ros_ws>
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
$ source install/setup.bash
```

## Launch File Usage
A launch file is the recommended way to create and configure multiple motor controllers. See [`launch/demo.launch.py`](https://github.com/vanderbiltrobotics/ros_phoenix/blob/foxy/launch/demo.launch.py) for an example. Alternatively, components can be created from the CLI.

## CLI Usage
This package uses ROS2 components to compose nodes into a single process. See the [ROS2 composition tutorial](https://index.ros.org/doc/ros2/Tutorials/Composition/) for a full guide to composition.

1. Start a component container and rename it PhoenixContainer (or any other name)
```
$ ros2 run ros_phoenix phoenix_container --ros-args -r __node:=PhoenixContainer
```
2. Load a ros_phoenix component into the container with the device id number
```
$ ros2 component load /PhoenixContainer ros_phoenix ros_phoenix::TalonSRX --node-name my_talon -p id:=0
```
3. Repeat step 2 for each motor controller

## Phoenix Container
The `phoenix_container` allows motor controller nodes to be dynamically created at runtime. It is responsible for configuring the CAN interface used by all of the motor controllers created inside it. It also feeds the CTRE watchdog. This watchdog is responsible for enabling/disabling all motor controllers on the interface. This global watchdog is separate from the watchdog inside each component which disables an individual motor if it stops receiving updates.

### Parameters
- `interface` ("can0"): The SocketCAN interface
- `period_ms` (50): Period in milliseconds at which watchdog is feed
- `watchdog_ms` (200): Period before motor controllers are disabled if watchdog dies

## Phoenix Components
- `ros_phoenix::VictorSPX`: Victor SPX Component
- `ros_phoenix::TalonSRX`: Talon SRX Component
- `ros_phoenix::TalonFX`: Talon FX / Falcon 500 Component

## Component Interface
### Published Topics
`<node_name>/status` (ros_phoenix/msg/MotorStatus)
- Publishes status information about the motor controller

### Subscribed Topics
`<node_name>/set` (ros_phoenix/msg/MotorControl)
- Sets the control mode and output of the motor controller

### Parameters
- `id` (0): Device CAN bus ID
- `period_ms` (20): Period in milliseconds of status updates
- `watchdog_ms` (100): Watchdog timer. Must be greater than period_ms!
- `follow_id` (-1): If >= to zero, the ID of another device to follow
- `edges_per_rot` (4096): Encoder edges per rotation
- `invert` (false): Invert motor output
- `invert_sensor` (false): Invert sensor direction
- `brake_mode` (true): Enable brake mode
- `analog_input` (false): Use an analog input instead of an encoder
- `max_voltage` (12.0): Voltage which corresponds to a percent output of 1.0
- `max_current` (30.0): Maximum current output
- `sensor_multiplier` (1.0): Factor to convert native device units to meaningful units
- `P` (0.0): kP
- `I` (0.0): kI
- `D` (0.0): kD
- `F` (0.0): kF