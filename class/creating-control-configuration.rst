ROS 2 control configuration
===========================

The next configuration step is to create the configuration for ros2_control
and ros2_controllers. So that we can control the robot with ROS 2 tools.


System interface (ros2_control)
-------------------------------

First we will create the configuration for the ros2_control interface.
This needs to be done with another xacro file. Create a new file called
``robby.ros2_control.xacro`` in the ``urdf`` folder.

.. code:: bash
    
    touch urdf/robby.ros2_control.xacro

Paste the following content into the file:

.. code:: xml

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:macro name="robby_control" params="
        name 
        prefix 
        use_mock_hardware
        bus_config
        master_config
        can_interface_name
        master_bin
        ">

        <ros2_control name="${name}" type="system">
        <xacro:unless value="${use_mock_hardware}">
            <hardware>
            <plugin>canopen_ros2_control/RobotSystem</plugin>
            <param name="bus_config">${bus_config}</param>
            <param name="master_config">${master_config}</param>
            <param name="can_interface_name">${can_interface_name}</param>
            <param name="master_bin">${master_bin}</param>
            </hardware>
        </xacro:unless> -->
        <xacro:if value="${use_mock_hardware}">
            <hardware>
            <plugin>mock_components/GenericSystem</plugin>
            <param name="calculate_dynamics">true</param>
            </hardware>
        </xacro:if>
        <joint name="${prefix}left_wheel_joint">
            <command_interface name="velocity"/>
            <param name="device_name">left_drive</param>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="${prefix}right_wheel_joint">
            <command_interface name="velocity"/>
            <param name="device_name">right_drive</param>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        </ros2_control>

    </xacro:macro>

    </robot>

Let's check the contents:

.. code:: xml

    <ros2_control name="${name}" type="system">
    </ros2_control>

This creates a ros2_control system interface with the name ``${name}``.

.. code:: xml

    <xacro:unless value="${use_mock_hardware}">
        <hardware>
        <plugin>canopen_ros2_control/RobotSystem</plugin>
        <param name="bus_config">${bus_config}</param>
        <param name="master_config">${master_config}</param>
        <param name="can_interface_name">${can_interface_name}</param>
        <param name="master_bin">${master_bin}</param>
        </hardware>
    </xacro:unless> -->
    <xacro:if value="${use_mock_hardware}">
        <hardware>
        <plugin>mock_components/GenericSystem</plugin>
        <param name="calculate_dynamics">true</param>
        </hardware>
    </xacro:if>

This creates the system interface using the plugin ``canopen_ros2_control/RobotSystem``
if ``use_mock_hardware`` is false. Otherwise it uses the plugin ``mock_components/GenericSystem``.
is used. 

The plugin ``canopen_ros2_control/RobotSystem`` is a plugin that is provided by the
``canopen_ros2_control`` package. It needs the following parameters:

* **bus_config**: Path to bus.yml
* **master_config**: Path to master.dcf (generated)
* **can_interface_name**: Name of the CAN interface
* **master_bin**: Path to the master binary

The plugin ``mock_components/GenericSystem`` is a plugin that is provided by the
``mock_components`` package. It enables us to create a mock system on ros2_control
level.


.. code:: xml

    <joint name="${prefix}left_wheel_joint">
        <command_interface name="velocity"/>
        <param name="device_name">left_drive</param>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    <joint name="${prefix}right_wheel_joint">
        <command_interface name="velocity"/>
        <param name="device_name">right_drive</param>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>

Here we define the interfaces the joints have and which CANopen device
is used for the joint. The ``device_name`` parameter is used to find the
device in the bus configuration.

Now we need to include this file in the ``robby.system.xacro`` file. 

.. code:: xml

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://wiki.ros.org/xacro" name="test_robot">
        <xacro:arg name="use_mock_hardware" default="false" />
        <xacro:arg name="can_interface_name" default="vcan0" />
        <xacro:include filename="$(find robby_description)/urdf/robby.robot.xacro"/>
        <xacro:include filename="$(find robby_description)/urdf/robby.ros2_control.xacro"/>
        <link name="world" />

        <xacro:robby
            name="robby"
            prefix=""
            parent="world"
            >
                <origin xyz="0 0 0.014" rpy="0 0 -1.57" />
        </xacro:robby>

        <xacro:robby_control
            name="robby" 
            prefix="" 
            use_mock_hardware="$(arg use_mock_hardware)"
            bus_config="$(find robby_description)/config/robby/bus.yml"
            master_config="$(find robby_description)/config/robby/master.dcf"
            can_interface_name="$(arg can_interface_name)"
            master_bin=""
            />

    </robot>

Controllers
-----------

Now we need to create the configuration for the controllers. Create a new
file called ``ros2_controllers.yaml`` in the ``config/robby`` folder.

.. code:: bash
    
    touch config/robby/ros2_controllers.yaml

For our robot we need a joint_state_broadcaster which publishes the state
of the wheels to the ``joint_states`` topic. We will also use a ``forward_command_controller``
and a ``diff_drive_controller`` to control the robot. Therefore, we need
to tell the ``controller_manager`` about these controllers. Add the following lines 
to the ``ros2_controllers.yaml`` file:

.. code:: yaml

    controller_manager:
    ros__parameters:
        update_rate: 50  # Hz

        joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

        robby_base_controller:
        type: diff_drive_controller/DiffDriveController

        robby_forward_controller:
        type: forward_command_controller/ForwardCommandController

We also need to configure the controllers. For the forward_command_controller
this is straight forward. Add the following lines to the ``ros2_controllers.yaml`` file:

.. code:: yaml

    robby_forward_controller:
    ros__parameters:
        left_wheel: left_wheel_joint
        right_wheel: right_wheel_joint
    interface_name: velocity

The diff_drive_controller is more complex. We need to configure many things.
Add the following lines to the ``ros2_controllers.yaml`` file:


.. code:: yaml

    robby_base_controller:
        ros__parameters:
            left_wheel_names: ["left_wheel_joint"]
            right_wheel_names: ["right_wheel_joint"]

            wheel_separation: 0.24
            #wheels_per_side: 1  # actually 2, but both are controlled by 1 signal
            wheel_radius: 0.035

            wheel_separation_multiplier: 1.0
            left_wheel_radius_multiplier: 1.0
            right_wheel_radius_multiplier: 1.0

            publish_rate: 50.0
            odom_frame_id: world
            base_frame_id: base_link
            pose_covariance_diagonal : [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
            twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

            open_loop: true
            enable_odom_tf: true

            cmd_vel_timeout: 0.5
            #publish_limited_velocity: true
            use_stamped_vel: false
            #velocity_rolling_window_size: 10

            # Velocity and acceleration limits
            # Whenever a min_* is unspecified, default to -max_*
            linear.x.has_velocity_limits: true
            linear.x.has_acceleration_limits: true
            linear.x.has_jerk_limits: false
            linear.x.max_velocity: 1.0
            linear.x.min_velocity: -1.0
            linear.x.max_acceleration: 1.0
            linear.x.max_jerk: 0.0
            linear.x.min_jerk: 0.0

            angular.z.has_velocity_limits: true
            angular.z.has_acceleration_limits: true
            angular.z.has_jerk_limits: false
            angular.z.max_velocity: 1.0
            angular.z.min_velocity: -1.0
            angular.z.max_acceleration: 1.0
            angular.z.min_acceleration: -1.0
            angular.z.max_jerk: 0.0
            angular.z.min_jerk: 0.0

This is it, we are done with the configurations. Now we only need to write
launch files to use the robot.