ROS2 creating a robot arm
=========================

Based on the previous work, it is also easily possible to create a robot arm. In this
chapter, we discuss the example of a prbt robot which has six axis that are controlled
via CANopen.

Bus configuration
-----------------
The bus configuration needs to configure six drives of the same type. The bus looks
like this:

* id 1: master
* id 3: joint_1
* id 4: joint_2
* id 5: joint_3
* id 6: joint_4
* id 7: joint_5
* id 8: joint_6

The robot uses interpolated position mode for controlling the robot in position mode.

.. code-block:: yaml

    options:
        dcf_path: "@BUS_CONFIG_PATH@"
    master:
    node_id: 1
        driver: "ros2_canopen::MasterDriver"
        package: "canopen_master_driver"
        sync_period: 10000

    defaults:
        dcf: "prbt_0_1.dcf"
        driver: "ros2_canopen::Cia402Driver"
        package: "canopen_402_driver"
        period: 10
        enable_lazy_load: false
        heartbeat_producer: 1000
        switching_state: 2          # Switch in CiA402 State 2
        position_mode: 7            # Use interpolated position mode
        scale_pos_from_dev: 0.00001745329252
        scale_pos_to_dev: 57295.7795131
        sdo:
            - {index: 0x6081, sub_index: 0, value: 1000}
            - {index: 0x60C2, sub_index: 1, value: 15} # Interpolation time period at 10 ms matches the period.
            - {index: 0x6060, sub_index: 0, value: 7} # Make default mode to interpolated position mode.
        tpdo: # TPDO needed statusword, actual velocity, actual position, mode of operation
            1:
            enabled: true
            cob_id: "auto"
            transmission: 0x01
            mapping:
                - {index: 0x6041, sub_index: 0} # status word
                - {index: 0x6061, sub_index: 0} # mode of operaiton display
            2:
            enabled: true
            cob_id: "auto"
            transmission: 0x01
            mapping:
                - {index: 0x6064, sub_index: 0} # position actual value
                - {index: 0x606c, sub_index: 0} # velocity actual position
            3:
            enabled: false
            4:
            enabled: false
        rpdo: # RPDO needed controlword, target position, target velocity, mode of operation
            1:
            enabled: true
            cob_id: "auto"
            mapping:
              - {index: 0x6040, sub_index: 0} # controlword
              - {index: 0x6060, sub_index: 0} # mode of operation
              - {index: 0x60C1, sub_index: 1} # interpolated position data
            2:
            enabled: true
            cob_id: "auto"
            mapping:
              - {index: 0x607A, sub_index: 0} # target position

    nodes:
        prbt_joint_1:
            node_id: 3
        prbt_joint_2:
            node_id: 4
        prbt_joint_3:
            node_id: 5
        prbt_joint_4:
            node_id: 6
        prbt_joint_5:
            node_id: 7
        prbt_joint_6:
            node_id: 8

Control configuration
---------------------
The ros2_rontol configuration requires the prbt.ros2_control.xacro.

.. code-block:: xml

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">

        <xacro:macro name="prbt_ros2_control" params="
        name
        prefix
        bus_config
        master_config
        can_interface_name
        master_bin">

            <ros2_control name="${name}" type="system">
                <hardware>
                    <plugin>canopen_ros2_control/RobotSystem</plugin>
                    <param name="bus_config">${bus_config}</param>
                    <param name="master_config">${master_config}</param>
                    <param name="can_interface_name">${can_interface_name}</param>
                    <param name="master_bin">"${master_bin}"</param>
                </hardware>
                <joint name="${prefix}joint_1">
                    <param name="device_name">prbt_joint_1</param>
                </joint>
                <joint name="${prefix}joint_2">
                    <param name="device_name">prbt_joint_2</param>
                </joint>
                <joint name="${prefix}joint_3">
                    <param name="device_name">prbt_joint_3</param>
                </joint>
                <joint name="${prefix}joint_4">
                    <param name="device_name">prbt_joint_4</param>
                </joint>
                <joint name="${prefix}joint_5">
                    <param name="device_name">prbt_joint_5</param>
                </joint>
                <joint name="${prefix}joint_6">
                    <param name="device_name">prbt_joint_6</param>
                </joint>
            </ros2_control>

        </xacro:macro>

    </robot>

In addition we need to define the ros2_controllers.

.. code-block:: yaml

    controller_manager:
    ros__parameters:
        update_rate: 100  # Hz

        joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

        forward_position_controller:
        type: forward_command_controller/ForwardCommandController
        
        arm_controller:
        type: joint_trajectory_controller/JointTrajectoryController


    forward_position_controller:
    ros__parameters:
        joints:
        - prbt_joint_1
        - prbt_joint_2
        - prbt_joint_3
        - prbt_joint_4
        - prbt_joint_5
        - prbt_joint_6
        interface_name: position

    arm_controller:
    ros__parameters:
        joints:
        - prbt_joint_1
        - prbt_joint_2
        - prbt_joint_3
        - prbt_joint_4
        - prbt_joint_5
        - prbt_joint_6
        command_interfaces:
        - position
        state_interfaces:
        - position
        - velocity
        stop_trajectory_duration: 0.2
        state_publish_rate:  100.0
        action_monitor_rate: 50.0
        goal_time: 0.6
        constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        prbt_joint_1: {trajectory: 0.157, goal: 0.01}
        prbt_joint_2: {trajectory: 0.157, goal: 0.01}
        prbt_joint_3: {trajectory: 0.157, goal: 0.01}
        prbt_joint_4: {trajectory: 0.157, goal: 0.01}
        prbt_joint_5: {trajectory: 0.157, goal: 0.01}
        prbt_joint_6: {trajectory: 0.157, goal: 0.01}
        limits:
        prbt_joint_1:
            has_acceleration_limits: true
            max_acceleration: 6.0
        prbt_joint_2:
            has_acceleration_limits: true
            max_acceleration: 6.0
        prbt_joint_3:
            has_acceleration_limits: true
            max_acceleration: 6.0
        prbt_joint_4:
            has_acceleration_limits: true
            max_acceleration: 6.0
        prbt_joint_5:
            has_acceleration_limits: true
            max_acceleration: 6.0
        prbt_joint_6:
            has_acceleration_limits: true
            max_acceleration: 6.0


Launch files
------------

We have already prepared the launch files for you.
To run the robot you need to do the follwing:


**Terminal 1**

.. code:: bash

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 txqueuelen 1000
    sudo ip link set up vcan0

    . install/setup.bash
    ros2 launch prbt_robot_support prbt_fake_slave.launch.py

**Terminal 2**

.. code:: bash

    . install/setup.bash
    ros2 launch prbt_robot_support robot.launch.py


**Terminal 3**

.. code:: bash

    . install/setup.bash
    ros2 launch prbt_robot_moveit_config vcan.launch.py


**Terminal 4**

.. code:: bash

    . install/setup.bash
    ros2 control switch_controllers --activate arm_controller --deactivate forward_position_controller


In case there is an error with the ``ros2 control`` command, install it using the following command:

.. code:: bash

    sudo apt install ros-rolling-ros2controlcli

Once you ran the ``ros2 control switch_controllers`` command, you can use the ``ros2 control list_controllers`` command to see the current state of the controllers.
The arm controller should be active.

No you can use RVIZ (moveit) to move your robot arm. The robot arm is emulated on the vcan0 interface.
You can also use ``candump vcan0`` to the the CAN communication with the fake drives.

