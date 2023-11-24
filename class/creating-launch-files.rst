ROS 2 launch files
==================

We will create three launch files.

* control simulation
* forward controller (fake & real)
* diff drive controller (fake & real)

To do this, we need to create a launch directory in our package.

.. code:: bash

    mkdir launch

We also need to tell CMakeLists.txt to install the launch files. Therefore,
add the launch directory in the install directive.


Launch control simulation
-------------------------

In this section we will create a launch file that will start
a very simple simulation of the system on hardware interface level.

The launch file will start the following nodes:

* controller_manager
* rviz
* joint_state_broadcaster
* robot_state_publisher
* robby_base_controller (diff_drive_controller)

The launch file will use the mock_components\/GenericSystem plugin. This means
that the robot system is only simulated on hardware interface level.

The following launch file will do the trick, paste the contents into a file
called control_simulation.launch.py in the launch directory.

.. code:: python

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, RegisterEventHandler
    from launch.conditions import IfCondition
    from launch.event_handlers import OnProcessExit
    from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare


    def generate_launch_description():
        # Declare arguments
        declared_arguments = []

        # Initialize Arguments
        gui = true
        use_mock_hardware = true

        # Load xacro file
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("my_robot_description"), "urdf", "robby.system.xacro"]
                ),
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
            ]
        )
        robot_description = {"my_robot_description": robot_description_content}

        # Load controllers configuration
        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("my_robot_description"),
                "config/robby",
                "ros2_controllers.yaml",
            ]
        )

        # Launch controller_manager
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output="both",
        )

        # Launch robot_state_publisher
        robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ],
        )
        
        # Launch rviz
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            condition=IfCondition(gui),
        )

        # Spawn joint state broadcaster controller
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

        # Spawn diff drive controller
        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robby_base_controller", "--controller-manager", "/controller_manager"],
        )

        # Delay rviz start after `joint_state_broadcaster`
        delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        )

        # Delay start of robot_controller after `joint_state_broadcaster`
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )

        nodes = [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            keyboard_teleop,
        ]

        return LaunchDescription(declared_arguments + nodes)


You can now build your package and launch the simulation.

.. code:: bash

    colcon build --packages-select my_robot_description
    ros2 launch my_robot_description control_simulation.launch.py

Open another terminal and source your workspace and ros2 installation.

.. code:: bash

    source /opt/ros/rolling/setup.bash
    source ~/ros2_ws/install/setup.bash

Now you can control the robot with the keyboard, if you run the following
command:

.. code:: bash

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

You should see the robot moving in rviz. This simulation is however very
basic and does not include CANopen communication and does not help
with validating the system.


Launch forward controller
-------------------------

In this section we will create a launch file that will start a forward
command controller. This controller will be able to control the robot
by writing velocity to a topic. The launch file will be able to either
run with fake CANopen slaves or with real CANopen slaves.

The launch file will start the following nodes:

* controller_manager
  
  * joint_state_broadcaster
  * robby_forward_controller


* device_container
  
  * master
  * left_drive
  * right_drive
  
* fake_left_drive
* fake_right_drive
* robot_state_publisher
* rviz2

This system can be launch with the following launch file. Paste the
following code into a file called forward_controller.launch.py in the
launch directory.

.. code:: python

  from launch import LaunchDescription
  from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
  from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
  from launch_ros.actions import Node
  from launch_ros.substitutions import FindPackageShare
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from launch.event_handlers import OnProcessExit
  from launch.conditions import UnlessCondition


  def generate_launch_description():

      # Declare arguments
      arg_use_real_hardware = DeclareLaunchArgument(
              "use_real_hardware",
              default_value="false",
              description="Start robot with real hardware.",
      )

      arg_can_interface_name = DeclareLaunchArgument(
              "can_interface_name",
              default_value="vcan0",
              description="Use this can interface for communication.",
      )

      can_interface_name = LaunchConfiguration("can_interface_name")
      use_real_hardware = LaunchConfiguration("use_real_hardware")
      
      # Load xacro file
      robot_description_content = Command(
          [
              PathJoinSubstitution([FindExecutable(name="xacro")]),
              " ",
              PathJoinSubstitution(
                  [
                      FindPackageShare("robby_description"),
                      "urdf",
                      "robby.system.xacro",
                  ]
              ),
              " ",
              "use_mock_hardware:=", "false", " ",
              "can_interface_name:=", can_interface_name
          ]
      )
      robot_description = {"robot_description": robot_description_content}
      
      # Load controllers configuration
      robot_control_config = PathJoinSubstitution(
          [FindPackageShare("robby_description"), "config/robby", "ros2_controllers.yaml"]
      )

      # Launch controller_manager
      control_node = Node(
          package="controller_manager",
          executable="ros2_control_node",
          parameters=[robot_description, robot_control_config],
          output="screen",
      )
      # Spawn joint state broadcaster controller
      joint_state_broadcaster_spawner = Node(
          package="controller_manager",
          executable="spawner",
          arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
      )
      # Spawn forward controller
      robot_controller_spawner = Node(
          package="controller_manager",
          executable="spawner",
          arguments=["robby_forward_controller", "--controller-manager", "/controller_manager"],
      )
      # Launch robot_state_publisher
      robot_state_publisher_node = Node(
          package="robot_state_publisher",
          executable="robot_state_publisher",
          output="both",
          parameters=[robot_description],
      )
      # Launch rviz
      rviz_node = Node(
          package="rviz2",
          executable="rviz2",
          name="rviz2",
          output="log"
      )

      # Load CANopen slave configuration file
      slave_config = PathJoinSubstitution(
          [FindPackageShare("robby_description"), "config/robby", "TMCM-1270.eds"]
      )
      
      # Find CANopen fake slave launch file
      slave_launch = PathJoinSubstitution(
          [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
      )

      # Launch fake slaves for the drives
      slave_node_1 = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(slave_launch),
          launch_arguments={
              "node_id": "3",
              "node_name": "fake_left_drive",
              "slave_config": slave_config,
          }.items(),
          condition=UnlessCondition(use_real_hardware)
      )

      slave_node_2 = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(slave_launch),
          launch_arguments={
              "node_id": "2",
              "node_name": "fake_right_drive",
              "slave_config": slave_config,
          }.items(),
          condition=UnlessCondition(use_real_hardware)
      )

      # Handle launch sequence
      delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=joint_state_broadcaster_spawner,
              on_exit=[robot_controller_spawner],
          )
      )

      delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=joint_state_broadcaster_spawner,
              on_exit=[rviz_node],
          )
      )

      delay_master_launch = TimerAction(
          period=1.0,
          actions=[
              control_node, 
              joint_state_broadcaster_spawner, 
              delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
              delay_rviz_after_joint_state_broadcaster_spawner]
      )

      delay_slave_launch = TimerAction(
          period=2.0,
          actions=[
              slave_node_1,slave_node_2]
      )

      nodes_to_start = [
          arg_can_interface_name,
          arg_use_real_hardware,
          robot_state_publisher_node,
          delay_master_launch,
          delay_slave_launch
      ]

      return LaunchDescription(nodes_to_start)


You can now build your package. If you want to run the launch you will have
to first setup the virtual can bus.

.. code:: bash

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 txqueuelen 1000
    sudo ip link set up vcan0

Then you can run the launch file.

.. code:: bash

    ros2 launch forward_command.launch.py

In another terminal with rolling sourced you can now send commands
to the robot.

.. code:: bash

    ros2 topic pub /robby_forward_controller/commands \
       std_msgs/msg/Float64MultiArray "data: [10.0, 10.0]

In rviz you can now see the wheels moving.


Launch differential drive controller
------------------------------------

In order to run the robot with a differential drive controller, you can create a launch file
called diff_drive.launch.py in the launch directory. Copy the contents of the forward_command.launch.py
into this new file and then change the following lines:

.. code-block:: python
    :emphasize-lines: 4

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robby_forward_controller", "--controller-manager", "/controller_manager"],
    )

Cange ``robby_forward_controller`` to ``robby_base_controller``. Now you can
build your workspace and then your are all setup to control the robot with
a differential drive controller.

.. code:: bash

    ros2 launch my_robot_description diff_drive.launch.py

In another terminal with rolling sourced you can use the ``teleop_twist_keyboard`` node to
control the robot. To do this open another terminal and source rolling.

.. code:: bash

    source /opt/ros/rolling/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robby_base_controller/cmd_vel_unstamped

This is it, you can now run the robot with a differential drive controller.