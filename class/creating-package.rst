ROS 2 configuration package
===========================

In order to use the ROS 2 CANopen stack, a configuration package is required.
This package will contain the necessary configuration files to describe the
robot that you are building. This includes files for the
following:

- CANopen devices
- CANopen bus
- Hardware description
- Hardware interface description
- Controller configuration
- Launch files

This chapter will teach you how to create the configuration package for your
robot. The following chapters will then explain how to create the individual
configuration files.

Creating a new package
----------------------

The first step is to create a new package for your robot. This can be done
using the following command:

.. code:: bash

    ros2 pkg create --build-type ament_cmake my_robot_description \
        --dependencies lely_core_libraries --license Apache-2.0

Now you have a package with the following content:

.. code:: bash

    my_robot_description
    ├── CMakeLists.txt
    ├── LICENSE
    ├── include
    │   └── my_robot_description
    ├── package.xml
    └── src
     
As we are creating a description package, we will not need the ``src`` and
``include`` directories. You can remove them using the following commands:

.. code:: bash

    rm -r src
    rm -r include

Now we will need to add the following dependencies to the ``package.xml`` file
(I did not to an extensive dependency analysis, we will discover the missing ones
on the go).

.. code:: xml

    <depend>diff_drive_controller</depend>
    <depend>forward_command_controller</depend>
    <depend>controller_manager</depend>
    <depend>robot_state_publisher</depend>
    <depend>joint_state_broadcaster</depend>
    <depend>rviz2</depend>

Now we are all set an can start creating the configuration files.

