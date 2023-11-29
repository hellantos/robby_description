Preparing the system
====================

In this master class we will use the ros2_canopen stack to build a simple
mobile robot with differential drive. In order to do this, we need to prepare
our system first.

First we need to create a new workspace for our project.

.. code-block:: bash

  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src


Next we need to clone the ros2_canopen stack into our workspace.

.. code-block:: bash

  git clone https://github.com/ros-industrial/ros2_canopen.git
  git clone https://github.com/ipa-cmh/robby_description.git
  git clone https://github.com/ros-industrial/ros2_canopen.git

Now we can build the ROS 2 CANopen stack.

.. code-block:: bash

  cd ~/ros2_ws
  source /opt/ros/rolling/setup.bash
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  colcon build
  source install/setup.bash

This is all we have to install for now.


