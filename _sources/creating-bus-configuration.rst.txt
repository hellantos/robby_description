ROS 2 CANopen bus configuration
===============================

The CANopen Bus Configuration consists of multiple files that we have to
provide or create:

* Device descriptions (*.eds/*.dcf)
* Bus description

The device descriptions are usually provided by the device manufacturer,
if you buy a CANopen drive make sure it provides a CiA402 interface.
Otherwise the CANopen stack will not be able to use device as a drive.

The bus description is a file that describes the bus of your robot. It
contains the devices that are connected to the bus and details about
how to control them as well as further configuraiton options.

Our first step in creating a bus configuration is to create a config folder
in our package.

.. code:: bash

    cd ~/ros2_ws/src/my_robot_description
    mkdir -p config/robby

Then we are going to copy the device description of our drive into the
the created folder.

.. code:: bash

    cp ~/ros2_ws/src/robby_description/config/robby/TMCM-1270.eds \
      ~/ros2_ws/src/my_robot_description/config/robby/

The next step is to create a bus description file. This is a yaml
file, that needs to be called ``bus.yml`` and needs to be placed in
the ``config/robby`` folder.

.. code:: bash

    touch ~/ros2_ws/src/my_robot_description/config/robby/bus.yml

The bus that we have on our robot is simple. It consists of a master (id=1)
an the two drives (left (id=3) and right (id=2)).

.. code:: yaml

  master:
    node_id: 1

  nodes:
    left_drive:
        node_id: 3
    right_drive:
        node_id: 2

Now we need to define the drivers that should be used for the master and
drives. The master uses the standard master driver provided by the ros2_canopen
stack and the drives use the CiA402 driver.

.. code:: yaml

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"

  defaults:
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"

  nodes:
    left_drive:
        node_id: 3
    right_drive:
        node_id: 2

Now we can add the dcf files to the bus description. As the bus description
is used a compile time to create machine readable configuration files, we need
to as well provide the path to the dcf files.

.. code:: yaml

  options:
    dcf_path: "@BUS_CONFIG_PATH@"

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"

  defaults:
    dcf: "TMCM-1270.eds"
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"

  nodes:
    left_drive:
        node_id: 3
    right_drive:
        node_id: 2

Next we can configure the CiA402 driver a bit more. Basically we want 
the driver run with a frequency of 50 hz and we want to use the profile
position mode as position mode and profile velocity mode as velocity mode.

.. code:: yaml

  options:
    dcf_path: "@BUS_CONFIG_PATH@"

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"

  defaults:
    dcf: "TMCM-1270.eds"
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"
    polling: true
    period: 20
    position_mode: 1
    velocity_mode: 3

  nodes:
    left_drive:
        node_id: 3
    right_drive:
        node_id: 2

We need to as well do some device specific configurations, that will base
executed during startup of the device (PREOP state).

.. code:: yaml

  options:
    dcf_path: "@BUS_CONFIG_PATH@"

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"

  defaults:
    dcf: "TMCM-1270.eds"
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"
    polling: true
    period: 20
    position_mode: 1
    velocity_mode: 3
    heartbeat_producer: 1000 # Heartbeat every 1000 ms
    sdo: # SDO executed during config
      - {index: 0x2000, sub_index: 0, value: 3}       # Microstep resolution (8 per step)
      - {index: 0x6081, sub_index: 0, value: 1600}    # Set velocity
      - {index: 0x6083, sub_index: 0, value: 16000}   # Set acceleration
      - {index: 0x6083, sub_index: 0, value: 16000}   # Set deceleration
      - {index: 0x6085, sub_index: 0, value: 16000}   # Set quickstop deceleration

  nodes:
    left_drive:
        node_id: 3
    right_drive:
        node_id: 2

For the CiA402 driver to work efficiently with profile velocity and position mode we need to configure the PDOs.
At least the follfowing objects need to be mapped on an active PDO:

.. csv-table:: 
  :delim: ;

  Index; Subindex; Name; Size (Bits)
  0x6040; 0; Controlword; 16
  0x6060; 0; Modes of operation; 8
  0x607A; 0; Target position; 32
  0x60FF; 0; Target velocity; 32
  0x6041; 0; Statusword; 16
  0x6061; 0; Modes of operation display; 8
  0x6064; 0; Position actual value; 32
  0x606C; 0; Velocity actual value; 32

The PDOs are configured to be sent on every SYNC message. The master issues sync every 20 ms.

.. code-block:: yaml
  :emphasize-lines: 8, 26 - 53

  options:
    dcf_path: "@BUS_CONFIG_PATH@"

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"
    sync_period: 20000

  defaults:
    dcf: "TMCM-1270.eds"
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"
    polling: true
    period: 20
    position_mode: 1
    velocity_mode: 3
    heartbeat_producer: 1000 # Heartbeat every 1000 ms
    sdo: # SDO executed during config
      - {index: 0x2000, sub_index: 0, value: 3}       # Microstep resolution (8 per step)
      - {index: 0x6081, sub_index: 0, value: 1600}    # Set velocity
      - {index: 0x6083, sub_index: 0, value: 16000}   # Set acceleration
      - {index: 0x6083, sub_index: 0, value: 16000}   # Set deceleration
      - {index: 0x6085, sub_index: 0, value: 16000}   # Set quickstop deceleration

    tpdo: # TPDO needed statusword, actual velocity, actual position, mode of operation
      1:
        enabled: true
        transmission: 0x01
        mapping:
          - {index: 0x6041, sub_index: 0} # status word
          - {index: 0x6061, sub_index: 0} # mode of operation display
      2:
        enabled: true
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
        mapping:
        - {index: 0x6040, sub_index: 0} # controlword
        - {index: 0x6060, sub_index: 0} # mode of operation
      2:
        enabled: true
        mapping:
        - {index: 0x607A, sub_index: 0} # target position
        - {index: 0x60FF, sub_index: 0} # target velocity
  nodes:
    left_drive:
        node_id: 3
    right_drive:
        node_id: 2


Our device is a stepper motor with 200 steps per revolution and 8 Microsteps
per step. This means that we have 1600 steps per revolution. The drives
use microsteps as unit for position and velocity and acceleration. This
means that we need to convert our values (radians) to microsteps.

To the device: ``x radians * 1600 / (2 * pi) = x * 254,64 microsteps``

From the device: ``x microsteps * (2 * pi) / 1600 = x * 0.00392 radians``

In addition, the left wheel needs to turn in opposite direction to the right wheel.
We can adjust this by passing scaling factors to the driver.


.. code-block:: yaml
  :emphasize-lines: 57-60, 63-66

  options:
    dcf_path: "@BUS_CONFIG_PATH@"

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"
    sync_period: 20000

  defaults:
    dcf: "TMCM-1270.eds"
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"
    polling: true
    period: 20
    position_mode: 1
    velocity_mode: 3
    heartbeat_producer: 1000 # Heartbeat every 1000 ms
    sdo: # SDO executed during config
      - {index: 0x2000, sub_index: 0, value: 3}       # Microstep resolution (8 per step)
      - {index: 0x6081, sub_index: 0, value: 1600}    # Set velocity
      - {index: 0x6083, sub_index: 0, value: 16000}   # Set acceleration
      - {index: 0x6083, sub_index: 0, value: 16000}   # Set deceleration
      - {index: 0x6085, sub_index: 0, value: 16000}   # Set quickstop deceleration

    tpdo: # TPDO needed statusword, actual velocity, actual position, mode of operation
      1:
        enabled: true
        transmission: 0x01
        mapping:
          - {index: 0x6041, sub_index: 0} # status word
          - {index: 0x6061, sub_index: 0} # mode of operation display
      2:
        enabled: true
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
        mapping:
        - {index: 0x6040, sub_index: 0} # controlword
        - {index: 0x6060, sub_index: 0} # mode of operation
      2:
        enabled: true
        mapping:
        - {index: 0x607A, sub_index: 0} # target position
        - {index: 0x60FF, sub_index: 0} # target velocity
  nodes:
    left_drive:
        node_id: 3
        scale_pos_from_dev: -0.003926991
        scale_pos_to_dev: -254.647897079
        scale_vel_from_dev: -0.003926991
        scale_vel_to_dev: -254.647897079
    right_drive:
        node_id: 2
        scale_pos_from_dev: 0.003926991
        scale_pos_to_dev: 254.647897079
        scale_vel_from_dev: 0.003926991
        scale_vel_to_dev: 254.647897079

Now we have completed the bus configuration.
We need to now tell the build system to use the bus configuration.
In the ``CMakeLists.txt`` of our package we need to add the following line after the ``find_package`` call.

.. code-block:: cmake

  cogen_dcf(robby)

Now we can build the package and the bus configuration will be compiled.