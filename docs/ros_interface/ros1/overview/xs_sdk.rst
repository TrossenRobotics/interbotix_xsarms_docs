=================
interbotix_xs_sdk
=================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_core/tree/main/interbotix_ros_xseries/interbotix_xs_sdk"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../../../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

The Interbotix X-Series Arms ROS 1 Interface builds primarily on top of the interbotix_xs_sdk
package. This package provides a driver, the **xs_sdk** node, that allows ROS to command, monitor,
and configure the lower-level DYNAMIXEL servos through messages, services, and parameters. A
simulated driver, the **xs_sdk_sim** node allows users to try out their programs in a safe
environment before using them on their real robot.

Publishers
==========

JointState Publisher
--------------------

The JointState Publisher ROS `sensor_msgs/JointState`_ messages at a user-desired frequency. In
general, positions are given in radians, velocities are given in radians per second, and effort is
given in milliamps. If the robot has a two-finger gripper, the positions for the 'right_finger' and
'left_finger' joints are given in meters relative to a virtual 'fingers_link' placed dead center.
You can launch a robot model in RViz to get familiar with the link and joint names/positions.

*   **Topic**: ``/<robot_name>/joint_states``
*   **Message Type**: `sensor_msgs/JointState`_
*   **Simulation Differences**: publishes ROS `sensor_msgs/JointState`_ messages at 20 Hz. Only the
    'name' and 'position' fields in the messages are populated. Otherwise, it behaves identically
    to the physical driver.

.. _`sensor_msgs/JointState`: https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/JointState.msg

Subscribers
===========

Joint Group Command Subscriber
------------------------------

The Joint Group Command Subscriber subscribes to interbotix_xs_msgs/JointGroupCommand messages;
this topic is used to control a specified group of joints synchronously (which is more efficient
than sending commands to individual joints consecutively). Refer to the message definition for
implementation details. Any number of joint groups can be defined in the motor config file.

*   **Topic**: ``/<robot_name>/commands/joint_group``
*   **Message Type**: :ref:`interbotix_xs_msgs/JointGroupCommand <interbotix_xs_msgs_JointGroupCommand_ros1>`
*   **Simulation Differences**: Behaves identically to the physical driver.

Joint Single Command Subscriber
-------------------------------

The Joint Single Command Subscriber subscribes to interbotix_xs_msgs/JointSingleCommand messages.
This topic is used to command a single joint. Refer to the message definition for implementation
details.

*   **Topic**: ``/<robot_name>/commands/joint_single``
*   **Message Type**: :ref:`interbotix_xs_msgs/JointSingleCommand <interbotix_xs_msgs_JointSingleCommand_ros1>`
*   **Simulation Differences**: Behaves identically to the physical driver.

Joint Trajectory Command Subscriber
-----------------------------------

Joint Trajectory Command Subscriber subscribes to interbotix_xs_msgs/JointTrajectoryCommand
messages; this topic is used to send desired trajectories to a specified joint or joint group.
Refer to the message definition for implementation details.

*   **Topic**:  ``/<robot_name>/commands/joint_trajectory``
*   **Message Type**: :ref:`interbotix_xs_msgs/JointTrajectoryCommand <interbotix_xs_msgs_JointTrajectoryCommand_ros1>`
*   **Simulation Differences**: Behaves identically to the physical driver.

Services
========

Enable/Disable Torque Service
-----------------------------

The Enable/Disable Torque Service to torque on/off the specified motor or motors.

*   **Topic**: ``/<robot_name>/torque_enable``
*   **Service Type**: :ref:`interbotix_xs_msgs/TorqueEnable <interbotix_xs_msgs_TorqueEnable_ros1>`
*   **Simulation Differences**: messages are displayed showing that the desired motors were torqued
    on/off, but nothing actually happens.

.. warning::

    The specified motors will torque off and the robot may collapse when this service is called.
    Make sure the robot is in its sleep pose or in a safe configuration before calling it.

Reboot Motors Service
---------------------

The Reboot Motors Service reboots the specified motor or motors.

*   **Topic**: ``/<robot_name>/reboot_motors``
*   **Service Type**: :ref:`interbotix_xs_msgs/Reboot <interbotix_xs_msgs_Reboot_ros1>`
*   **Simulation Differences**: messages are displayed showing that the desired motors were rebooted,
    but nothing actually happens.

.. warning::

    The specified motors will torque off and the robot may collapse when this service is called.
    Make sure the robot is in its sleep pose or in a safe configuration before calling it.

Get Robot Info Service
----------------------

The Get Robot Info Service service to get robot information like joint limits, joint names, and
joint 'sleep' positions.

*   **Topic**: ``/<robot_name>/get_robot_info``
*   **Service Type**: :ref:`interbotix_xs_msgs/RobotInfo <interbotix_xs_msgs_RobotInfo_ros1>`
*   **Simulation Differences**: Behaves identically to the physical driver.

Set Operating Modes Service
---------------------------

The Set Operating Modes Service sets a motor's or multiple motors' operating modes (like position,
velocity, current, etc...).

*   **Topic**: ``/<robot_name>/set_operating_modes``
*   **Service Type**: :ref:`interbotix_xs_msgs/OperatingModes <interbotix_xs_msgs_OperatingModes_ros1>`
*   **Simulation Differences**: behaves exactly the same except setting the operating mode just
*   changes how the desired motors will be simulated. Of course, nothing is torqued off either.
    Note that 'current_based_position', 'ext_position', and 'position' are all treated
    equivalently. Similarly, 'pwm' and 'current' modes are also treated the same, and
    masses/inertias of links are NOT considered. Additionally, when in a 'position-type' mode,
    motors should be using the 'Time-based Profile' Drive Mode as the 'Profile_Velocity' register
    is used to determine how long a motion should take ('Profile Acceleration' is ignored).

.. warning::

    The motors will torque off for a moment when changing operating modes so make sure that the
    robot is in its 'sleep' pose (defined in the :ref:`motor config file
    <motor_configs_file_ros1>`) or otherwise secured before calling this service.

Set Motor Gains Service
-----------------------

*   The Set Motor Gains Service service to set a motor's or multiple motors' internal
    PID gains for position/velocity control; refer to the `interbotix_xs_msgs/MotorGains` service
    description for implementation details.

*   **Topic**: ``/<robot_name>/set_motor_pid_gains``
*   **Service Type**: :ref:`interbotix_xs_msgs/MotorGains <interbotix_xs_msgs_MotorGains_ros1>`
*   **Simulation Differences**: doesn't affect anything; no messages are even displayed.
*   **Example Usage**: the below example calls the ``set_motor_pid_gains`` service to set the gains
    of each servo in the ``arm`` group to its default value.

    .. tabs::

        .. group-tab:: Python

            .. code-block:: python

                import rospy
                from interbotix_xs_msgs.srv import MotorGains, MotorGainsRequest
                srv_set_motor_gains = rospy.ServiceProxy(
                    name='set_motor_pid_gains',
                    service_class=MotorGains
                )
                gains_request = MotorGainsRequest(
                    cmd_type='group',
                    name='arm',
                    kp_pos=800,
                    ki_pos=0,
                    kd_pos=0,
                    k1=0,
                    k2=0,
                    kp_vel=100,
                    ki_vel=1920,
                )
                srv_set_motor_gains.call(gains_request)

        .. group-tab:: C++

            .. code-block:: c++

                ros::ServiceClient srv_motor_gains = nh.serviceClient<interbotix_xs_msgs::MotorGains>("set_motor_pid_gains");
                srv_motor_gains.waitForExistence();
                interbotix_xs_msgs::MotorGains motor_gains;
                motor_gains.request.cmd_type = "group";
                motor_gains.request.name = "arm";
                motor_gains.request.kp_pos = 800;
                motor_gains.request.ki_pos = 0;
                motor_gains.request.kd_pos = 0;
                motor_gains.request.k1 = 0;
                motor_gains.request.k2 = 0;
                motor_gains.request.kp_vel = 100;
                motor_gains.request.ki_vel = 1920;
                srv_motor_gains.call(motor_gains);

Set Register Values Service
---------------------------

The Set Register Values Service sets a motor's or multiple motors' register values simultaneously
for a user-provided register name.

*   **Topic**: ``/<robot_name>/set_motor_registers``
*   **Service Type**: :ref:`interbotix_xs_msgs/RegisterValues <interbotix_xs_msgs_RegisterValues_ros1>`
*   **Simulation Differences**: only works the same if setting the 'Profile_Velocity' or
    'Profile_Acceleration' registers; otherwise, nothing happens.

Get Register Values Service
---------------------------

The Get Register Values Service gets a motor's or multiple motors' register values simultaneously
for a user-provided register name.

*   **Topic**: ``/<robot_name>/get_motor_registers``
*   **Service Type**: :ref:`interbotix_xs_msgs/RegisterValues <interbotix_xs_msgs_RegisterValues_ros1>`
*   **Simulation Differences**: only works the same if getting the 'Profile_Velocity' or
    'Profile_Acceleration' registers; otherwise, an empty service message is returned.
*   **Example Usage**: the below example calls the ``get_motor_registers`` service to get the
*   ``Present_Temperature`` register value of each servo in the ``arm`` group.

    .. tabs::

        .. group-tab:: Python

            .. code-block:: python

                import rospy
                from interbotix_xs_msgs.srv import RegisterValues, RegisterValuesRequest
                srv_get_motor_registers = rospy.ServiceProxy(
                    name='get_motor_registers',
                    service_class=RegisterValues
                )
                gains_request = RegisterValuesRequest(
                    cmd_type='group',
                    name='arm',
                    reg='Present_Temperature'
                )
                srv_get_motor_registers.call(gains_request)

        .. group-tab:: C++

            .. code-block:: c++

                ros::ServiceClient srv_get_motor_registers = nh.serviceClient<interbotix_xs_msgs::RegisterValues>("get_motor_registers");
                srv_get_motor_registers.waitForExistence();
                interbotix_xs_msgs::RegisterValues register_values;
                register_values.request.cmd_type = "group";
                register_values.request.name = "arm";
                register_values.request.reg = "Present_Temperature";
                srv_get_motor_registers.call(register_values);

Gripper Calibration Service
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Gripper Calibration Service gets the calibration offset value of a specific gripper name.

*   **Topic**: ``/<robot_name>/gripper_calibration``
*   **Service Type**: `interbotix_xs_msgs/GripperCalib <https://github.com/Interbotix/interbotix_ros_core/blob/devel/interbotix_ros_xseries/interbotix_xs_msgs/srv/GripperCalib.srv>`_
*   **Simulation Differences**: behaves exactly the same in simulation. this routine is executed upon startup
    of the SDK.
*   **Service Definition**: `InterbotixRobotXS::robot_srv_gripper_calib <https://github.com/Interbotix/interbotix_ros_core/blob/77ebd0c13a778111e36eb83710a8020cc9303d99/interbotix_ros_xseries/interbotix_xs_sdk/src/xs_sdk_obj.cpp#L1097>`_


.. warning::

    This service is for internal use to enable communication between gripper_calib.cpp script and the SDK. It must not be called manually for executing gripper calibration. For executing Gripper Calibration check out the next section.

Gripper Calibration Routine
---------------------------

Executes the gripper calibration node in the SDK.
This node is called upon startup and performs the gripper calibration operation
to derive a constant offset value for the gripper.

`Gripper Calibration Node  <https://github.com/Interbotix/interbotix_ros_core/blob/devel/interbotix_ros_xseries/interbotix_xs_sdk/src/gripper_calib.cpp>`_

**Algorithm:**

#.  Apply a PWM value to the gripper actuator to move it inwards.
#.  Calculate the error between the previous position and the current position of the gripper.
#.  Repeat steps 1 and 2 until the gripper reaches its minimum position.
#.  When there is no longer an error between the current and previous positions, stop the gripper and use the current position value as the offset.
#. Send the offset and gripper name to the SDK using the Gripper Calibration Service.
#. The SDK uses this offset value to map between the minimum and maximum position values.

Parameters
==========

.. _motor_configs_param_ros1:

``motor_configs``
-----------------

The file path to the 'motor config' YAML file. Refer to the template below for details.

.. _mode_configs_param_ros1:

``mode_configs``
----------------

The file path to the 'mode config' YAML file. Refer to the template below for details.

.. _load_configs_param_ros1:

``load_configs``
----------------

A boolean that specifies whether or not the initial register values (under the 'motors' heading) in
a motor config file should be written to the motors; as the values being written are stored in each
motor's EEPROM (which means the values are retained even after a power cycle), this can be set to
``false`` after the first time using the robot. Setting to ``false`` also shortens the node startup
time by a few seconds and preserves the life of the EEPROM.

*   **Simulation Differences**: the ``load_configs`` parameter is unused.
