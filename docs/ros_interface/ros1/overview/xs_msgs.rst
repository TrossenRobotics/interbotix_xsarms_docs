==================
interbotix_xs_msgs
==================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_core/tree/main/interbotix_ros_xseries/interbotix_xs_msgs"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../../../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

The interbotix_xs_msgs package defines and builds the common messages, services, and actions used
by the Interbotix X-Series platforms. This guide details those interfaces relevant to the X-Series
Arms.

Usage
=====

Simply build this package and include any generated message in your own custom node or script. For
example, if you wanted to reboot a servo in an X-Series manipulator, you would include the Reboot
service in your program's header.

*   C++

    .. code-block:: c++

        #include "interbotix_xs_msgs/Reboot.h"

*   Python

    .. code-block:: python

        from interbotix_xs_msgs.srv import Reboot

Structure
=========

Messages
--------

.. _interbotix_xs_msgs_JointGroupCommand_ros1:

JointGroupCommand
^^^^^^^^^^^^^^^^^

Command the joints in the specified joint group. Note that the commands are processed differently
based on the group's operating mode. For example, if a group's operating mode is set to 'position',
the commands are interpreted as positions in radians


.. collapse:: JointGroupCommand Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/msg/JointGroupCommand.msg
        :lines: 4-

.. _interbotix_xs_msgs_JointSingleCommand_ros1:

JointSingleCommand
^^^^^^^^^^^^^^^^^^

Command a desired joint. Note that the command is processed differently based on the joint's
operating mode. For example, if a joint's operating mode is set to 'position', the command is
interpreted as a position in radians

.. collapse:: JointSingleCommand Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/msg/JointSingleCommand.msg
        :lines: 4-

.. _interbotix_xs_msgs_JointTrajectoryCommand_ros1:

JointTrajectoryCommand
^^^^^^^^^^^^^^^^^^^^^^

Command a joint trajectory to the desired joint(s). Note that the commands are processed
differently based on the currently set operating mode. For example, if the operating mode is set to
'position', the commands are interpreted as positions in radians. This message wraps the
`trajectory_msgs/JointTrajectory`_ message.

.. collapse:: JointTrajectoryCommand Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/msg/JointTrajectoryCommand.msg
        :lines: 4-

.. _`trajectory_msgs/JointTrajectory`: https://docs.ros.org/en/latest/api/trajectory_msgs/html/msg/JointTrajectory.html

Services
--------

.. _interbotix_xs_msgs_TorqueEnable_ros1:

TorqueEnable
^^^^^^^^^^^^

Torque a joint or group of joints on/off.

.. collapse:: TorqueEnable Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/srv/TorqueEnable.srv
        :lines: 3-

.. warning::

    The specified motors will torque off and the robot may collapse when this service is called.
    Make sure the robot is in its sleep pose or in a safe configuration before calling it.

.. _interbotix_xs_msgs_Reboot_ros1:

Reboot
^^^^^^

Reboot a joint or group of joints.

.. warning::

    If a dual-joint is selected, both motors will be rebooted.

.. warning::

    The specified motors will torque off and the robot may collapse when this service is called.
    Make sure the robot is in its sleep pose or in a safe configuration before calling it.

.. warning::

    Only EEPROM registers will retain their values, but RAM registers will not. See details on the
    RAM and EEPROM Control Tables for your specific motors, for example, the `XM430-W350`_.

.. collapse:: Reboot Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/srv/Reboot.srv
        :lines: 8-

.. _`XM430-W350`: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#area-eeprom-ram

.. _interbotix_xs_msgs_RobotInfo_ros1:

RobotInfo
^^^^^^^^^

Get information about a joint, group of joints, or all joints on the robot.

.. note::

    If a 'gripper' joint is specified, all information will be specified in terms of the
    'left_finger' joint


.. collapse:: RobotInfo Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/srv/RobotInfo.srv
        :lines: 5-

.. _interbotix_xs_msgs_OperatingModes_ros1:

OperatingModes
^^^^^^^^^^^^^^

Used to set Operating Modes on a joint or group of joints.

To get familiar with the various operating modes, go to the `DYNAMIXEL Workbench E-Manual page
<http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/>`_, click on a
motor model, and scroll down to the 'Operating Mode' section.

There are 6 valid operating modes:

*   "position" - allows up to 1 complete joint revolution (perfect for arm joints); units are in
    radians
*   "ext_position" - allows up to 512 joint revolutions; units are in radians
*   "velocity" - allows infinite number of rotations (perfect for wheeled robots); units are in
    rad/s
*   "current" - allows infinite number of rotations (perfect for grippers); units are in milliamps
*   "current_based_position" - allows up to 512 joint revolutions; units are in radians
*   "pwm" - allows infinite number of rotations (perfect for grippers); units are in PWM

Note that the **interbotix_xs_sdk** offers one other 'pseudo' operating mode that can be useful in
controlling Interbotix Grippers - called "linear_position". Behind the scenes, it uses the
"position" operating mode mentioned above. The main difference is that with this mode, a desired
linear distance [m] between the two gripper fingers can be commanded. In the "position" mode
though, only the angular position of the motor can be commanded.

There are 2 valid profile types - either 'time' or 'velocity'. Depending on which is chosen, the
following parameters behave differently.

*   profile_velocity: acts as a pass-through to the Profile_Velocity register and operates in one
    of two ways. If 'profile_type' is set to 'velocity', this parameter describes the max velocity
    limit for the specified joint(s); for example, if doing 'position' control, setting this to
    '131' would be equivalent to a limit of 3.14 rad/s; if 'profile_type' is set to 'time', this
    parameter sets the time span (in milliseconds) that it should take for the specified joint(s)
    to move; to have an 'infinite' max limit, set to '0'.

*   profile_acceleration: acts as a pass-through to the Profile_Acceleration register and operates
    in one of two ways. If 'profile_type' is set to 'velocity', this parameter describes the max
    acceleration limit for the specified joint(s); for example, if doing 'position' or 'velocity'
    control, setting this to '15' would be equivalent to a limit of 5.6 rad/s^2; if 'profile_type'
    is set to 'time', this parameter sets the time span (in milliseconds) that it should take for
    the specified joint(s) to accelerate; to have an 'infinite' max limit, set to '0'.

.. collapse:: OperatingModes Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/srv/OperatingModes.srv
        :lines: 33-

.. _interbotix_xs_msgs_MotorGains_ros1:

MotorGains
^^^^^^^^^^

Used to set PID gains on a joint or group of joints.

To get familiar with the various PID gains, go to the `DYNAMIXEL Workbench E-Manual page
<http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/>`_, click on a
motor model, and scroll down to the 'PID' section.

.. collapse:: MotorGains Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/srv/MotorGains.srv
        :lines: 7-

.. _interbotix_xs_msgs_RegisterValues_ros1:

RegisterValues
^^^^^^^^^^^^^^

Used to set or get the register(s) value(s) from a joint or group of joints.

To get familiar with the register values, go to the `DYNAMIXEL Workbench E-Manual page
<http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/>`_, click on a
motor model, and scroll down to the 'Control Table of RAM Area' section.

.. collapse:: RegisterValues Definition

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs/srv/RegisterValues.srv
        :lines: 7-
