========================
ROS 1 Interface Overview
========================

Structure
=========

X-Series SDK
------------

The Interbotix X-Series Arms ROS 1 Interface builds primarily on top of the interbotix_xs_sdk
package. This package provides a driver, the **xs_sdk** node, that allows ROS to command, monitor,
and configure the lower-level DYNAMIXEL servos through messages, services, and parameters. A
simulated driver, the **xs_sdk_sim** node allows users to try out their programs in a safe
environment before using them on their real robot.

Publishers
^^^^^^^^^^

*   ``/<robot_name>/joint_states`` - publishes ROS `sensor_msgs/JointState`_ messages at a
    user-desired frequency; note that in general, positions are given in radians, velocities are
    given in radians per second, and effort is given in milliamps. If the robot has a two-finger
    gripper, the positions for the 'right_finger' and 'left_finger' joints are given in meters
    relative to a virtual 'fingers_link' placed dead center (launch a robot model in RViz to get
    familiar with the link and joint names/positions).

    **Simulation**: publishes ROS `sensor_msgs/JointState`_ messages at 20 Hz. Only the 'name' and
    'position' fields in the messages are populated. Otherwise, it operates the same as the
    physical driver.

    .. code-block::
        :caption: `sensor_msgs/JointState`_ definition
        :name: sensor_msgs_JointState

        # This is a message that holds data to describe the state of a set of torque controlled joints.
        #
        # The state of each joint (revolute or prismatic) is defined by:
        #  * the position of the joint (rad or m),
        #  * the velocity of the joint (rad/s or m/s) and
        #  * the effort that is applied in the joint (Nm or N).
        #
        # Each joint is uniquely identified by its name
        # The header specifies the time at which the joint states were recorded. All the joint states
        # in one message have to be recorded at the same time.
        #
        # This message consists of a multiple arrays, one for each part of the joint state.
        # The goal is to make each of the fields optional. When e.g. your joints have no
        # effort associated with them, you can leave the effort array empty.
        #
        # All arrays in this message should have the same size, or be empty.
        # This is the only way to uniquely associate the joint name with the correct
        # states.


        Header header

        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

.. _`sensor_msgs/JointState`: https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/JointState.msg

Subscribers
^^^^^^^^^^^

*   ``/<robot_name>/commands/joint_group`` - subscribes to `interbotix_xs_msgs/JointGroupCommand`_
    messages; this topic is used to control a specified group of joints synchronously (which is
    more efficient than sending commands to individual joints consecutively). Refer to the message
    description for implementation details. Any number of joint groups can be defined in the motor
    config file.

    .. code-block::
        :caption: `interbotix_xs_msgs/JointGroupCommand`_ definition
        :name: interbotix_xs_msgs_JointGroupCommand

        # Command the joints in the specified joint group. Note that the commands are processed differently based on the group's operating mode.
        # For example, if a group's operating mode is set to 'position', the commands are interpreted as positions in radians

        string name            # Name of joint group
        float32[] cmd          # List of joint commands; order is dictated by the index of each joint name for the given group in the 'groups' section of a 'motor_config' yaml file

*   ``/<robot_name>/commands/joint_single`` - subscribes to
    `interbotix_xs_msgs/JointSingleCommand`_ messages; this topic is used to control a single
    joint. Refer to the message description for implementation details.

    .. code-block::
        :caption: `interbotix_xs_msgs/JointSingleCommand`_ definition
        :name: interbotix_xs_msgs_JointSingleCommand

        # Command a desired joint. Note that the command is processed differently based on the joint's operating mode.
        # For example, if a joint's operating mode is set to 'position', the command is interpreted as a position in radians

        string name          # Name of joint
        float32 cmd          # Joint command

*   ``/<robot_name>/commands/joint_trajectory`` - subscribes to
    `interbotix_xs_msgs/JointTrajectoryCommand`_ messages; this topic is used to send desired
    trajectories to a specified joint or joint group. Refer to the message description for
    implementation details.

    .. code-block::
        :caption: `interbotix_xs_msgs/JointTrajectoryCommand`_ definition
        :name: interbotix_xs_msgs_JointTrajectoryCommand

        # Command a joint trajectory to the desired joint(s). Note that the commands are processed differently based on the currently set operating mode.
        # For example, if the operating mode is set to 'position', the commands are interpreted as positions in radians

        string cmd_type                               # set to 'single' for a single joint or 'group' for a group of joints
        string name                                   # joint group name if 'cmd_type' is set to 'group' or joint name if 'cmd_type' is set to 'single'
        trajectory_msgs/JointTrajectory traj          # ROS trajectory message

    **Simulation**: All Subscription topics behave identically to the physical driver ones.

.. _`interbotix_xs_msgs/JointGroupCommand`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/msg/JointGroupCommand.msg
.. _`interbotix_xs_msgs/JointSingleCommand`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/msg/JointSingleCommand.msg
.. _`interbotix_xs_msgs/JointTrajectoryCommand`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/msg/JointTrajectoryCommand.msg

Services
^^^^^^^^

*   ``/<robot_name>/torque_enable`` - service to torque on/off the specified motor or motors; refer
    to the `interbotix_xs_msgs/TorqueEnable`_ service description for implementation details.

    **Simulation**: messages are displayed showing that the desired motors were torqued on/off, but
        nothing actually happens.

    .. code-block::
        :caption: `interbotix_xs_msgs/TorqueEnable`_ definition
        :name: interbotix_xs_msgs_TorqueEnable

        # Torque joints on/off

        string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
        string name              # name of the group if commanding a joint group or joint if commanding a single joint
        bool enable              # set to 'true' to torque on or 'false' to torque off
        ---

*   ``/<robot_name>/reboot_motors`` - service to reboot the specified motor or motors; refer to the
    `interbotix_xs_msgs/Reboot`_ service description for implementation details.

    **Simulation**: messages are displayed showing that the desired motors were rebooted, but
    nothing actually happens.

    .. code-block::
        :caption: `interbotix_xs_msgs/Reboot`_ definition
        :name: interbotix_xs_msgs_Reboot

        # Reboot motors
        #
        # Note that if a dual-joint is selected, both motors will be rebooted. Also note
        # that motors will NOT hold position during the reboot process. Additionally, only
        # EEPROM registers will retain their values, but RAM registers will not. See details
        # at https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#area-eeprom-ram

        string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
        string name              # name of the group if commanding a joint group or joint if commanding a single joint
        bool enable              # whether to torque the selected joints on after reboot
        bool smart_reboot        # set to true to only reboot motors in a specified group that are in an error state
                                 # (as opposed to a blanket reboot of all motors in said group)
        ---

*   ``/<robot_name>/get_robot_info`` - service to get robot information like joint limits, joint
    names, and joint 'sleep' positions; refer to the `interbotix_xs_msgs/RobotInfo`_ service
    description for implementation details.

    .. code-block::
        :caption: `interbotix_xs_msgs/RobotInfo`_ definition
        :name: interbotix_xs_msgs_RobotInfo

        # Get robot information
        #
        # Note that if a 'gripper' joint is specified, all information will be specified in terms of the 'left_finger' joint

        string cmd_type                          # set to 'group' if requesting info about a joint group or 'single' if requesting info about a single joint
        string name                              # the group name if requesting info about a group or the joint name if requesting info about a single joint
        ---
        string mode                              # the operating mode for the specified joint group or joint
        string profile_type                      # the profile type for the specified joint group or joint
        string[] joint_names                     # the name of each joint in a group or just the specified joint
        int16[] joint_ids                        # the Dynamixel ID for each joint in a group or for the specified joint
        float32[] joint_lower_limits             # the lower limit [radians] for each joint in a group or for the specified joint (taken from URDF)
        float32[] joint_upper_limits             # the upper limit [radians] for each joint in a group or for the specified joint (taken from URDF)
        float32[] joint_velocity_limits          # the velocity limit [rad/s] for each joint in a group or for the specified joint (taken from URDF)
        float32[] joint_sleep_positions          # the sleep position [rad] for each joint in a group or for the specified joint
        int16[] joint_state_indices              # index for each joint in a group or for the specified joint in the published JointState message
        int16 num_joints                         # the number of joints in a group or 1
        string[] name                            # the name of the group or joint requested; if group was 'all', returns the names of all groups

*   ``/<robot_name>/set_operating_modes`` - service to set a motor's or multiple motors' operating
    modes (like position, velocity, current, etc...); refer to the
    `interbotix_xs_msgs/OperatingModes`_ service description for implementation details. Note that
    the motors will torque off for a moment when changing operating modes so make sure that the
    robot is in its 'sleep' pose (defined in the motor config file) or otherwise secured before
    calling this service.

    **Simulation**: behaves exactly the same except setting the operating mode just changes how the
    desired motors will be simulated; of course, nothing is torqued off either. Note that
    'current_based_position', 'ext_position', and 'position' are all treated equivalently.
    Similarly, 'pwm' and 'current' modes are also treated the same, and masses/inertias of links
    are NOT considered. Additionally, when in a 'position-type' mode, motors should be using the
    'Time-based Profile' Drive Mode as the 'Profile_Velocity' register is used to determine how
    long a motion should take ('Profile Acceleration' is ignored).

    .. code-block::
        :caption: `interbotix_xs_msgs/OperatingModes`_ definition
        :name: interbotix_xs_msgs_OperatingModes

        # Set Operating Modes
        #
        # To get familiar with the various operating modes, go to...
        # http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/
        # ...click on a motor model, and scroll down to the 'Operating Mode' section.
        #
        # There are 6 valid operating modes. They are...
        #   "position" - allows up to 1 complete joint revolution (perfect for arm joints); units are in radians
        #   "ext_position" - allows up to 512 joint revolutions; units are in radians
        #   "velocity" - allows infinite number of rotations (perfect for wheeled robots); units are in rad/s
        #   "current" - allows infinite number of rotations (perfect for grippers); units are in milliamps
        #   "current_based_position" - allows up to 512 joint revolutions; units are in radians
        #   "pwm" - allows infinite number of rotations (perfect for grippers); units are in PWM
        #
        # Note that the interbotix_xs_sdk offers one other 'pseudo' operating mode that can be useful in controlling Interbotix Grippers - called "linear_position".
        # Behind the scenes, it uses the "position" operating mode mentioned above. The main difference is that with this mode, a desired linear distance [m]
        # between the two gripper fingers can be commanded. In the "position" mode though, only the angular position of the motor can be commanded.
        #
        # There are 2 valid profile types - either 'time' or 'velocity'. Depending on which is chosen, the following parameters behave differently.
        #
        # 1) profile_velocity: acts as a pass-through to the Profile_Velocity register and operates in one of two ways. If
        #    'profile_type' is set to 'velocity', this parameter describes the max velocity limit for the specified joint(s);
        #    for example, if doing 'position' control, setting this to '131' would be equivalent to a limit of 3.14 rad/s; if
        #    'profile_type' is set to 'time', this parameter sets the time span (in milliseconds) that it should take for the
        #    specified joint(s) to move; to have an 'infinite' max limit, set to '0'.
        #
        # 2) profile_acceleration: acts as a pass-through to the Profile_Acceleration register and operates in one of two ways. If
        #    'profile_type' is set to 'velocity', this parameter describes the max acceleration limit for the specified joint(s);
        #    for example, if doing 'position' or 'velocity' control, setting this to '15' would be equivalent to a limit of 5.6 rad/s^2;
        #    if 'profile_type' is set to 'time', this parameter sets the time span (in milliseconds) that it should take for the
        #    specified joint(s) to accelerate; to have an 'infinite' max limit, set to '0'.

        string cmd_type                     # set to 'group' if commanding a joint group or 'single' if commanding a single joint
        string name                         # name of the group if commanding a joint group or joint if commanding a single joint
        string mode                         # desired operating mode as described above
        string profile_type                 # desired 'profile' type - either 'time' or 'velocity' as described above
        int32 profile_velocity              # desired velocity profile as explained above - only used in 'position' or the 'ext_position' control modes
        int32 profile_acceleration          # desired acceleration profile as explained above - used in all modes except for 'current' and 'pwm' control
        ---

*   ``/<robot_name>/set_motor_pid_gains`` - service to set a motor's or multiple motors' internal
    PID gains for position/velocity control; refer to the `interbotix_xs_msgs/MotorGains`_ service
    description for implementation details.

    **Simulation**: doesn't affect anything; no messages are even displayed.

    .. code-block::
        :caption: `interbotix_xs_msgs/MotorGains`_ definition
        :name: interbotix_xs_msgs_MotorGains

        # Set PID gains
        #
        # To get familiar with the various PID gains, go to...
        # http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/
        # ...click on a motor model, and scroll down to the 'PID' section.

        string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
        string name              # name of the group if commanding a joint group or joint if commanding a single joint
        int32 kp_pos             # acts as a pass-through to the Position_P_Gain register
        int32 ki_pos             # acts as a pass-through to the Position_I_Gain register
        int32 kd_pos             # acts as a pass-through to the Position_D_Gain register
        int32 k1                 # acts as a pass-through to the Feedforward_1st_Gain register
        int32 k2                 # acts as a pass-through to the Feedforward_2nd_Gain register
        int32 kp_vel             # acts as a pass-through to the Velocity_P_Gain register
        int32 ki_vel             # acts as a pass-through to the Velocity_I_Gain register
        ---

*   ``/<robot_name>/set_motor_registers`` - service to set a motor's or multiple motors' register
    values at once for a user-provided register name; refer to the
    `interbotix_xs_msgs/RegisterValues`_ service description for implementation details.

    **Simulation**: only works the same if setting the 'Profile_Velocity' or 'Profile_Acceleration'
    registers; otherwise, nothing happens.

*   ``/<robot_name>/get_motor_registers`` - service to get a motor's or multiple motors' register
    values at once for a user-provided register name; refer to the
    `interbotix_xs_msgs/RegisterValues`_ service description for implementation details.

    **Simulation**: only works the same if getting the 'Profile_Velocity' or 'Profile_Acceleration'
    registers; otherwise, an empty service message is returned.

    .. code-block::
        :caption: `interbotix_xs_msgs/RegisterValues`_ definition
        :name: interbotix_xs_msgs_RegisterValues

        # Set or get the register(s) value(s) from motor(s)
        #
        # To get familiar with the register values, go to...
        # http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/
        # ...click on a motor model, and scroll down to the 'Control Table of RAM Area' section.

        string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
        string name              # name of the group if commanding a joint group or joint if commanding a single joint
        string reg               # register name (like Profile_Velocity, Profile_Acceleration, etc...)
        int32 value              # desired register value (only set if 'setting' a register)
        ---
        int32[] values           # current register values (only filled if 'getting' a register)

.. _`interbotix_xs_msgs/TorqueEnable`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/TorqueEnable.srv
.. _`interbotix_xs_msgs/Reboot`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/Reboot.srv
.. _`interbotix_xs_msgs/RobotInfo`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/RobotInfo.srv
.. _`interbotix_xs_msgs/OperatingModes`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/OperatingModes.srv
.. _`interbotix_xs_msgs/MotorGains`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/MotorGains.srv
.. _`interbotix_xs_msgs/RegisterValues`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/RegisterValues.srv

Parameters
^^^^^^^^^^

*   ``motor_configs`` - the file path to the 'motor config' YAML file; refer to the Motor Config
    Template file for details.

*   ``mode_configs`` - the file path to the 'mode config' YAML file; refer to the Mode Config
    Template file for details.

*   ``load_configs`` - a boolean that specifies whether or not the initial register values (under
    the 'motors' heading) in a motor config file should be written to the motors; as the values
    being written are stored in each motor's EEPROM (which means the values are retained even after
    a power cycle), this can be set to 'false' after the first time using the robot. Setting to
    'false' also shortens the node startup time by a few seconds and preserves the life of the
    EEPROM.

    **Simulation**: the load_configs parameter is unused.

.. literalinclude:: https://raw.githubusercontent.com/Interbotix/interbotix_ros_core/main/interbotix_ros_xseries/interbotix_xs_msgs/srv/RobotInfo.srv
