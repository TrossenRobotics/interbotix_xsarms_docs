======================
ROS 2 Quickstart Guide
======================

This guide is intended to get the use familiar with the basic functions and interfaces of the ROS 2
Interface.

1.  Get familiar with the virtual robot model by launching it in RViz and playing with the
    `joint_state_publisher`. Note that you must specify which arm model is being used as a command
    line argument. For example, the WidowX-200 robot arm can be launched with the command below.
    Make sure to press :kbd:`Ctrl` + :kbd:`C` in the terminal when to end the session you're done.

    .. code-block:: console

        $ ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=wx200 use_joint_pub_gui:=true

.. image:: images/rviz_remote.png
    :align: center
    :width: 70%

2.  Get familiar with the physical robot arm (we'll use a ViperX-250 as an example) by executing
    the following command in the terminal:

    .. code-block:: console

        $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx250

3.  By default, all the motors in the robot are torqued on so it will be very difficult to manually
    manipulate it. To torque off all the motors, execute the command below in another terminal.

    .. warning::

        This command will cause the robot arm to collapse (if it's not already resting) so manually
        hold or secure it before executing.

    .. code-block:: console

        $ ros2 service call /vx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"

    .. note::

        The command above torques off every motor in the ``all`` group. This is a special group
        that includes every Dynamixel motor in the manipulator. To only torque off the arm motors,
        change the name from ``all`` to ``arm``.

        .. list-table::
            :header-rows: 1
            :align: center
            :widths: 10 40

            * - Group Name
              - Servos in Group
            * - ``all``
              - Every DYNAMIXEL servo on the robot
            * - ``arm``
              - All DYNAMIXEL servos on the arm excluding the gripper

4.  Now you should be able to freely manipulate the arm and gripper. Take note of how the RViz
    model accurately mimics the real robot. To make the robot hold a certain pose, manually hold
    the robot in the desired pose and execute the following command:

    .. code-block:: console

        $ ros2 service call /vx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"

5.  Let go and observe how the arm stays in place.

6.  Hold on to the robot and shutdown all nodes by pressing :kbd:`Ctrl` + :kbd:`C` in the terminal
    where you started the launch file.

    .. warning::

        Ending the control process will cause the robot arm to collapse (if it's not already
        resting) so manually hold or secure it before executing.

7.  Another way to check if all functions work is to launch the `interbotix_xsarm_joy` package.
    This package allows you to control your arm using a Bluetooth controller. See it's usage
    details in `its documentation page`_.

.. _`its documentation page`: /ros2_packages/joystick_control.html

That ends the quickstart tutorial. To get familiar with the architecture and launch file arguments,
refer to the READMEs of the core packages. Start with the :doc:`X-Series Arm Descriptions
</ros2_packages/arm_descriptions>` package, then the :doc:`X-Series Arm Control
</ros2_packages/arm_control>` package. Next, look at the :doc:`Gazebo Configuration
</ros2_packages/simulation_configuration>` package followed by the :doc:`ROS Controllers
Configuration </ros2_packages/ros_control>` and :doc:`MoveIt Configuration
</ros2_packages/moveit_motion_planning_configuration>` packages. This is the most logical approach
to take to gain a better understanding of how they relate to each other.

Afterwards, feel free to check out the demo projects like :doc:`Joystick Control
</ros2_packages/joystick_control>`, or any of the other :doc:`ROS 2 Open Source Packages
</ros2_packages>`.

Video Tutorials
===============

ROS 2 Quickstart
----------------

.. youtube:: vGCIU6CX72M
    :width: 70%
    :align: center
