====================
Python-ROS Interface
====================

.. note::

    The Interbotix Python-ROS Interface requires that the :doc:`ROS Interface
    <ros_interface>` has already been installed.

For usage and structure information on the Python interface that builds on top of ROS, check out
the :doc:`Python Demos for ROS 1<ros1_packages/python_demos>` or the :doc:`Python Demos for ROS
2<ros2_packages/python_demos>` pages. Further documentation of the Python API's functionality can
be found on this page. Note that you can check the source code methods' docstrings for information
on each method.

.. attention::

    The Python-ROS API is not compatible with Gazebo-simulation robots.

.. TODO: include the different modules here from interbotix_xs_modules

Terminology
===========

Transforms
----------

.. image:: python_ros_interface/images/xsarm_demos_frames.png
    :align: center
    :width: 40%

.. glossary::

    Body Frame
        The frame at `/<robot_name>/ee_gripper_link`

    Space Frame
        The frame at `/<robot_name>/base_link`

    T_sb
        End-effector poses are specified from the :term:`Body Frame` to the :term:`Space Frame`. In
        the code documentation, this transform is knows as **T_sb** (i.e. the transform that
        specifies the :term:`Body Frame` :math:`b` in terms of the :term:`Space Frame` :math:`s`).
        In the image above, you can see both of these frames. The X axes are in red, the Y axes are
        in green, and the Z axes are in blue. The rotation and translation information is stored in
        a `homogeneous transformation matrix`_.

.. _`homogeneous transformation matrix`: https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/

.. math::

    T =
    \begin{bmatrix}
        R & p \\
        0 & 1
    \end{bmatrix}
    =
    \begin{bmatrix}
        r_{11} & r_{12} & r_{13} & p_1 \\
        r_{21} & r_{22} & r_{23} & p_2 \\
        r_{31} & r_{32} & r_{33} & p_3 \\
        0     & 0      & 0       & 1 \\
    \end{bmatrix}

In a homogeneous transformation matrix, the first three rows and three columns :math:`R` define a
3-dimensional rotation matrix that describes the orientation of the :term:`Body Frame` with respect
to the :term:`Space Frame`. The first three rows and the fourth column :math:`p` of the matrix
represent the translational position (i.e. xyz) of the :term:`Body Frame` with respect to the
:term:`Space Frame`. The fourth row of the matrix is always [0 0 0 1] for matrix multiplication
purposes.

You will see two other homogeneous transformation matrices in the code:

.. glossary::

    T_sd
        Defines the desired end-effector pose with respect to the :term:`Space Frame`. This
        transformation is used in methods like ``set_ee_pose_matrix``, where a single desired pose
        is to be solved for.

    T_sy
        The transform from the :term:`Space Frame` to a virtual frame with the exact same x, y, z,
        roll, and pitch as the :term:`Space Frame`. However, it contains the 'yaw' of the
        :term:`Body Frame`. Thus, if the end-effector is located at xyz = [0.2, 0.2, 0.2] with
        respect to the :term:`Space Frame`, this converts to xyz = [0.2828, 0, 0.2] with respect to
        the virtual frame of the :term:`T_sy` transformation. This convention helps simplify how
        you think about the relative movement of the end-effector. The method
        ``set_ee_cartesian_trajectory`` uses :term:`T_sy` to command relative movement of the
        end-effector using the end-effector's yaw as a basis for its frame of reference.

Timing Parameters
-----------------

The Python API uses five different timing parameters to shape the time profile of movements.

The first two parameters are used to determine the time profile of the arm when completing moves
from one pose to another. These can be set in the constructor of the object, or by using the
``set_trajectory_time`` method.

.. glossary::

    moving_time
        Duration in seconds it should take for all joints in the arm to complete one move.

    accel_time
        Duration in seconds it should take for all joints in the arm to accelerate/decelerate
        to/from max speed.

The other three parameters are used to define the time profile of waypoints within a trajectory.
These are used in functions that build trajectories consisting of a series of waypoints such as
``set_ee_cartesian_trajectory``.

.. glossary::

    wp_moving_time
        Duration in seconds that each waypoint in the trajectory should move.

    wp_accel_time
        Duration in seconds that each waypoint in the trajectory should be
        accelerating/decelerating (must be equal to or less than half of :term:`wp_moving_time`).

    wp_period
        Duration in seconds between each waypoint.

Functions
=========

set_ee_pose_matrix
------------------

``set_ee_pose_matrix`` allows the user to specify a desired pose in the form of the homogeneous
transformation matrix, :term:`T_sd`. This method attempts to solve the inverse kinematics of the
arm for the desired pose. If a solution is not found, the method returns ``False``. If the IK
problem is solved successfully, each joint's limits are checked against the IK solver's output. If
the solution is valid, the list of joint positions is returned. Otherwise, ``False`` is returned.

.. warning::

    If an IK solution is found, the method will always return it even if it exceeds joint limits
    and returns ``False``. Make sure to take this behavior into account when writing your own
    scripts.

set_ee_pose_components
----------------------

Some users prefer not to think in terms of transformation or rotation matrices. That's where the
``set_ee_pose_components`` method comes in handy. In this method, you define :term:`T_sd` in terms
of the components it represents - specifically the x, y, z, roll, pitch, and yaw of the :term:`Body
Frame` with respect to the :term:`Space Frame` (where x, y, and z are in meters, and roll, pitch
and yaw are in radians).

.. note::

    If using an arm with less than 6dof, the 'yaw' parameter, even if specified, will always be
    ignored.

set_ee_cartesian_trajectory
---------------------------

When specifying a desired pose using the methods mentioned above, your arm will its end-effector to
the desired pose in a curved path. This makes it difficult to perform movements that are
'orientation-sensitive' (like carrying a small cup of water without spilling). To get around this,
the ``set_ee_cartesian_trajectory`` method is provided. This method defines a trajectory using a
series of waypoints that the end-effector should follow as it travels from its current pose to the
desired pose such that it moves in a straight line. The number of waypoints generated depends on
the duration of the trajectory (a.k.a :term:`moving_time`), along with the period of time between
waypoints (a.k.a :term:`wp_period`). For example, if the whole trajectory should take 2 seconds and
the waypoint period is 0.05 seconds, there will be a total of 2/0.05 = 40 waypoints. Besides for
these method arguments, there is also :term:`wp_moving_time` and :term:`wp_accel_time`.
Respectively, these parameters refer to the duration of time it should take for the arm joints to
go from one waypoint to the next, and the time it should spend accelerating while doing so.
Together, they help to perform smoothing on the trajectory. If the values are too small, the joints
will do a good job following the waypoints but the motion might be very jerky. If the values are
too large, the motion will be very smooth, but the joints will not do a good job following the
waypoints.

This method accepts relative values only. So if the end-effector is located at xyz = [0.2, 0, 0.2],
and then the method is called with 'z=0.3' as the argument, the new pose will be xyz = [0.2, 0,
0.5].

End-effector poses are defined with respect to the virtual frame :term:`T_sy` as defined above. If
you want the end-effector to move 0.3 meters along the X-axis of :term:`T_sy`, I can call the
method with 'x=0.3' as the argument, and it will move to xyz = [0.5828, 0, 0.2] with respect to
:term:`T_sy`. This way, you only have to think in 1 dimension. However, if the end-effector poses
were defined in the :term:`Space Frame`, then relative poses would have to be 2 dimensional. For
example, the pose equivalent to the one above with respect to the :term:`Space Frame` would have to
be defined as xyz = [0.412, 0.412, 0.2].

Tips & Best Practices
=====================

Control Sequence
----------------

The recommended way to control an arm through a series of movements from its Sleep pose is as
follows:

1.  Command the arm to go to its Home pose or any end-effector pose where 'y' is defined as 0 (so
    that the upper-arm link moves out of its cradle).

2.  Command the waist joint until the end-effector is pointing in the desired direction.

3.  Command poses to the end-effector using the ``set_ee_cartesian_trajectory`` method as many
    times as necessary to do a task (pick, place, etc...).

4.  Repeat the above two steps as necessary.

5.  Command the arm to its Home pose.

6.  Command the arm to its Sleep pose.

You can refer to the `bartender`_ script to see the above method put into action.

.. _`bartender`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/python_demos/bartender.py

Miscellaneous Tips
------------------

.. note::

    If using a 6dof arm, it is also possible to use the ``set_ee_cartesian_trajectory`` method to
    move the end-effector along the 'Y-axis' of :term:`T_sy` or to perform 'yaw' motion.

.. note::

    Some functions allow you to provide a **custom_guess** parameter to the IK solver. If you know
    where the arm should be close to in terms of joint positions, providing the solver with them
    will allow it to find the solution faster, more robustly, and avoid joint flips.

.. warning::

    The end-effector should not be pitched past +/- 89 degrees as that can lead to unintended
    movements.

Troubleshooting
===============

The robot 'robot_name' is not discoverable. Did you enter the right robot_model?
--------------------------------------------------------------------------------

.. code-block::

    The robot 'robot_name' is not discoverable. Did you enter the right robot_model?

This error means that essential ROS services can't be found under the 'robot_name' namespace. Check
that the robot name given to your robot and the xsarm_control robot_model/robot_name parameter
match and that the **xs_sdk** node launched successfully.

Video Tutorials
===============

Working with the Interbotix Python API
--------------------------------------

.. youtube:: KoqBEvz4GII
    :width: 40%
    :align: center
