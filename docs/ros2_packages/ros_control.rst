===============================
ROS 2 Controllers Configuration
===============================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/rolling/interbotix_ros_xsarms/interbotix_xsarm_ros_control"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package provides the necessary ROS controllers needed to get MoveIt to control any physical
Interbotix X-Series arm. It essentially takes in Joint Trajectory commands from MoveIt (via the
FollowJointTrajectoryAction interface) and then publishes joint commands at the right time to the
**xs_sdk** node. Currently, only the 'position' values in the Joint Trajectory messages are used
since that provides the smoothest motion. Note that while this package is really only meant to be
used with MoveIt, it could technically be used with any other node that can interface properly with
the `joint_trajectory_controller`_ package.

.. _`joint_trajectory_controller`: http://wiki.ros.org/joint_trajectory_controller

Structure
=========

.. image:: images/xsarm_ros_control_flowchart_ros2.png
    :align: center

As explained in the Overview, this package builds on top of the `interbotix_xsarm_control` package
(which starts the **xs_sdk** node), and is typically used in conjunction with the
`interbotix_xsarm_moveit` package. To get familiar with the nodes in those packages, feel free to
look at their documentation. The other nodes are described below:

-   **controller_manager** - responsible for loading and starting a set of controllers at once, as
    well as automatically stopping and unloading those same controllers
-   **spawner** - responsible for spawning each controller; in this case, three spawners are
    loaded, one for the **arm_controller**, one for the **gripper_controller**, and if using fake
    hardware, one for a **joint_state_broadcaster**
-   **xs_hardware_interface** - receives joint commands from the ROS controllers and publishes them
    to the correct topics (subscribed to by the **xs_sdk** node) at the appropriate times

Usage
=====

This package is not meant to be used by itself but included in a launch file within your custom ROS
package (which should expose a FollowJointTrajectoryAction interface).

To run this package, enter the command below in a terminal (assuming a PincherX-150 is being
launched).

.. code-block:: console

    $ ros2 launch interbotix_xsarm_ros_control xsarm_ros_control.launch.py robot_model:=px150

This is the bare minimum needed to get up and running. Take a look at the table below to see how to
further customize with other launch file arguments.

.. csv-table::
    :file: ../_data/arm_ros_control_ros2.csv
    :header-rows: 1
    :widths: 20, 60, 20, 20
