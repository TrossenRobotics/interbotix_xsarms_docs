========================
Simulation Configuration
========================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/galactic/interbotix_ros_xsarms/interbotix_xsarm_sim"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package contains the necessary config files to simulate any of the many Interbotix X-Series
arms. For now, this package is only able to simulate the arms in Gazebo Classic. Below is a
description of each supported simulation environment's configuration.

-   Gazebo Classic - Contains parameter files that configure trajectory_controllers so that
    gazebo_ros2_control can control the arms effectively. This package has one of two applications.
    It can either be used in conjunction with MoveIt via the FollowJointTrajectory interface or it
    can be used by itself via the JointTrajectoryController interface.

Structure
=========

.. image:: images/xsarm_gz_classic_flowchart_ros2.png
    :align: center

As shown above, the `interbotix_xsarm_sim` package builds on top of the
`interbotix_xsarm_descriptions` and other simulator's ROS 2 compatibility packages. To get familiar
with the nodes in the `interbotix_xsarm_descriptions` package, please look at its README. The other
nodes are described below:

- Gazebo Classic:
    -   **gzserver** - responsible for running the physics update-loop and sensor data generation
    -   **gzclient** - provides a nice GUI to visualize the robot simulation
    -   **controller_manager** - responsible for loading and starting a set of controllers at once,
        as well as automatically stopping and unloading those same controllers
    -   **spawn_model** - adds the robot model as defined in the 'robot_description' parameter into
        the Gazebo world

Usage
=====

Gazebo Classic
--------------

To run this package, enter the command line below in a terminal (assuming the WidowX-250 is being
launched).

.. code-block:: console

    $ ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=wx250

This is the bare minimum needed to get up and running. Take a look at the table below to see how to
further customize with other launch file arguments.

.. csv-table::
    :file: ../_data/gz_classic_simulation_configuration_ros2.csv
    :header-rows: 1
    :widths: 20, 60, 20, 20
