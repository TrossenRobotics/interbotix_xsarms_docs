====================
Gazebo Configuration
====================

.. raw:: html

    <a
    href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_gazebo"
        class="docs-view-on-github-button" target="_blank"> <img
        src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package contains the necessary config files to get any of the many Interbotix X-Series arms
simulated in Gazebo. Specifically, it contains the `interbotix_texture.gazebo`_ file which allows
the black texture of the robotic arms to display properly (following the method explained here). It
also contains YAML files with tuned PID gains for the arm and gripper joints so that ros_control
can control the arms effectively. This package has one of two applications. It can either be used
in conjunction with MoveIt via the FollowJointTrajectory interface or it can be used by itself via
the JointPositionController interface.

.. _`interbotix_texture.gazebo`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_gazebo/config/interbotix_texture.gazebo

Structure
=========

.. image:: /_images/xsarm_gazebo_flowchart.png
    :align: center

As shown above, the `interbotix_xsarm_gazebo` package builds on top of the
`interbotix_xsarm_descriptions` and `gazebo_ros` packages. To get familiar with the nodes in the
`interbotix_xsarm_descriptions` package, please look at its README. The other nodes are described
below:

-   **gzserver** - responsible for running the physics update-loop and sensor data generation
-   **gzclient** - provides a nice GUI to visualize the robot simulation
-   **controller_manager** - responsible for loading and starting a set of controllers at once, as
    well as automatically stopping and unloading those same controllers
-   **spawn_model** - adds the robot model as defined in the 'robot_description' parameter into the
    Gazebo world

Usage
=====

To run this package, type the line below in a terminal (assuming the WidowX-250 is being launched
with trajectory controllers). Note that if you're using a 4 or 6 dof arm, you should set the
``dof`` launch file argument appropriately.

.. code-block:: console

    $ roslaunch interbotix_xsarm_gazebo xsarm_gazebo.launch robot_model:=wx250 use_trajectory_controllers:=true

Since by default, Gazebo is started in a 'paused' state (this is done to give time for the
controllers to kick in), unpause the physics once it is fully loaded by typing:

.. code-block:: console

    $ rosservice call /gazebo/unpause_physics

This is the bare minimum needed to get up and running. Take a look at the table below to see how to
further customize with other launch file arguments.

.. csv-table::
    :file: ../_data/gazebo_simulation_configuration.csv
    :header-rows: 1
    :widths: 20, 60, 20

.. _`xsarm_gazebo.launch`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_gazebo/launch/xsarm_gazebo.launch

Video Tutorial
==============

Working with the Interbotix Arm in Gazebo
-----------------------------------------

.. youtube:: k3zkgN7TYTE
    :width: 70%
    :align: center
