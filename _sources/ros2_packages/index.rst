==========================
ROS 2 Open Source Packages
==========================

.. note::

    To use any of these packages, you must already have ROS 2 and the X-Series Arm packages
    installed. If you do not have these installed, follow the steps detailed in the :doc:`ROS 2
    Interface Setup<../ros_interface/index>`.

Below is a list of all ROS 2 packages meant to be used with the X-Series robotic arms sold by
Trossen Robotics. Packages were tested on Ubuntu Linux 20.04 using ROS 2 Galactic. Additionally, all
ROS 2 nodes were written using Python or C++. However, any programming language capable of sending
ROS 2 messages can be used to control the robots. The core packages inside this repo are as follows:

-   :doc:`interbotix_xsarm_descriptions<./arm_descriptions>` - contains the meshes and URDFs
    (including accurate inertial models for the links) for all arm platforms
-   :doc:`interbotix_xsarm_control<./arm_control>` - contains the motor configuration files and the
    'root' launch file that is responsible for launching the robot arm
-   :doc:`interbotix_xsarm_sim<./simulation_configuration>` - contains the config files necessary
    to simulate an arm in Gazebo classic
-   :doc:`interbotix_xsarm_ros_control<./ros_control>` - contains configuration files necessary to
    set up ROS 2 controllers between MoveIt and the physical robot arm
-   :doc:`interbotix_xsarm_moveit<./moveit_motion_planning_configuration>` - contains the config
    files necessary to launch an arm using MoveIt

There are also several packages demonstrating possible applications of the core packages. A list of
those packages is below in order of importance:

.. toctree::
    :maxdepth: 1

    arm_descriptions.rst
    arm_control.rst
    simulation_configuration.rst
    ros_control.rst
    moveit_motion_planning_configuration.rst
    perception_pipeline_configuration.rst
    moveit_interface_and_api.rst
    python_demos.rst
    joystick_control.rst
