===========================
ROS Standard Software Setup
===========================

.. contents::
  :local:

Compatible Models
=================

The ROS 1 Interface packages can be used with any of the Interbotix arm kits listed below. Next to
each name is the codename used to describe it in software (specifically for the ``robot_model``
argument in launch files). There are up to four parts in a name. In general, the first two letters
represent model type (ex. ``wx`` for 'WidowX'). The number afterwards corresponds to the length of
both the forearm and upper-arm links in millimeters (ex. ``200``). Finally, the ``s`` after some
numbers signifies that arm has six degrees of freedom. The ``mobile`` before some names means that
arm is designed to be mounted on a mobile base.

.. list-table:: **Models**
    :align: center
    :header-rows: 1
    :widths: 30 40 20

    * - Model Name and Store Link
      - Robot Documentation
      - Codename
    * - `PincherX-100 Robot Arm`_
      - :doc:`PincherX-100 Robot Arm Documentation </specifications/px100>`
      - ``px100``
    * - `PincherX-150 Robot Arm`_
      - :doc:`PincherX-150 Robot Arm Documentation </specifications/px150>`
      - ``px150``
    * - `ReactorX-150 Robot Arm`_
      - :doc:`ReactorX-150 Robot Arm Documentation </specifications/rx150>`
      - ``rx150``
    * - `ReactorX-200 Robot Arm`_
      - :doc:`ReactorX-200 Robot Arm Documentation </specifications/rx200>`
      - ``rx200``
    * - `WidowX-200 Robot Arm`_
      - :doc:`WidowX-200 Robot Arm Documentation </specifications/wx200>`
      - ``wx200``
    * - `WidowX-250 Robot Arm`_
      - :doc:`WidowX-250 Robot Arm Documentation </specifications/wx250>`
      - ``wx250``
    * - `WidowX-250 Robot Arm 6DOF`_
      - :doc:`WidowX-250 Robot Arm 6DOF Documentation </specifications/wx250s>`
      - ``wx250s``
    * - `ViperX-250 Robot Arm`_
      - :doc:`ViperX-250 Robot Arm Documentation </specifications/vx250>`
      - ``vx250``
    * - `ViperX-300 Robot Arm`_
      - :doc:`ViperX-300 Robot Arm Documentation </specifications/vx300>`
      - ``vx300``
    * - `ViperX-300 Robot Arm 6DOF`_
      - :doc:`ViperX-300 Robot Arm 6DOF Documentation </specifications/vx300s>`
      - ``vx300s``

.. _PincherX-100 Robot Arm: https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx
.. _PincherX-150 Robot Arm: https://www.trossenrobotics.com/pincherx-150-robot-arm.aspx
.. _ReactorX-150 Robot Arm: https://www.trossenrobotics.com/reactorx-150-robot-arm.aspx
.. _ReactorX-200 Robot Arm: https://www.trossenrobotics.com/reactorx-200-robot-arm.aspx
.. _WidowX-200 Robot Arm: https://www.trossenrobotics.com/widowx-200-robot-arm.aspx
.. _WidowX-250 Robot Arm: https://www.trossenrobotics.com/widowx-250-robot-arm.aspx
.. _WidowX-250 Robot Arm 6DOF: https://www.trossenrobotics.com/widowx-250-robot-arm-6dof.aspx
.. _ViperX-250 Robot Arm: https://www.trossenrobotics.com/viperx-250-robot-arm.aspx
.. _ViperX-300 Robot Arm: https://www.trossenrobotics.com/viperx-300-robot-arm.aspx
.. _ViperX-300 Robot Arm 6DOF: https://www.trossenrobotics.com/viperx-300-robot-arm-6dof.aspx

Requirements
============

Below is a list of the hardware you will need to get started:

- One of the X-Series Robot Arm Kits mentioned above
- Computer running Ubuntu Linux 18.04, 20.04, or 22.04

.. important::

    Virtual Linux machines have not been tested are **not supported**.

Software Installation
=====================

To get all the code setup, refer to the computer platform types below and run the appropriate
installation script. Afterwards, continue with the `Installation Checks`_ sub-section.

AMD64 Architecture
------------------

If your computer uses an Intel or AMD based processor (which is the case for NUCs, most laptops and
desktop computers), follow the commands below to download and run the installation script. Specify
the version of ROS that you want to install using the ``-d`` flag followed by the distribution's
codename. Alternatively, you can run it without the ``-d`` flag and the script will install
packages for the ROS 1 distribution supported by the version of Ubuntu, or the latest stable
release of ROS 2 if using Ubuntu version 22.04 or later. See the `list of currently supported
distributions`_. Note that the script will also install the full desktop version of ROS if it's not
yet on your system, ask you if you want to install the Interbotix Perception packages and ask you
if you want to install the MATLAB-ROS API. The commands below demonstrate the process of running
the installation script for ROS 1 Noetic.

.. _`list of currently supported distributions`: https://github.com/Interbotix/interbotix_ros_manipulators/security/policy#supported-versions

    .. code-block:: console

        $ sudo apt install curl
        $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
        $ chmod +x xsarm_amd64_install.sh
        $ ./xsarm_amd64_install.sh -d noetic

    .. note::

        The install script provides more in-depth control of some installation options. Append the
        ``-h`` flag to see the help document like below:

        .. code-block:: console

            $ ./xsarm_amd64_install.sh -h
            USAGE: ./xsarm_amd64_install.sh [-h][-d DISTRO][-p PATH][-n]

            ...

Raspberry Pi 4B (ARM64 Architecture)
------------------------------------

If you purchased a Raspberry Pi 4B Kit with an arm from our website, there is no need to install
anything as the Pi should already come preloaded with all the necessary software. If you purchased
your own Raspberry Pi 4B from a third party, feel free to follow the `Raspberry Pi 4B Setup
instructions`_ to get it properly setup before following the commands below. If you only purchased
the stand-alone Raspberry Pi 4B Kit from our store (which comes pre-configured with Ubuntu and
ROS), and would like to use it with an arm, then follow the commands below to download and run the
installation script. Note that the script will install the full desktop version of ROS if it's not
yet on your system, ask you for your desired robot model (ex. wx200), and prompt you about whether
or not you'd like the Joystick ROS package to start at boot. The commands below
demonstrate the process of running the installation script for ROS 1 Noetic.


.. _Raspberry Pi 4B Setup instructions: ./raspberry_pi_setup.html

    .. code-block:: console

        $ sudo apt install curl
        $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh' > xsarm_rpi4_install.sh
        $ chmod +x xsarm_rpi4_install.sh
        $ ./xsarm_rpi4_install.sh -d noetic

If you **do** want to have the Joystick ROS package start at boot, you will first have to pair your
PS4 controller with the Pi. Refer to the :ref:`RPi PS4 Controller Setup Guide
<raspberry-pi-4b-sony-ps4-controller-label>` for details.

    .. note::

        The install script provides more in-depth control of some installation options. Append the
        ``-h`` flag to see the help document like below:

        .. code-block:: console

            $ ./xsarm_rpi4_install.sh -h
            USAGE: ./xsarm_rpi4_install.sh [-h][-d DISTRO][-j ROBOT_MODEL][-p PATH][-n]

            ...

Remote Install
--------------

For some robotic projects, you may want to run your robot in a 'headless' state on some computer
(like a NUC or Raspberry Pi), and monitor the robot's state (in RViz for example) on your personal
(a.k.a remote) computer over a local network. For this to work, run the installation script below
on your personal computer running Linux Ubuntu 18.04, 20.04, or 22.04. Note that ROS and RViz must
already be installed! As an FYI, the script will prompt you to insert the hostname of the robot
(NOT the remote) computer. As an example, if you wanted to monitor the state of-a robot arm
purchased with a Raspberry Pi 4B Kit, you would set the hostname to ``pibot``. To find out the
hostname of the robot computer, just open a terminal and type ``hostname``.

    .. code-block:: console

        $ sudo apt install curl
        $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/xsarm_remote_install.sh' > xsarm_remote_install.sh
        $ chmod +x xsarm_remote_install.sh
        $ ./xsarm_remote_install.sh

Be aware that the installation script will export the `ROS_MASTER_URI` environment variable in your
personal computer's ~/.bashrc file to ``http://<hostname>.local:11311``. Make sure to comment out
this line when done monitoring or your personal computer will complain about not being able to find
its ROS Master.

.. _installation-checks-label:

Installation Checks
===================

After running the installation script on the robot computer, we can verify that the script ran successfully.

udev Rules
----------

Check that the udev rules were configured correctly and that they are triggered by the U2D2. This
can be done by checking that the port name shows up as ``ttyDXL`` when the U2D2 is plugged into a
USB port. The command and the expected output are below:

    .. code-block:: console

        $ ls /dev | grep ttyDXL
        ttyDXL

Interbotix ROS Packages
-----------------------

Check that the Interbotix ROS packages were installed correctly. The command and example output are
below:

    .. code-block:: console

        $ source /opt/ros/$ROS_DISTRO/setup.bash
        $ source ~/interbotix_ws/devel/setup.bash
        $ rospack list | grep interbotix
        ...
        interbotix_common_modules /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_common_modules
        interbotix_landmark_modules /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules
        interbotix_moveit_interface /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface
        interbotix_perception_modules /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules
        interbotix_tf_tools /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_tf_tools
        interbotix_xs_modules /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules
        interbotix_xs_msgs /home/$USER/interbotix_ws/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs
        interbotix_xs_ros_control /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_ros_control
        interbotix_xs_rviz /home/$USER/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_rviz
        interbotix_xs_sdk /home/$USER/interbotix_ws/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
        interbotix_xsarm_control /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control
        interbotix_xsarm_descriptions /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions
        interbotix_xsarm_diagnostic_tool /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool
        interbotix_xsarm_dual /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual
        interbotix_xsarm_dual_joy /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_joy
        interbotix_xsarm_gazebo /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_gazebo
        interbotix_xsarm_joy /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_joy
        interbotix_xsarm_moveit /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_moveit
        interbotix_xsarm_moveit_dual /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_ros_xsarms_dual_moveit
        interbotix_xsarm_moveit_interface /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_moveit_interface
        interbotix_xsarm_perception /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception
        interbotix_xsarm_pid /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_pid
        interbotix_xsarm_puppet /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet
        interbotix_xsarm_ros_control /home/$USER/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_ros_control
        ...

Specific packages you should confirm have been built are `interbotix_xs_sdk`, `interbotix_xs_msgs`,
and `interbotix_xs_modules`. These serve as the fundamental core of the ROS 1 Interface and are
required to use it. If these are missing, check the installation script's output for errors.

Next Steps
==========

If the ROS Interface installed properly, you can continue on to the :doc:`ROS Interface Quickstart
Guide <./quickstart>`.

.. _troubleshooting-label:

Troubleshooting
===============

Refer to the :doc:`X-Series Troubleshooting guide </troubleshooting>` to try to solve your
problem. If you still need help, feel free to `open an Issue`_ on the ros_manipulators repo. We
strongly recommend the latter option though so that other people who may be facing the same
difficulty can benefit. This repository is actively maintained and any open Issues will be
addressed as soon as possible.

.. _open an Issue: https://github.com/Interbotix/interbotix_ros_manipulators/issues

.. Keeping your Installation Updated
.. =================================

.. Because the Interbotix X-Series Arm packages are not (currently) handled through a package manager
.. and instead through a source installation, updates must be installed manually by the user. To do
.. this, follow the steps detailed below:

.. 1.  Check if an update exists in any of the three repositories using the ``git fetch`` command.
.. 2.  

Video Tutorials
===============

X-Series Arm ROS Installation Guide
-----------------------------------

.. youtube:: kZx2tNVfQAQ
    :align: center
