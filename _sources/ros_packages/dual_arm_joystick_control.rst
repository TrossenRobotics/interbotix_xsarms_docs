=========================
Dual Arm Joystick Control
=========================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_joy"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package allows the user to control two (or any number of) X-Series arms using a single
joystick controller. This package relies on two lower-level packages, `interbotix_xsarm_dual` to
provide simultaneous control of multiple arms, and `interbotix_xsarm_joy` to provide joystick
control.

Structure
=========

.. image:: /_images/xsarm_dual_joy_flowchart.png
    :align: center

As shown above, the `interbotix_xsarm_dual_joy` package builds on top of the
`interbotix_xsarm_control`, `interbotix_xsarm_joy` and `interbotix_xsarm_dual` packages. See the
other packages for descriptions of their nodes.

Usage
=====

To get started, see the Usage section for both of the lower-level packages mentioned previously.

- `Dual Usage`_
- `Joy Usage`_

.. _`Dual Usage`: https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_dual#usage
.. _`Joy Usage`: https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_joy#usage

If you have a Bluetooth controller and it is not yet connected to you computer, see `X-Series Arm
Bluetooth Setup`_.

.. _`X-Series Arm Bluetooth Setup`: https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_joy#bluetooth-setup

Once you have familiarized yourself with the capabilities of the lower-level packages, you will be
ready to use this one. We will assume that you have done the following:

-   Properly paired your controller with your computer.
-   Configured udev rules.
-   Checked the ``/dev/ttyUSB*`` device name for each arm.
-   Changed the settings in the config files config/, especially ``port``.

Unplug/replug the U2D2s to refresh the ports and run the launch file below with the correct robot
model in place of ``<MODEL_*>``.

.. code-block:: console

    $ roslaunch interbotix_xsarm_dual_joy xsarm_dual_joy.launch robot_model_1:=<MODEL_1> robot_model_2:=<MODEL_2>

You can now use your Bluetooth controller to move around the arms.

To further customize the launch file at run-time (like with a different robot model), look at the
table below:

.. csv-table::
    :file: ../_data/dual_arm_joystick_control.csv
    :header-rows: 1
    :widths: 20, 60, 20

.. _`xsarm_dual_joy.launch`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_joy/launch/xsarm_dual_joy.launch

Video Tutorial
==============

Using a Joystick to Control X-Series Arms
-----------------------------------------

.. youtube:: AyKjcZvu8lo
    :width: 70%
    :align: center
    :url_parameters: ?start=408
