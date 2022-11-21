================
Arm Puppeteering
================

.. raw:: html

  <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet"
    class="docs-view-on-github-button"
    target="_blank">
    <img src="../_static/GitHub-Mark-Light-32px.png"
        class="docs-view-on-github-button-gh-logo">
      View Package on GitHub
  </a>

Overview
========

Imagine that you have two (or more) Interbotix arms with the same number of joints. What the
**xsarm_puppet** allows you to do is to manually manipulate one of the arms and watch as the motion
is repeated in real time on the second robot. A potential application of this could be a
warehousing environment. Instead of a worker straining his back to lift a heavy package, he could
manually manipulate a small version of a robotic arm to maneuver an industrial-sized arm to pick it
up.

Structure
=========

.. image:: /_images/xsarm_puppet_flowchart.png
    :align: center

The above diagram shows the structure for the 'puppet control' feature. The nodes involved are:

-   **xsarm_puppet** - responsible for reading the current joint states from one arm and then
    publishing them as position commands to a second arm; it accepts two robot names as parameters
    - ``robot_name_master`` and ``robot_name_puppet``
-   **static_transform_publisher** - two of these nodes are launched to specify the position of
    both arms relative to the RViz 'world' frame
-   **rviz** - only launch one instance of RViz with two RobotModels for the two robot arms

Usage
=====

To run this feature, plug two robots with the same number of joints (ex. the PincherX-150 and
WidowX-250) into two USB ports and in a terminal, type:

.. code-block:: console

    $ roslaunch interbotix_xsarm_puppet xsarm_puppet.launch robot_model_master:=px150 robot_model_puppet:=wx250

It might be good idea to verify which robot is given ttyUSB0 as its port name so that you know
which robot should be manually manipulated. Do this by only plugging one robot into your computer
and looking at the name in the /dev directory before plugging in the second one. Next, manually
manipulate the first arm and observe how the second arm closely mimics the motion! To see other
command line arguments for the 'xsarm_puppet.launch' file, refer to the table below.

.. csv-table::
    :file: ../_data/arm_puppeteering.csv
    :header-rows: 1
    :widths: 20, 60, 20

.. _`xsarm_puppet.launch`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/launch/xsarm_puppet.launch

Video Tutorials
===============

Puppet Control X-Series Arm Demo
--------------------------------

.. youtube:: uRQX2yupPEI
    :width: 70%
    :align: center

|

Working With Multiple Arms
--------------------------

.. youtube:: DnjbNXxBE_8
    :width: 70%
    :align: center
