===================
Arm Diagnostic Tool
===================

.. raw:: html

  <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool"
    class="docs-view-on-github-button"
    target="_blank">
    <img src="../_static/GitHub-Mark-Light-32px.png"
        class="docs-view-on-github-button-gh-logo">
      View Package on GitHub
  </a>

Overview
========

This package is meant to be used as a way to analyze joint data over time. For example, if the user
would like the arm to perform a certain task but is not sure if it will 'strain' a specific arm
joint too much, then this tool would be an easy way to record, save (to a rosbag and CSV file), and
view data live while the joint in question rotates.

Structure
==========

.. image:: images/xsarm_diagnostic_tool_flowchart.png
    :align: center
    :width: 70%

As shown above, the `interbotix_xsarm_diagnostic_tool` package builds on top of the
`interbotix_xsarm_control` package. To get pointers about the nodes in the that package, please
look at its README. The other nodes are described below:

-   **xsarm_diagnostic_tool** - responsible for commanding joint positions to the user-specified
    joint following a sinusoidal trajectory; the trajectory is symmetric around '0' radians where
    the upper bound is the minimum of the absolute value of the upper and lower joint limits. It
    also publishes temperatures at all the arm joints (excluding gripper) to the
    ``/<robot_name>/temperatures/joint_group`` topic.
-   **record** - responsible for recording the ``/<robot_name>/commands/joint_single``,
    ``/<robot_name>/joint_states``, and ``/<robot_name>/temperatures/joint_group`` topics and
    saving it to a user-specified bagfile
-   **rqt_plot** - three instances of this node are launched to plot data for a user-specified
    joint; one plots the joint commands along with the observed joint positions [rad] and
    velocities [rad/s] vs. time [s]. Another plots effort [mA] vs time [s], while the last one
    plots joint temperature [C] vs. time [s].

Usage
=====

To use this package, first manually manipulate the arm (let's say the PincherX-150) until the
desired starting pose is reached. Then type the command below (if testing the waist joint):

.. code-block:: console

    $ roslaunch interbotix_xsarm_diagnostic_tool xsarm_diagnostic_tool.launch robot_model:=px150 cmd_joint:=waist observe_joint:=waist bag_name:=px150_diagnostics

Keep holding the arm until the motors torque on. At this point, let go of the arm. Since the
cmd_joint argument is waist, this means that the 'waist' joint will begin to rotate following a
sinusoidal trajectory. Additionally, since the observe_joint argument is waist, the plots will
start showing data for the 'waist' joint only. The plot axes might have to be adjusted to visualize
the data properly. The joint will then continue to rotate for the user-specified time (see the
launch file argument table below) at which point the arm will go to its 'sleep' pose and the
`xsarm_diagnostic_tool` node will terminate. Finally, the user should :kbd:`Ctrl` + :kbd:`C` the
launch file so that rosbag will stop recording data.

To convert the `rosbag` data to a csv file, navigate to the `scripts`_ directory. If the
'bag2csv.py' program is not yet executable, make it so by typing:

.. _scripts: https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool/scripts

.. code-block:: console

    $ chmod a+x bag2csv.py

Next, type:

.. code-block:: console

    $ python bag2csv.py px150 waist px150_diagnostics.bag px150_diagnostics.csv

The command is pretty self explanatory - the arguments are the robot name, the name of the joint to
be observed, the bagfile name (the program expects it to be located in the 'bag' directory), and
the desired CSV file name. To better understand how this program works, take a look at
`bag2csv.py`_. Then, take a look at the table below to understand the launch file arguments.

.. _bag2csv.py: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool/scripts/bag2csv.py

.. csv-table::
    :file: ../_data/arm_diagnostic_tool.csv
    :header-rows: 1
    :widths: 20, 60, 20

.. _xsarm_diagnostic_tool.launch: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool/launch/xsarm_diagnostic_tool.launch

Notes
=====

If choosing to rotate the 'waist' joint, make sure during the initial step of manually positioning
the arm that it is NOT rotated past the joint limit (+/-180 degrees). Otherwise, the cable
connecting the 'waist' motor to the 'shoulder' motor(s) can be ripped out.

Also, take a note of the `data`_ directory in the package. After plotting and saving the data to a
CSV, it might be a good idea to create a new directory to keep all the files together. In this
case, the CSV file and picture in that directory show diagnostics for the waist joint of the WidowX
200 arm as it rotates for 100 seconds.

.. _data: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool/data

Video Tutorial
==============

Diagnostic Tool X-Series Arm Demo
---------------------------------

.. youtube:: 0N85vMS8LMU
    :width: 70%
    :align: center
