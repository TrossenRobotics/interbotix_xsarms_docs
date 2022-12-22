===================
Record And Playback
===================

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

Imagine that you would like to have the robot arm perform some motions to achieve a specific task.
One way of doing this would be to create a `JointTrajectory`_ of desired joint positions at
specific times which you could then command the robot. Alternatively (and much less time
consuming), you could manually manipulate the arm to do the specific motions and record them in a
ROS bag file. Then, you could 'play back' the bag file as many times as you like to repeat the
motions on the same robot later. This 'record/playback' feature is made possible by the
**xsarm_puppet_single**.

.. _`JointTrajectory`: http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html

Structure
=========

As shown below, the `interbotix_xsarm_puppet` package builds on top of the
`interbotix_xsarm_control` package. To get familiar with that package, please look at its README.
The nodes specific to this package are described below.

.. image:: images/xsarm_puppet_single_flowchart.png
    :align: center

The above diagram shows the structure for the 'record/playback' feature. The two
nodes involved are:

-   **xsarm_puppet_single** - responsible for reading the arm's current joint states and publishing
    them as position commands to the ``/<robot_name>/commands/joint_group`` and
    ``/<robot_name>/commands/joint_single`` topics. Conveniently, as the arm is torqued off so that
    the user can manipulate it, the joints will not act upon these commands
-   **record** - responsible for recording the two topics mentioned above in a bag file so that
    they can be played back later
-   **play** - responsible for playing back the bag file mentioned above with the arm in a torqued
    on state; the playback is delayed 3 seconds to give time for the **xs_sdk** node to load

Usage
=====

To record joint commands while manipulating a single robot (let's say the PincherX-150), type the
following in a terminal:

.. code-block:: console

    $ roslaunch interbotix_xsarm_puppet xsarm_puppet_single.launch robot_model:=px150 record:=true

Once the nodes finish launching, manually manipulate the arm and gripper through your desired
motions. When done, return the robot to its initial starting position and :kbd:`Ctrl` + :kbd:`C`
the nodes so that rosbag stops recording. By default, the bag file will be saved in the `bag`_
directory. To playback the motion, type the following in the terminal:

.. _`bag`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/bag

.. code-block:: console

    $ roslaunch interbotix_xsarm_puppet xsarm_puppet_single.launch robot_model:=px150 playback:=true

If you specified a custom bag file name, make sure to include the name in the above command as
well. When the bag file finishes playing, you can restart it by navigating to the directory where
the bag file is located and in a terminal, type:

.. code-block:: console

    $ rosbag play <bag file name>

The robot should now repeat the motions. When done, :kbd:`Ctrl` + :kbd:`C` to stop all nodes. To
see other command line arguments for the 'xsarm_puppet_single.launch' file, refer to the table
below.

.. csv-table::
    :file: ../_data/record_and_playback.csv
    :header-rows: 1
    :widths: 20, 60, 20

Notes
=====

There is a bag file in the `bag`_ directory. It was created using the 'record/playback' feature
with the WidowX 250 arm. See if you can figure out what it does! As a hint, it involves rubber
ducks. To run it in simulation, type...

.. code-block:: console

    $ roslaunch interbotix_xsarm_puppet xsarm_puppet_single.launch robot_model:=wx250 use_sim:=true playback:=true bag_name:=duck_dunk

Video Tutorials
===============

Record & Playback X-Series Arm Demo
-----------------------------------

.. youtube:: uHEyd6nuYzI
    :width: 70%
    :align: center
