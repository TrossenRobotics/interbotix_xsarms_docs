===================
Record And Playback
===================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/rolling/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

Imagine that you would like to have the robot arm perform some motions to achieve a specific task.
One way of doing this would be to create a `JointTrajectory`_ of desired joint positions at specific times which you could then command the robot.
Alternatively (and much less time consuming), you could manually manipulate the arm to do the specific motions and record them in a ROS bag file.
Then, you could 'play back' the bag file as many times as you like to repeat the motions on the same robot later.
This 'record/playback' feature is made possible by the **xsarm_puppet_single**.

.. _`JointTrajectory`: http://docs.ros.org/latest/api/trajectory_msgs/html/msg/JointTrajectory.html

Structure
=========

As shown below, the `interbotix_xsarm_puppet` package builds on top of the `interbotix_xsarm_control` package.
To get familiar with that package, please look at its README.
The nodes specific to this package are described below.

.. image:: images/xsarm_puppet_single_flowchart.png
    :align: center

The above diagram shows the structure for the 'record/playback' feature.
The two nodes involved are:

-   **xsarm_puppet_single** - responsible for reading the arm's current joint states and publishing them as position commands to the ``/<robot_name>/commands/joint_group`` and ``/<robot_name>/commands/joint_single`` topics.
    Conveniently, as the arm is torqued off so that the user can manipulate it, the joints will not act upon these commands
-   **record** - responsible for recording the two topics mentioned above in a bag file so that they can be played back later
-   **play** - responsible for playing back the bag file mentioned above with the arm in a torqued on state; the playback is delayed 3 seconds to give time for the **xs_sdk** node to load

Usage
=====

To record joint commands while manipulating a single robot (let's say the PincherX-150), type the following in a terminal:

.. code-block:: console

    $ ros2 launch interbotix_xsarm_puppet xsarm_puppet_single.launch.py robot_model:=px150 record_or_playback:=record

Once the nodes finish launching, manually manipulate the arm and gripper through your desired motions.
When done, return the robot to its initial starting position and :kbd:`Ctrl` + :kbd:`C` the nodes so that rosbag stops recording.
By default, the bag file will be saved in the `bag`_ directory.
To playback the motion, type the following in the terminal:

.. _`bag`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/rolling/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet/bag

.. code-block:: console

    $ ros2 launch interbotix_xsarm_puppet xsarm_puppet_single.launch.py robot_model:=px150 record_or_playback:=playback

If you specified a custom bag file name, make sure to include the name in the above command as well.
When the bag file finishes playing, you can restart it by navigating to the directory where the bag file is located and in a terminal, type:

.. code-block:: console

    $ ros2 bag play </path/to/bag>

The robot should now repeat the motions.
When done, :kbd:`Ctrl` + :kbd:`C` to stop all nodes.
To see other command line arguments for the 'xsarm_puppet_single.launch.py' file, refer to the table below.

.. csv-table::
    :file: ../_data/record_and_playback_ros2.csv
    :header-rows: 1
    :widths: 20, 60, 20, 20
