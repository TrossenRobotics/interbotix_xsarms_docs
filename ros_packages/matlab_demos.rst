============
MATLAB Demos
============

.. raw:: html

    <a
    href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/matlab_demos"
        class="docs-view-on-github-button" target="_blank"> <img
        src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

These demos showcases various ways of using the `Interbotix MATLAB Arm Module`_ (click the link to
see the fully documented code; also check out the `interbotix_xs_modules`_ package to get a better
understanding on how this and other modules can be used). Simply put, this API was created so that
users with little to no ROS experience would still have the ability to control any Interbotix Arm
supported by the `interbotix_xs_sdk`. Specifically, the API also allows a user to make an arm go to
desired end-effector poses or follow Cartesian trajectories. This last feature was made possible by
the `Modern Robotics: Mechanics, Planning, and Control Code Library`_  created at Northwestern
University.

For the API to work, the arm joints must be set to 'position' control and the gripper set to 'PWM'
control (conveniently, these are the default configs in the `interbotix_xsarm_control` package).
Furthermore, the API assumes that all the arm-joint motors' `Drive Mode`_ registers are set to
`Time-Based-Profile`_ (this is also the default configuration). In a nutshell, this setting makes
it very easy for you as the user to customize the duration and smoothness of an arm's motion from
one pose to the next.

.. _`Interbotix MATLAB Arm Module`: https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/InterbotixArmXSInterface.m
.. _`interbotix_xs_modules`: https://github.com/Interbotix/interbotix_ros_toolboxes/tree/main/interbotix_xs_toolbox/interbotix_xs_modules
.. _`Modern Robotics: Mechanics, Planning, and Control Code Library`: https://github.com/NxRLab/ModernRobotics
.. _`Drive Mode`: http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode
.. _`Time-Based-Profile`: http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#profile-velocity112

Structure
=========

.. image:: /_images/xsarm_matlab_demos_flowchart.png
    :align: center

The MATLAB Arm module (located in the `interbotix_xs_modules` ROS package) builds on top of the
`interbotix_xs_sdk` package. To get familiar with the other nodes in the graph above, look at the
various packages documentation.

-   **robot_manipulation** - a ROS node (operating 'behind the scenes') that takes in commands
    entered via the MATLAB API and publishes data to various ROS topics as necessary. It is not a
    classic ROS node in the sense that it can't be launched from a ROS launch file or run from the
    terminal using a ``rosrun`` command. Rather, the MATLAB API module contains a Class that when
    instantiated, gives the node life. At the completion of a program, the object gets destroyed,
    killing the node.

Usage
=====

To get started with the MATLAB demos, you must first build the Interbotix messages and setup your
MATLAB workspace. Follow the `MATLAB Setup Guide`_ for this process.

.. _`MATLAB Setup Guide`: ../matlab_ros_interface/index.html#setup

Once your MATLAB workspace is properly configured, open up a terminal and type (assuming a
WidowX-250 is being launched)...

.. code-block:: console

    $ roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250

Then, in another terminal, navigate to the matlab_demos directory in the MATLAB console and type...

.. code-block:: matlab

    bartender

You should observe the robot pick up a virtual bottle (from behind a virtual bar), rotate so that
the end-effector is facing the opposite direction, pour a virtual drink (on the virtual bar), then
place the 'bottle' down, and go to its Sleep pose.

The other scripts work in a similar fashion, but you must make sure to change the robot name in the
file to the arm you have. You might also have to adjust the commanded poses/trajectories if working
with smaller arm models (like the PincherX 100) as some of them might by physically unattainable.
To make things easier, each script also outlines the commands necessary to get the robot moving!

Notes
=====

MATLAB does not delete timers when a workspace is cleared. A buildup of timers will accumulate if
not handled properly, and your machine will slow down. To prevent this from occurring, you can run
the ``stop_timers()`` method of the InterbotixManipulatorXS class at the end of each of your
scripts. Examples of this process are in each of the MATLAB demo scripts. You can also run the
commands ``stop(timerfindall)`` and ``delete(timerfindall)``, though this may have unintended
consequences if you have timers in objects other than the arm.

When using the MATLAB modules, you may see the error ``incoming connection failed: unable to
receive data from sender, check sender's logs for details``. This is just a result from how MATLAB
constructs its rosservice objects and can be safely ignored.

.. note::

    If you want to test out your code first on a simulated arm, make sure to set
    the ``use_sim`` arg to true like this:

        .. code-block:: console

            $ roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250 use_sim:=true

Video Tutorial
==============

Working with the Interbotix MATLAB API
--------------------------------------

.. youtube:: gs4x_G837tI
    :width: 70%
    :align: center
