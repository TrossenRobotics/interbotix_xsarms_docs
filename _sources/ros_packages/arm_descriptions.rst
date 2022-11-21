================
Arm Descriptions
================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package contains the URDFs and meshes for the many X-Series Interbotix Arms. The STL files for
each robot are located in a unique folder inside the `meshes`_ directory. Also in the 'meshes'
directory is the `interbotix_black.png`_ picture. The appearance and texture of the robots come
from this picture. Next, the URDFs for the robot are located in the `urdf`_ directory. They are
written in 'xacro' format so that users have the ability to customize what parts of the URDF get
loaded to the parameter server (see the 'Usage' section below for details). Note that all the other
Interbotix X-Series Arms core ROS packages in the sub-repo reference this package to launch the
robot description.

.. _`meshes`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes
.. _`interbotix_black.png`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes/interbotix_black.png
.. _`urdf`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf

Structure
=========

.. image:: /_images/xsarm_descriptions_flowchart.png
    :align: center

This package contains the `xsarm_description.launch`_ file responsible for loading parts or all of
the robot model. It launches up to four nodes as described below:

-   **joint_state_publisher** - responsible for parsing the 'robot_description' parameter to find
    all non-fixed joints and publish a JointState message with those joints defined.
-   **joint_state_publisher_gui** - does the same thing as the 'joint_state_publisher' node but
    with a GUI that allows a user to easily manipulate the joints.
-   **robot_state_publisher** - uses the URDF specified by the parameter robot_description and the
    joint positions from the joint_states topic to calculate the forward kinematics of the robot
    and publish the results via tf.
-   **rviz** - displays the virtual robot model using the transforms in the 'tf' topic.

.. _`xsarm_description.launch`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/launch/xsarm_description.launch

Usage
=====

To run this package, type the line below in a terminal. Note that the ``robot_model`` argument must
be specified as the name of one of the URDF files located in the `urdf`_ directory (excluding the
'.urdf.xacro' part). For example, to launch the ReactorX-150 arm, type:

.. code-block:: console

    $ roslaunch interbotix_xsarm_descriptions xsarm_description.launch robot_model:=rx150 use_joint_pub_gui:=true

This is the bare minimum needed to get up and running. Take a look at the table below to see how to
further customize with other launch file arguments.

.. csv-table::
    :file: ../_data/arm_descriptions.csv
    :header-rows: 1
    :widths: 20, 60, 20

.. _`xsarm_description.launch`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/launch/xsarm_description.launch

