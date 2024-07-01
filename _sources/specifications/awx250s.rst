=====================
ALOHA WidowX-250 6DOF
=====================

Overview
========

The ALOHA WidowX-250s 6DOF Robot Arm is a variant of the WidowX-250s 6DOF with improvements following those proposed in `ALOHA 2`_.

.. _`ALOHA 2`: https://aloha-2.github.io/

See the :ref:`specifications/wx250s:WidowX-250 6DOF` specifications page for more details.

.. list-table::
    :align: center

    * - .. image:: images/awx250s.png
            :align: center
            :width: 500px

      - .. table::
            :align: center

            +----------------------------------------+--------------------------------------+
            | **ALOHA WidowX-250 6DOF**                                                     |
            +========================================+======================================+
            | Degrees of Freedom                     | 6                                    |
            +----------------------------------------+--------------------------------------+
            | Reach                                  | 650mm                                |
            +----------------------------------------+--------------------------------------+
            | Total Span                             | 1300mm                               |
            +----------------------------------------+--------------------------------------+
            | Repeatability                          | 1mm                                  |
            +----------------------------------------+--------------------------------------+
            | Accuracy                               | 5 - 8mm                              |
            +----------------------------------------+--------------------------------------+
            | Working Payload                        | 250g*                                |
            +----------------------------------------+--------------------------------------+
            | Total Servos                           | 9                                    |
            +----------------------------------------+--------------------------------------+
            | Wrist Rotate                           | Yes                                  |
            +----------------------------------------+--------------------------------------+

.. note::

    \* Working Payload for the WidowX-250 6DOF is inside its maximum reach.
    If intending to use a 250g weight we recommend no more than a 50% extension of the arm.

Default Joint Limits
====================

Default joint limits are the safe range of operation for each joint.
These are set in the firmware, defined as degrees from Zero (servo centered).

.. table::
    :align: center

    +--------------+-------+------+-------------+
    | Joint        | Min   | Max  | Servo ID(s) |
    +==============+=======+======+=============+
    | Waist        | -180  | 180  | 1           |
    +--------------+-------+------+-------------+
    | Shoulder     | -108  | 114  | 2+3         |
    +--------------+-------+------+-------------+
    | Elbow        | -123  | 92   | 4+5         |
    +--------------+-------+------+-------------+
    | Forearm Roll | -180  | 180  | 6           |
    +--------------+-------+------+-------------+
    | Wrist Angle  | -100  | 123  | 7           |
    +--------------+-------+------+-------------+
    | Wrist Rotate | -180  | 180  | 8           |
    +--------------+-------+------+-------------+
    | Gripper      | 30mm  | 74mm | 9           |
    +--------------+-------+------+-------------+

Default Servo Configurations
============================

.. csv-table::
    :file: ../_data/servos_awx250s.csv
    :header-rows: 1
    :widths: 5 10 10 10
    :align: center

.. Kinematic Properties
.. ====================

.. Product of Exponentials [`Learn More`_]
.. ---------------------------------------

.. .. math::

..     M & =
..     \begin{bmatrix}
..     1.0 & 0.0 & 0.0 & 0.458325 \\
..     0.0 & 1.0 & 0.0 & 0.0      \\
..     0.0 & 0.0 & 1.0 & 0.36065  \\
..     0.0 & 0.0 & 0.0 & 1.0
..     \end{bmatrix}

.. .. math::

..     Slist & =
..     \begin{bmatrix}
..     0.0 & 0.0 & 1.0 &  0.0     & 0.0     & 0.0     \\
..     0.0 & 1.0 & 0.0 & -0.11065 & 0.0     & 0.0     \\
..     0.0 & 1.0 & 0.0 & -0.36065 & 0.0     & 0.04975 \\
..     1.0 & 0.0 & 0.0 &  0.0     & 0.36065 & 0.0     \\
..     0.0 & 1.0 & 0.0 & -0.36065 & 0.0     & 0.29975 \\
..     1.0 & 0.0 & 0.0 &  0.0     & 0.36065 & 0.0
..     \end{bmatrix}^T

.. .. _`Learn More`: https://en.wikipedia.org/wiki/Product_of_exponentials_formula

Drawings and CAD Files
======================

.. image:: images/awx250s_drawing.png
    :align: center
    :width: 70%

:download:`ALOHA WidowX-250 6DOF Technical Drawing </_downloads/ALOHA WidowX-250s.pdf>`

.. .. raw:: html

..     <iframe
..         src="https://trossenrobotics.autodesk360.com/shares/public/SH56a43QTfd62c1cd968e4764c968ec64523?mode=embed"
..         width="100%"
..         height="600px"
..         allowfullscreen="true"
..         webkitallowfullscreen="true"
..         mozallowfullscreen="true"
..         frameborder="0">
..     </iframe>

.. - :download:`WidowX-250 6DOF Solid STEP Files </_downloads/solids/10_WXA-250-6DOF.zip>`
.. - `WidowX-250 6DOF Mesh STL Files <https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes/wx250s_meshes>`_
