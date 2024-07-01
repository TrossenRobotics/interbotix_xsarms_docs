=====================
ALOHA ViperX-300 6DOF
=====================

Overview
========

The ALOHA ViperX-300 6DOF Robot Arm is a variant of the ViperX-300 6DOF with improvements following those proposed in `ALOHA 2`_.

.. _`ALOHA 2`: https://aloha-2.github.io/

See the :ref:`specifications/vx300s:ViperX-300 6DOF` specifications page for more details.

.. list-table::
    :align: center

    * - .. image:: images/avx300s.png
            :align: center
            :width: 500px

      - .. table::
            :align: center

            +----------------------------------------+--------------------------------------+
            | **ALOHA ViperX-300 6DOF**                                                     |
            +========================================+======================================+
            | Degrees of Freedom                     | 6                                    |
            +----------------------------------------+--------------------------------------+
            | Reach                                  | 750mm                                |
            +----------------------------------------+--------------------------------------+
            | Total Span                             | 1500mm                               |
            +----------------------------------------+--------------------------------------+
            | Repeatability                          | 1mm                                  |
            +----------------------------------------+--------------------------------------+
            | Accuracy                               | 5 - 8mm                              |
            +----------------------------------------+--------------------------------------+
            | Working Payload                        | 750g*                                |
            +----------------------------------------+--------------------------------------+
            | Total Servos                           | 9                                    |
            +----------------------------------------+--------------------------------------+
            | Wrist Rotate                           | Yes                                  |
            +----------------------------------------+--------------------------------------+

.. note::

    \* Working Payload for the ALOHA ViperX-300 6DOF is inside its maximum reach.
    If intending to use a 750g weight we recommend no more than a 50% extension of the arm.

Default Joint Limits
====================

Default joint limits are the safe range of operation for each joint.
These are set in the firmware, defined as degrees from Zero (servo centered).

.. table::
    :align: center

    +--------------+-------+-------+-------------+
    | Joint        | Min   | Max   | Servo ID(s) |
    +==============+=======+=======+=============+
    | Waist        | -180  | 180   | 1           |
    +--------------+-------+-------+-------------+
    | Shoulder     | -101  | 101   | 2+3         |
    +--------------+-------+-------+-------------+
    | Elbow        | -101  | 92    | 4+5         |
    +--------------+-------+-------+-------------+
    | Wrist Angle  | -107  | 130   | 6           |
    +--------------+-------+-------+-------------+
    | Forearm Roll | -180  | 180   | 7           |
    +--------------+-------+-------+-------------+
    | Wrist Rotate | -180  | 180   | 8           |
    +--------------+-------+-------+-------------+
    | Gripper      | 42mm  | 116mm | 9           |
    +--------------+-------+-------+-------------+

Default Servo Configurations
============================

.. csv-table::
    :file: ../_data/servos_avx300s.csv
    :header-rows: 1
    :widths: 5 10 10 10
    :align: center

.. Kinematic Properties
.. ====================

.. Product of Exponentials
.. -----------------------

.. `Read more about the product of exponential approach.`_


.. .. math::

..     M & =
..     \begin{bmatrix}
..     1.0 & 0.0 & 0.0 & 0.536494 \\
..     0.0 & 1.0 & 0.0 & 0.0 \\
..     0.0 & 0.0 & 1.0 & 0.42705 \\
..     0.0 & 0.0 & 0.0 & 1.0
..     \end{bmatrix}

.. .. math::

..     Slist & =
..     \begin{bmatrix}
..     0.0 & 0.0 & 1.0 &  0.0      & 0.0      & 0.0 \\
..     0.0 & 1.0 & 0.0 & -0.12705  & 0.0      & 0.0 \\
..     0.0 & 1.0 & 0.0 & -0.42705  & 0.0      & 0.05955 \\
..     1.0 & 0.0 & 0.0 &  0.0      & 0.42705  & 0.0 \\
..     0.0 & 1.0 & 0.0 & -0.42705  & 0.0      & 0.35955 \\
..     1.0 & 0.0 & 0.0 &  0.0      & 0.42705  & 0.0
..     \end{bmatrix}^T

.. .. _`Read more about the product of exponential approach.`: https://en.wikipedia.org/wiki/Product_of_exponentials_formula

Drawings and CAD Files
======================

.. image:: images/avx300s_drawing.png
    :align: center
    :width: 70%

:download:`ALOHA ViperX-300 6DOF Technical Drawing </_downloads/ALOHA ViperX-300s.pdf>`

.. .. raw:: html

..     <iframe
..         src="https://trossenrobotics.autodesk360.com/shares/public/SH56a43QTfd62c1cd9680d0d9bb438fff39a?mode=embed"
..         width="100%"
..         height="600px"
..         allowfullscreen="true"
..         webkitallowfullscreen="true"
..         mozallowfullscreen="true"
..         frameborder="0">
..     </iframe>

.. - :download:`ViperX-300 6DOF Solid STEP Files </_downloads/solids/9_VXA-300S-M.zip>`
.. - `ViperX-300 6DOF Mesh STL Files <https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes/vx300s_meshes>`_
