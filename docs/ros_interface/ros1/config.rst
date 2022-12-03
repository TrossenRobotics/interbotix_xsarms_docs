===================
ROS 1 Configuration
===================

Network Configuration
=====================

ROS_IP
------

.. code-block:: bash

    export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)
    if [ -z "$ROS_IP" ]; then
        export ROS_IP=127.0.0.1
    fi

X-Series SDK Configuration
==========================

.. _motor_configs_file_ros1:

Motor Configs
-------------

The motors can be configured on startup by providing the :ref:`motor_configs parameter
<motor_configs_param_ros1>` to the xs_sdk. The expected format and explanation of the configuration
options is below:

.. note::

    The xs_sdk :ref:`load_configs parameter <load_configs_param_ros1>` must be set to ``true`` for the
    configurations to take effect.

.. note::

    The motor_configs only need to be set once per robot configuration - meaning that the
    :ref:`load_configs parameter <load_configs_param_ros1>` can be set to ``false`` after the first
    boot.

.. collapse:: Motor Configs Template

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/config/motor_configs_template.yaml

.. _mode_configs_file_ros1:

Mode Configs
------------

The startup operating modes can be set on startup by providing the :ref:`mode_configs parameter
<mode_configs_param_ros1>` to the xs_sdk. The expected format and explanation of the configuration
options is below:

.. collapse:: Mode Configs Template

    .. literalinclude:: /sources/ros1/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/config/mode_configs_template.yaml
