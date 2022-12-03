====================
interbotix_xs_driver
====================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_xs_driver/tree/ros2"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../../../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

The Interbotix X-Series Driver provides low-level interfaces to easily control the DYNAMIXEL servos
on any Interbotix X-Series Robot.

Usage
=====

The X-Series Driver is compiled as a library and can be used in any C++ project by simply including
its headers and linking against it.

.. code-block:: c++

    #include "interbotix_xs_driver/xs_logging.hpp"  // Logging macros and utils
    #include "interbotix_xs_driver/xs_common.hpp"   // Common variables and types
    #include "interbotix_xs_driver/xs_driver.hpp"   // The InterbotixDriverXS class


.. code-block:: cmake

    find_package(interbotix_xs_driver REQUIRED)
    add_executable(your_executable)
    ament_target_dependencies(your_executable interbotix_xs_driver ...)

Then create an `InterbotixDriverXS` object, providing the following in the order stated:

*   Absolute filepath to the :ref:`motor configs file <motor_configs_file_ros2>`
*   Absolute filepath to the :ref:`mode configs file <mode_configs_param_ros2>`
*   A boolean indicating whether the Driver should :ref:`write to the EEPROM on startup
    <load_configs_param_ros2>`
*   A string indicating the driver's logging level containing one of the following: "DEBUG",
    "INFO", "WARN", "ERROR", "FATAL"

This initialization would look something like below:

.. code-block:: c++

    std::unique_ptr<InterbotixDriverXS> xs_driver = std::make_unique<InterbotixDriverXS>(
        filepath_motor_configs,
        filepath_mode_configs,
        write_eeprom_on_startup,
        logging_level);

See the package's source code for more details.
