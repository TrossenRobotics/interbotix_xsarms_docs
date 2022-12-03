========================
ROS 1 Interface Overview
========================

Structure
=========

The core structure of the Interbotix ROS 1 Interface is made up of three key components:

*   :doc:`The ROBOTIS DYNAMIXEL Workbench Toolbox packages <overview/dxl_wb>` - Provides
    easy-to-use interfaces and definitions to control DYNAMIXEL servos.
*   :doc:`The Interbotix X-Series Messages package <overview/xs_msgs>` - Defines messages,
    services, and actions to be used in other X-Series projects.
*   :doc:`The Interbotix X-Series SDK package <overview/xs_sdk>` - Wraps the DYNAMIXEL Workbench
    Toolbox, exposing methods of control via topics and services.

.. toctree::
    :caption: Table of Contents
    :maxdepth: 1

    overview/dxl_wb.rst
    overview/xs_msgs.rst
    overview/xs_sdk.rst
