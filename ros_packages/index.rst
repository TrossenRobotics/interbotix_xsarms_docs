========================
ROS Open Source Packages
========================

.. note::

    To use any of these packages, you must already have ROS and the X-Series Arm packages
    installed. If you do not have these installed, follow the steps detailed in the :doc:`ROS
    Interface Setup<../ros_interface/index>`.

Below is a list of all ROS packages meant to be used with the many X-Series robotic arms sold by
Trossen Robotics. Packages were tested on Ubuntu Linux 18.04 and 20.04 using ROS Melodic and Noetic
respectively. Additionally, all ROS nodes were written using Python or C++. However, any
programming language capable of sending ROS messages can be used to control the robots. The core
packages inside this repo are as follows:

-   :doc:`interbotix_xsarm_descriptions<./arm_descriptions>` - contains the meshes and URDFs
    (including accurate inertial models for the links) for all arm platforms
-   :doc:`interbotix_xsarm_control<./arm_control>` - contains the motor configuration files and the
    'root' launch file that is responsible for launching the robot arm
-   :doc:`interbotix_xsarm_gazebo<./gazebo_simulation_configuration>` - contains the config files
    necessary to launch an arm in Gazebo, including tuned PID gains for the ros_control package
-   :doc:`interbotix_xsarm_ros_control<./ros_control>` - contains configuration files necessary to
    set up ROS controllers between MoveIt and the physical robot arm
-   :doc:`interbotix_xsarm_moveit<./moveit_motion_planning_configuration>` - contains the config
    files necessary to launch an arm using MoveIt either in Gazebo, on the physical robot, or just
    in RViz

There are also several packages demonstrating possible applications of the core packages. A list of
those packages is below in order of importance:

.. toctree::
    :maxdepth: 1

    arm_descriptions.rst
    arm_control.rst
    gazebo_simulation_configuration.rst
    ros_control.rst
    moveit_motion_planning_configuration.rst
    perception_pipeline_configuration.rst
    moveit_interface_and_api.rst
    python_demos.rst
    matlab_demos.rst
    joystick_control.rst
    record_and_playback.rst
    arm_diagnostic_tool.rst
    arm_diagnostic_listener.rst
    pid_gains_test_environment.rst
    arm_puppeteering.rst
    dual_arm_control.rst
    dual_arm_joystick_control.rst

.. raw:: html

    <!--X-SERIES CAROUSEL LIVE-->
    <h1 class="xseries-carousel-title-textbox">OPEN SOURCE DEMOS</h1>
    <div class="carousel-content-container">
        <div id="flex-scene">
            <div id="left-zone">
                <ul class="xseries-carousel-list">

                    <!-- Motion Planning -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control3" name="basic_carousel" checked="checked" />

                        <label class="static-container" for="xseries-radio-control3" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/xsarm_moveit_interface.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/xsarm_moveit_interface.gif">
                            </div>
                            <p class="xseries-left-label">Motion Planning</p>
                        </label>

                        <div class="content content_sec3">
                            <span class="xseries-right-pic3"></span>
                            <h1 class="xseries-right-title3-h1">MoveIt Motion Planning Configuration</h1>
                            <p class="xseries-right-text-p">Contains the necessary config files to get any of the X-Series
                                arms working with MoveIt, the standard motion planning framework for ROS. This package also
                                provides interfaces to work with the arms in a Gazebo simulation.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_moveit"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=Z9s3hRYcIp0" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/moveit_motion_planning_configuration.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Gazebo Simulation -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control1" name="basic_carousel"/>

                        <!--<label class="xseries-label1" for="xseries-radio-control1" style="display: flex;">-->
                        <label class="static-container" for="xseries-radio-control1" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/gazebo_arm.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/gazebo_arm.gif">
                            </div>
                            <p class="xseries-left-label">Gazebo Simulation</p>
                        </label>

                        <div class="content content_sec1">
                            <span class="xseries-right-pic1"></span>
                            <h1 class="xseries-right-title1-h1">Gazebo Simulation Configuration</h1>
                            <p class="xseries-right-text-p">Contains configuration files, launch files, and example worlds
                                to simulate the X-Series arms in Gazebo. Also has position and trajectory controllers with
                                tunable PID gains so that ROS can control the arms.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_gazebo"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=k3zkgN7TYTE" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/gazebo_simulation_configuration.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Joystick Control -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control2" name="basic_carousel" />

                        <label class="xseries-label2" for="xseries-radio-control2" style="display: flex;">
                            <p class="xseries-left-label">Joystick Control</p>
                        </label>

                        <div class="content content_sec2">
                            <span class="xseries-right-pic2"></span>
                            <h1 class="xseries-right-title2-h1">Joystick Control</h1>
                            <p class="xseries-right-text-p">Control any X-Series arm using a Bluetooth joystick controller.
                                This package can be used to control the movements of any X-Series robotic arm using a SONY
                                PS3/PS4 controller or Microsoft Xbox360 controller (untested) via Bluetooth.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_joy"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=AyKjcZvu8lo" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/joystick_control.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Dual Arm Control -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control4" name="basic_carousel" />

                        <label class="static-container" for="xseries-radio-control4" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/dual_arm_control.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/dual_arm_control.gif">
                            </div>
                            <p class="xseries-left-label">Dual Arm Control</p>
                        </label>

                        <div class="content content_sec4">
                            <span class="xseries-right-pic4"></span>
                            <h1 class="xseries-right-title4-h1">Dual Arm Control</h1>
                            <p class="xseries-right-text-p">Enables the control of multiple X-Series arms using scripts.
                                Configuration files are provided that allow for 2 or more arms to be controlled
                                simultaneously using multithreading techniques.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_dual"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=DnjbNXxBE_8" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/dual_arm_control.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Perception Pipeline Configuration -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control5" name="basic_carousel" />

                        <label class="static-container" for="xseries-radio-control5" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/python_perception_arm.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/python_perception_arm.gif">
                            </div>
                            <p class="xseries-left-label">Perception Configuration</p>
                        </label>

                        <div class="content content_sec5">
                            <span class="xseries-right-pic5"></span>
                            <h1 class="xseries-right-title5-h1">Perception Pipeline Configuration</h1>
                            <p class="xseries-right-text-p">Contains the necessary config and launch files to get any of the
                                many Interbotix X-Series arms working with the perception pipeline, allowing for the arms to
                                interact with depth cameras such as the RealSense D415.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_perception"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=03BZ6PLFOac" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/perception_pipeline_configuration.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- MoveIt Interface and API -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control6" name="basic_carousel" />

                        <label class="xseries-label6" for="xseries-radio-control6" style="display: flex;">
                            <p class="xseries-left-label">MoveIt Interface</p>
                        </label>

                        <div class="content content_sec6">
                            <span class="xseries-right-pic6"></span>
                            <h1 class="xseries-right-title6-h1">MoveIt Interface and API</h1>
                            <p class="xseries-right-text-p">Contains a small API that allows a user to command desired
                                end-effector poses to an Interbotix arm. Also has a modified version of the Move Group
                                Python Interface Tutorial script for users to understand the basics of MoveIt Commander.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_moveit_interface"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=Z9s3hRYcIp0" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/moveit_interface_and_api.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Arm Diagnostic Tool -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control7" name="basic_carousel" />

                        <label class="static-container" for="xseries-radio-control7" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/diagnostic_tool_arm.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/diagnostic_tool_arm.gif">
                            </div>
                            <p class="xseries-left-label">Arm Diagnostic Tool</p>
                        </label>

                        <div class="content content_sec7">
                            <span class="xseries-right-pic7"></span>
                            <h1 class="xseries-right-title7-h1">Arm Diagnostic Tool</h1>
                            <p class="xseries-right-text-p">Analyze the state of one joint or any number of joints over time
                                while running through a script, or while playing back a ROS bag file. Visualize different
                                parameters such as temperature, position, and effort over time.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_diagnostic_tool"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=0N85vMS8LMU" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/arm_diagnostic_tool.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- PID Gains -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control8" name="basic_carousel" />

                        <label class="xseries-label8" for="xseries-radio-control8" style="display: flex;">
                            <p class="xseries-left-label">PID Gains</p>
                        </label>

                        <div class="content content_sec8">
                            <span class="xseries-right-pic8"></span>
                            <h1 class="xseries-right-title8-h1">PID Gains Test Environment</h1>
                            <p class="xseries-right-text-p">A way to test 'pwm' or 'current' PID gains when operating the
                                arm in either 'pwm' or 'current' mode. The package moves the robot through a series of poses
                                as a way to test the effects of different gains and control modes.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_pid"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=U0v_0ZyX-SI" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/pid_gains_test_environment.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Dual Arm Joystick Control -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control9" name="basic_carousel" />

                        <label class="static-container" for="xseries-radio-control9" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/dual_arm_joystick.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/dual_arm_joystick.gif">
                            </div>
                            <p class="xseries-left-label">Dual Arm Joystick</p>
                        </label>

                        <div class="content content_sec9">
                            <span class="xseries-right-pic9"></span>
                            <h1 class="xseries-right-title9-h1">Dual Arm Joystick Control</h1>
                            <p class="xseries-right-text-p">Control two X-Series arms using a single joystick controller.
                                Builds on top of the Joystick and Dual Control packages and contains only configuration and
                                launch files, an example of ROS's modularity.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_dual_joy"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=AyKjcZvu8lo" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/dual_arm_joystick_control.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Arm Puppeteering -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control10" name="basic_carousel" />

                        <label class="static-container" for="xseries-radio-control10" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/puppet.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/puppet.gif">
                            </div>
                            <p class="xseries-left-label">Arm Puppeteering</p>
                        </label>

                        <div class="content content_sec10">
                            <span class="xseries-right-pic10"></span>
                            <h1 class="xseries-right-title10-h1">Arm Puppeteering</h1>
                            <p class="xseries-right-text-p">Imagine that you have two (or more) Interbotix arms with the
                                same number of joints. What the xsarm_puppet allows you to do is to manually manipulate one
                                of the arms and watch as the motion is repeated in real time on the second robot.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=DnjbNXxBE_8" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/arm_puppeteering.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Record and Playback -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control11" name="basic_carousel" />

                        <label class="xseries-label11" for="xseries-radio-control11" style="display: flex;">
                            <p class="xseries-left-label">Record and Playback</p>
                        </label>

                        <div class="content content_sec11">
                            <span class="xseries-right-pic11"></span>
                            <h1 class="xseries-right-title11-h1">Record and Playback</h1>
                            <p class="xseries-right-text-p">Untorque your arm and record manual movements to a ROS bag file.
                                Play back the same motions as many times as you want.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=DnjbNXxBE_8" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/arm_puppeteering.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>

                    <!-- Python Demos -->
                    <li class="xseries-carousel-item">
                        <input type="radio" id="xseries-radio-control12" name="basic_carousel" />

                        <label class="static-container" for="xseries-radio-control12" style="display: flex;">
                            <div class="carousel-thumbnail-container">
                                <img class="static"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/python_demos.png">
                                <img class="active-ca"
                                    src="https://trossenrobotics.com/Shared/X-Series content carousel/python_demos.gif">
                            </div>
                            <p class="xseries-left-label">Python Demos</p>
                        </label>

                        <div class="content content_sec12">
                            <span class="xseries-right-pic12"></span>
                            <h1 class="xseries-right-title12-h1">Python Demos</h1>
                            <p class="xseries-right-text-p">Showcase of examples for the Interbotix Python Arm Module. We
                                made Python wrappers for the X-Series arm that allow users to control their arm without
                                writing a single line of ROS code Module.</p>

                            <div class="footer-social-flex">
                                <div style="display: flex;">
                                    <figure class="xseries-social-icon1">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/python_demos"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/github-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon2">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.youtube.com/watch?v=KoqBEvz4GII" target="_blank"
                                            rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/youtube-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>

                                <div style="display: flex;">
                                    <figure class="xseries-social-icon3">
                                        <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-grey.png" class="grey-icon"
                                            data-image="0">

                                        <a href="https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/python_demos.html"
                                            target="_blank" rel="noopener noreferrer">
                                            <img src="https://trossenrobotics.com/Shared/Social%20Media%20Icons/book-blue.png" class="blue-icon"
                                                data-image="0">
                                        </a>
                                    </figure>
                                </div>
                            </div>
                        </div>
                    </li>
                </ul>
            </div>

            <div id="middle-border" style="display: none;"></div>
            <div id="right-zone"></div>
        </div>
    </div>

Contributing
============

To contribute your own custom X-Series arm in this repo, you will need to do the following steps:

-   Create a motor config file similar to the YAML files found in `xsarm_control/config`_. To get
    familiar with the parameter names, checkout the `Motor Config Template`_. Note that the name of
    this file is what defines your robot_model name, and should be used when naming other files
    like the URDF.
-   Create a URDF similar in structure to the ones found in `xsarm_descriptions/urdf`_. Don't
    forget to put all necessary meshes in the `xsarm_descriptions/meshes`_ directory! Follow the
    naming convention for the links, joints, and frame poses as found in the other arm files for
    consistency.
-   Create a set of `Gazebo/ROS position controllers`_.
-   Create a set of `Gazebo/ROS trajectory controllers`_.
-   Create an `SRDF file`_ for Moveit. You should first use the MoveIt Setup Assistant Wizard for
    this step and then edit the generated SRDF file based on the structure of those files.
-   Add the appropriate Screw axes and M matrices to the `mr_descriptions.py`_ and
    `mr_descriptions.m`_ modules. For help doing this, refer to Chapter 4 in `Modern Robotics`_ and
    `this video`_, or check out our `kinematics_from_description`_ tool.
-   Make sure to follow the same naming convention, structure, and documentation procedures as
    found in the repo before making a PR.

.. _`xsarm_control/config`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_control/config
.. _`Motor Config Template`: https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_sdk/config/motor_configs_template.yaml
.. _`xsarm_descriptions/urdf`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf
.. _`xsarm_descriptions/meshes`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes
.. _`Gazebo/ROS position controllers`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_gazebo/config/position_controllers
.. _`Gazebo/ROS trajectory controllers`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_gazebo/config/trajectory_controllers
.. _`SRDF file`: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_moveit/config/srdf
.. _`mr_descriptions.py`: https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/mr_descriptions.py
.. _`mr_descriptions.m`: https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/mr_descriptions.m
.. _`Modern Robotics`: http://hades.mech.northwestern.edu/images/7/7f/MR.pdf
.. _`this video`: https://www.youtube.com/watch?v=cKHsil0V6Qk&ab_channel=NorthwesternRobotics
.. _`kinematics_from_description`: https://github.com/Interbotix/kinematics_from_description
