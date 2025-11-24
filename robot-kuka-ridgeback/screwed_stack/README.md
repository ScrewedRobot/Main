## Screwed Stack


### Background Info
This stack is designed to work with the Kuka iiwa R7 which is integrated with a Clearpath Ridgeback. Although this package can be operated using service calls, it is __highly recommended__ that you use Foxglove Studio which acts as the GUI.

_Note: This package is built for ROS Noetic (because that's what Clearpath used :| )_


### Package Info
The screwed stack is composed of 5 custom packages which each serving a specific purpose:
- [screwed_2dnav](/robot-kuka-ridgeback/screwed_stack/screwed_2dnav/)
    - Launch files and param files for the robots navigation
    - Requires the navigation and slam_toolbox package
- [screwed_endeff](/robot-kuka-ridgeback/screwed_stack/screwed_endeff/)
    - ROS node and launch file to run the end effector motor
    - Note: this node runs as a service on the raspberry pi and will be advertised on the /screwed_stack/command/end_effector topic
- [screwed_msgs](/robot-kuka-ridgeback/screwed_stack/screwed_msgs/)
    - Custom interfaces required by all of the screwed_stack packages
- [screwed_machine](/robot-kuka-ridgeback/screwed_stack/screwed_machine/)
    - Contains state machine, task acquisition, and joystick nodes and launch files
- [screwed_planning](/robot-kuka-ridgeback/screwed_stack/screwed_planning/)
    - Planning/Controller nodes such as api (move_it), direct (velocity control), and force (force control)
    - Various filter implementations and kinematics helper functions


### Installation

1. Follow the Clearpath Quick Start guide provided with this robot.

2. Clone this repository into your `catkin_ws/src` folder.

3. Build the workspace.


### Usage

1. SSH into the Kuka-Ridgeback:

``` ssh administrator@cpr-r100-0150 ```

2. In the SSH terminal, run the robot_side launch file:

``` roslaunch screwed_planning robot_side.launch ```

3. On your device, run the nuc_side launch file:

``` roslaunch screwed_machine nuc_side.launch  ```

4. Open Foxglove Studio and load the `ScrewedUI.json` as your user interface.

5. Load the end effector with a screw.

6. Open the "DEMO UI" panel. Set the goal pose, toggle manual control, and hit navigate!


#### Notes
- Ensure the repository is cloned on both the Kuka-Ridgeback and your device
- If something not building on the Kuka-Ridgeback, Create a `CATKIN_IGNORE` file in the "screwed_machine", "screwed_2dnav", and "screwed_endeff" packages. The robot will not have all of the dependencies to build these packages.


### Contact

For questions or issues related to this stack, please contact dj.holden@queensu.ca.