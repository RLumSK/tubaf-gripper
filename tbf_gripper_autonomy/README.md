# Autonomy package of Julius' manipulator

This package focuses on the automation capabilities of Julius manipulation unit.
Often task driven, controllers are implemented to control arm and hand in a certain manner.

## Dependencies:

* [MoveIt!](https://moveit.ros.org/)
* [HaF Grasping](https://github.com/davidfischinger/haf_grasping.git)

## Python Packages

* autonomy
* grasping

## Common launch calls:

```bash
roslaunch tbf_gripper_autonomy hand.launch
roslaunch tbf_gripper_autonomy haf_server.launch
roslaunch tbf_gripper_autonomy controller.launch
```