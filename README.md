# Block World
## Let's stack 'em...

### Instructions to run
Launch RVIZ and Movegroup for the Panda Arm - no user setup needed - using,
`roslaunch block_world block_world.launch`
This will advertise the following servers:
* `rosservice call /spawn_blocks`
* `rosservice call /reset`
* Action Server `/stack_blocks`

We provide a simple testing script to demonstrate the spawning and stacking abilities. After launching RVIZ, run this,
```
rosrun block_world test_stacking.py
```
Alternatively, you may use the CLI to interact with the robot.
***
##### Some Caveats
**How to grasp objects?**
Consider the case where certain grasps are infeasible. To work around this, we encode a grasping region for each block (for eg, grasp from any side). Then, we use OMPL Lazy Goal Sampling to instantiate goal samples from these regions. This significantly ammortizes planning time. Same logic applies for placement.

**Avoiding the narrow passage problem**
Sampling-based planners fear narrow passages - i.e. where it's difficult to generate valid samples. If you think about it, grasping is a narrow passage problem given the geometries of the grippers. Hence, we first plan to a preplace configuration and then use simple cartesian planning to grasp. Similar logic applies to placing objects.

I can talk about sampling-based planning all day. Reach me at shlok.sobti@gmail.com.
