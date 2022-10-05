# Block World

### Instructions to Run
`roslaunch block_world block_world.launch`
 To spawn objects: `rossrv call /spawn_blocks`
 To stack objects:
 This is implemented as an Action service. You can call it from the command line as follows:
 ```
 rostopic pub /stack_blocks/goal block_world/StackBlocksActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  top: 'Block2'
  bottom: 'Block1'" 
 ```
 This will stack Block2 on top of Block1.
 
 To reset scene: `rosservice call block_world /reset`
