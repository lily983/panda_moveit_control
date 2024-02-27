## Usage for PRIMP-STOMP projects
Give motion_primitive, mode, object, etc values from arguments.
```
roslaunch panda_moveit_control stomp_execution.launch [args]
```
For example, for door opening task(sink_door)
```
roslaunch panda_moveit_control stomp_execution.launch motion_primitive:=opening mode:=rotating_left object:=sink_door
```
