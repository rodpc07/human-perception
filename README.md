# human-perception

This ROS package facilitates the representation of humans detected by [ROS4HRI](https://github.com/ros4hri/) as Collision Objects within [MoveIt](https://github.com/ros-planning/moveit)'s planning scene. This integration enhances safety in collaborative tasks by ensuring that the robot's motion planning takes into account the presence of humans.

## Parameters
### Collision Mode Configuration (Default: 0)

The `collision_mode` parameter enables you to define the collision geometry for representing humans in the environment. It offers two modes:

#### Collision Mode 0 (Default)
- When `collision_mode := 0`, the human body is entirely enclosed within a BOX Collision Object.
- **Less** computational intensive.

#### Collision Mode 1
- When `collision_mode := 1`, each part of the human body is encapsulated within a CYLINDER Collision Object, except for the head, which is enclosed by a SPHERE Collision Object.
- **More** computational intensive.

## Run

The human_perception node requires the presence of an active MoveIt planning_scene and the running ROS4HRI's hri_fullbody node to function properly.

### rosrun

`rosrun human_perception human_perception collision_mode := VALUE`

### roslaunch

```
<node name="human_perception" pkg="human_perception" type="human_perception" output="screen">
    <param name="collision_mode" type="int" value="0 or 1" />
</node>
```
