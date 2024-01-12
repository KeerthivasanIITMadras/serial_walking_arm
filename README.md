### To launch the gazebo environment containing the arm
```roslaunch b47_walking_control gazebo.launch```
### To attach the base link to the ground plane to mimic attaching
```rosservice call /link_attacher_node/attach "model_name_1:'ground_plane'link_name_1: 'link' model_name_2: 'robot'link_name_2: 'base_link'"```
### To run the execution wrt to base_link
```rosrun b47_walking_control move.py```
### To run the execution wrt to end effector
``` ROS_NAMESPACE=/inv_arm rosrun b47_walking_control move_inv.py```

### To run the combined launcher 
This runs both the move groups simultaneously meaning it can be controlled with a single script

```rosrun b47_walking_control combined.py```

### To run the teleop node

```rosrun b47_walking_control teleop.py```

### Note:
The gazebo.launch runs a script which maps the joints for inverse end effector planning, no need to run separately

The link attacher should be used only on some models created not on the ground plane



https://github.com/KeerthivasanIITMadras/serial_walking_arm/assets/94305617/90870c0a-956d-4b80-9f26-164fa5ff5917

