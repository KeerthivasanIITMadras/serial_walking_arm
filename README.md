### To launch the gazebo environment containing the arm
```roslaunch b47_walking_control gazebo.launch```
### To attach the base link to the ground plane to mimic attaching
```rosservice call /link_attacher_node/attach "model_name_1:'ground_plane'link_name_1: 'link' model_name_2: 'robot'link_name_2: 'base_link'"```
### To run the execution wrt to base_link
```rosrun b47_walking_control move.py```
### To run the execution wrt to end effector
```rosrun b47_walking_control move_inv.py```

### Note:
The gazebo.launch runs a script which maps the joints for inverse end effector planning, no need to run separately

The link attacher should be used only on some models created not on the ground plane

