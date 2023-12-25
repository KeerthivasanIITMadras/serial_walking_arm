#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import sys
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class Move_Arm:
    def __init__(self):
        joint_state_topic = ['joint_states:=/inv_arm/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("inv_arm")
        moveit_commander.roscpp_initialize(sys.argv)
        self.group.set_pose_reference_frame('link_05')
        pass
    
    def give_goal(self,p_x,p_y,p_z,x,y,z,w):
        end_effector_link = self.group.get_end_effector_link()
        current_pose = self.group.get_current_pose(end_effector_link).pose
        print("End Effector Position:")
        print("X:", current_pose.position.x)
        print("Y:", current_pose.position.y)
        print("Z:", current_pose.position.z)
    
        print("\nEnd Effector Orientation (Quaternion):")
        print("X:", current_pose.orientation.x)
        print("Y:", current_pose.orientation.y)
        print("Z:", current_pose.orientation.z)
        print("W:", current_pose.orientation.w)

        pose_goal = Pose()
        pose_goal.orientation.w = w
        pose_goal.orientation.x = x
        pose_goal.orientation.y = y
        pose_goal.orientation.z = z
        pose_goal.position.x = p_x
        pose_goal.position.y = p_y
        pose_goal.position.z = p_z
        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

class Attacher:
    def __init__(self):
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service()

    def attach_request(self,model_name_1,link_name_1,model_name_2,link_name_2):
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        self.attach_srv.call(req)
    
    def detach_request(self,model_name_1,link_name_1,model_name_2,link_name_2):
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        self.detach_srv.call(req)


if __name__ == "__main__":
    rospy.init_node("move_arm")
    rospy.loginfo("Starting move_arm")
    attacher = Attacher()
    attacher.detach_request("unit_box","link","robot","base_link")
    attacher.attach_request("unit_box_clone","link","robot","link_05")
    move_arm = Move_Arm()
    move_arm.give_goal(-1.3936,-0.10464,0.4031,0,1,0,0)
