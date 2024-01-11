#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import os

class Move_Arm:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander(robot_description="/robot_description")
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_pose_reference_frame('base_link')

        '''HINT:http://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html'''
        
        self.robot_inv = moveit_commander.RobotCommander()
        self.group_inv = moveit_commander.MoveGroupCommander("inv_arm",ns="/inv_arm",robot_description="/inv_arm/robot_description")
        self.group_inv.set_pose_reference_frame('link_05')

    
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

    
    def give_goal_inv(self,p_x,p_y,p_z,x,y,z,w):
        end_effector_link = self.group_inv.get_end_effector_link()
        current_pose = self.group_inv.get_current_pose(end_effector_link).pose
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
    rospy.init_node("combined")
    rospy.loginfo("Starting move_arm")
    attacher = Attacher()
    attacher.attach_request("test_surface","link","robot","base_link")
    move_arm = Move_Arm()
    move_arm.give_goal(1.3936,0,0.2031,0,1,0,0)
    attacher.detach_request("test_surface","link","robot","base_link")
    attacher.attach_request("test_surface","link","robot","link_05")
    move_arm = Move_Arm()
    move_arm.give_goal_inv(-1.3936,0,0.2031,0,1,0,0)
    attacher.detach_request("test_surface","link","robot","link_05")