#! /usr/bin/env python3
import rospy  
from sensor_msgs.msg import JointState
class CommandToJointState:
    def __init__(self):
        self.inv_joint_pub = rospy.Publisher("/inv_arm/joint_states", JointState, queue_size=1)
      
        self.gazebo_joint_sub = rospy.Subscriber("/joint_states", JointState,
                                            self.joint_callback, queue_size=1)
    def joint_callback(self,msg):
        msg.header.stamp = rospy.Time.now()
        self.inv_joint_pub.publish(msg)
       
if __name__ == '__main__':
    rospy.init_node('inverted_joint_state_publisher')
    command_to_joint_state = CommandToJointState()
    rospy.spin()