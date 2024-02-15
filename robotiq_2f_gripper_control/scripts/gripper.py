#!/usr/bin/env python
import sys
import time
import rospy

from robotiq_2f_gripper_msgs.msg import RobotiqGripperCommand,CommandRobotiqGripperAction,CommandRobotiqGripperGoal # CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
#from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
import actionlib
#from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
#from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


class GripperPosControl:
    def __init__(self):
        rospy.init_node('robotiq_2f_client')
        #rospy.init_node("gripper") #ur3_joints
        action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction) #CommandRobotiqGripperAction
      
        # Wait until grippers are ready to take command
        self.robotiq_client.wait_for_server()
        rospy.logwarn("Client test: Starting sending goals")
      
        ## ROS Subscriber Topic
        # Subscribe to the ar_pose_marker topic to get the image width and height

        self.ur_sub_gripper = rospy.Subscriber('gripper',RobotiqGripperCommand, self.callback_gripper) #CommandRobotiqGripperGoal
        
        ## ROS parameters
        self.rate = rospy.Rate(1) # Hz
       
        self.Position_control()
        
    def callback_gripper(self, msg): 
        #обработка смс 
        goal = CommandRobotiqGripperGoal()   
        goal.emergency_release = False
        goal.stop = False
        goal.position = msg.position
        goal.speed = msg.speed
        goal.force = msg.force
        self.robotiq_client.send_goal(goal) 
        self.robotiq_client.wait_for_result()  
        
    def Position_control(self):
        """ Control the robot"""
        while not rospy.is_shutdown():
            self.rate.sleep()
        rospy.signal_shutdown("We are done here!")

if __name__ == '__main__':

    try:
        client = GripperPosControl()
        client.Position_control()
    except rospy.ROSInterruptException:
        pass


