#! /usr/bin/env python
import rospy
import time
import actionlib

from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class MoveClass(object):
    
   # create messages that are used to publish feedback/result
   _feedback = CustomActionMsgFeedback()
   _result   = CustomActionMsgResult()

   def __init__(self):
     # creates the action server
     self._as = actionlib.SimpleActionServer("/action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
 
     self._as.start()
     self.ctrl_c = False
     self.rate = rospy.Rate(10)

    
   def goal_callback(self, goal):

     c=goal.goal
 
     success = True
     r = rospy.Rate(1)   

     self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
     self._takeoff_msg = Empty()
     self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
     self._land_msg = Empty()
      
     if c == "TAKEOFF":
         self._pub_takeoff.publish(self._takeoff_msg)
         rospy.loginfo('Taking off...')
         self._as.publish_feedback(self._feedback)
            

     if c == "LAND":
         self._pub_land.publish(self._land_msg)
         rospy.loginfo('Landing...')
         self._as.publish_feedback(self._feedback)
         


     if success:
         self._result = Empty()
         self._as.set_succeeded(self._result)

     
     r.sleep()
        

rospy.init_node('action_custom_msg')
MoveClass()
rospy.spin()
