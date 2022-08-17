# IITISoC-22
Autonomous Vehicles - Last Mile Delivery Drone

# Install ROS
# Set Up Catkin workspace

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

#Clone ArduPilot

In home directory:

cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

Checkout Latest Copter Build

git checkout Copter-4.0.4
git submodule update --init --recursive

# Install Gazebo

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list
Setup keys:

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

Reload software list:

sudo apt update

Install Gazebo:

sudo apt-get install gazebo11 libgazebo11-dev



# CODES 

# FOR TAKE OFF
#! /usr/bin/env python
import rospy
import time
import actionlib

from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class MoveClass(object):
    
  
   _feedback = CustomActionMsgFeedback()
   _result   = CustomActionMsgResult()

   def __init__(self):
    
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

# FOR MOVING IN SQUARE

#! /usr/bin/env python

import rospy

from iq_gnc.py_gnc_functions import *

from iq_gnc.PrintColours import *


def main():
   
    rospy.init_node("drone_controller", anonymous=True)

    
    drone = gnc_api()
    drone.wait4connect()
    drone.wait4start()

   
    drone.initialize_local_frame()
    drone.takeoff(3)
    rate = rospy.Rate(3)

   
    goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
             [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
   
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
      
# FOR MOVING FROM ONE POINT TO OTHER

#! /usr/bin/env python
import math

import rospy
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *


def main():
   
    rospy.init_node("drone_controller", anonymous=True)

  
    drone = gnc_api()
    drone.wait4connect()
    drone.wait4start()
    
    rate = rospy.Rate(3)

  
    a=float(0)
    b=float(1)
    d=float(0)
    c=float(input("enter value of z= "))
    drone.takeoff(c)
    drone.set_speed(1)


    rate.sleep()
    i=int(0)
    while i<1:
      p=float(input("enter value of x= "))
      q=float(input("enter value of y= "))    
      u=float(p-a)
      v=float(q-d)
      j=float(math.sqrt((a*a+b*b)*(u*u+v*v)))
      r=float(math.acos((a*u+b*v)/j))*180/math.pi
      t=float(a*v-b*u)
      if t>0:
         drone.initialize_local_frame()
         drone.set_destination(x=p,y=q,z=c,psi=r)         
         a==p
         b==q
         d==q
      else:
         drone.initialize_local_frame()
         drone.set_destination(x=p,y=q,z=c,psi=-r)        
         a==p
         b==q
         d==q

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
        
        
# FOR OBSTACLE REPEL

#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from iq_gnc.py_gnc_functions import *
from math import cos, sin, pow, radians, sqrt
from geometry_msgs.msg import Point

drone = gnc_api()

def laser_cb(msg):
    # Callback function of the subscriber.
    cr_scan = LaserScan()
    cr_scan = msg
    avoid_x = 0.0
    avoid_y = 0.0
    avoid = False

    for i in range(1, len(cr_scan.ranges)):
        d0 = 3.0
        k = 0.5
        if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 0.35:
            avoid = True
            x = cos(cr_scan.angle_increment * i)
            y = sin(cr_scan.angle_increment * i)
            u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))

            avoid_x += (x*u)
            avoid_y += (y*u)

    # Getting the current_heading of the drone and converting it to radians.
    cr_heading = radians(drone.get_current_heading())
    avoid_x = (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading))
    avoid_y = (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading))

    if avoid:
        dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))

        if dist > 3:
            avoid_x = (3 * (avoid_x/dist))
            avoid_y = (3 * (avoid_y/dist))

        cur_pose = Point()
        # Getting the current location from the drone.
        cur_pose = drone.get_current_location()
        # Sending the goal.
        drone.set_destination(avoid_x + cur_pose.x,
                              avoid_y + cur_pose.y,
                              2, 0)


def main():
    # Initializing the ROS node.
    rospy.init_node("obs_avoider", anonymous=True)
    # Creating a subscriber for the topic '/spur/laser/scan'.
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)

    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 2m.
    drone.takeoff(2)

    # Used to keep the node running.
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
        
# FOR OBSTACLE AVOIDENCE

#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from iq_gnc.py_gnc_functions import *
from iq_gnc.rectangle import *
import math

from geometry_msgs.msg import Point


drone = gnc_api()
r2d=float(math.pi/180)


def laser_cb(msg):
   
    cr_scan = LaserScan()
    cr_scan = msg
    avoid = False
    for i in range(math.floor(88*r2d/cr_scan.angle_increment),math.floor(92*r2d/cr_scan.angle_increment)):
        l = 15
        if cr_scan.ranges[i] < l:
            avoid = True
            drone.set_speed(0.01)
            break

    

    if avoid:
        list1=[]
        for k in range(math.floor(90*r2d/cr_scan.angle_increment),math.floor(180*r2d/cr_scan.angle_increment)):
            if cr_scan.ranges[k]<15:
                list1.append(cr_scan.ranges[k])
            else:
                break
        list2=[]
        for t in range(0,math.floor(90*r2d/cr_scan.angle_increment)):
            if cr_scan.ranges[math.floor(90*r2d/cr_scan.angle_increment)-t]<15:
              list2.append(cr_scan.ranges[math.floor(90*r2d/cr_scan.angle_increment)-t])
            else:
               break
        right=False
        left=False
        if list1[len(list1)-1]>list2[len(list2)-1]:
           right=True
        else:
           left=True
 
        avoided=False
        cur_pose = Point()
        if right:
            drone.initialize_local_frame()
            drone.set_destination(x=list2[len(list2)-1]*math.cos(math.pi/2-len(list2)*cr_scan.angle_increment)+1,y=list2[len(list2)-1]*math.sin(math.pi/2-len(list2)*cr_scan.angle_increment)+1,z=0,psi=0)
            rospy.sleep(list2[len(list2)-1]+1)
            lq=int(0)
            lp=int(0)
            while lp<1:
               if cr_scan.ranges[math.floor(math.pi/cr_scan.angle_increment)]<3:
                   drone.initialize_local_frame()
                   drone.set_speed(1)
                   drone.set_destination(x=0,y=lq,z=0,psi=0)
                   lq=lq+2
               else:
                    avoided=True
                    break
        elif left:
            drone.initialize_local_frame()
            drone.set_speed(1)
            drone.set_destination(x=-list1[len(list1)-1]*math.cos(math.pi/2-len(list1)*cr_scan.angle_increment)-1,y=list1[len(list1)-1]*math.sin(math.pi/2-len(list1)*cr_scan.angle_increment)+1,z=0,psi=0)
            rospy.sleep(list2[len(list2)-1]+1)
            q=int(0)
            lp=int(0)
            while lp<1:
                if cr_scan.ranges[0]<3:
                   drone.initialize_local_frame()
                   drone.set_destination(x=0,y=q,z=0,psi=0)
                   q=q+2
                else:
                   avoided=True
                   break

 

def main():
   
    rospy.init_node("obs_avoider", anonymous=True)
   
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)


  
    drone.wait4connect()
    drone.wait4start()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
        
# Link of the demonstration vedio

https://drive.google.com/file/d/1X2NHQ1RkdBcuFMDrjGt16V-P1sxjAm6j/view?usp=sharing
