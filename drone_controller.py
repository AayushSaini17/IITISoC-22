#! /usr/bin/env python
import math
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Specify some waypoints
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
         drone.set_destination(x=p,y=q,z=c,psi=r)         
         a==p
         b==q
         d==q
      else:
         drone.set_destination(x=p,y=q,z=c,psi=-r)        
         a==p
         b==q
         d==q

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
