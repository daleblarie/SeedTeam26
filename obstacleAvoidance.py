# -*- coding: utf-8 -*-
"""
"""

#import summit_xl_joystick.srv
import rospy
import sensor_msgs.msg
import geometry_msgs.msg


import math
import time
import std_msgs
import tf
import numpy as np

class Obstacle_Avoidance():
    def __init__(self, nameSpace = "/", listener = None):
        '''
        @param nameSpace: namespace of the robot (optional, default: '/')
        @param listener: an instance of tf.TransformListener (optional, deafult: None)
        '''
        # Publisher for vel commands - Set up the publisher to publish your velocity commands to your robot
        rospy.loginfo("Registering publisher for robot control.")
        self.velCmdPub = rospy.Publisher("/ROBOT_NAME_PATH/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        self.nameSpace = nameSpace
        rospy.loginfo("Done...")
        
        # Set up the listener for the TF tree. Used to determine the current position of the robot        
        if listener is None:        
            self.listener = tf.TransformListener()
            rospy.sleep(2.) # listener stabilising time
        else:
            self.listener = listener 
        
        # Configure this to set up the subscriber to listen to the laser/sonar readings from your robot
        rospy.loginfo("Waiting for the laser services")
        rospy.Subscriber(self.nameSpace+"scan", sensor_msgs.msg.LaserScan, self.process_laser)
        rospy.loginfo("Done...")
        
        #==============================================================================
        #         LASER VARIABLES
        #==============================================================================
        # Configure this section to set the parameters of your laser or sonar readings
        # These current settings are for a SICK laser scanner mounted on the top of a Summit XL Robot
        # Radians        
        self.laserAngleMaxRad = 2.35619449615
        self.laserAngleMinRad = -2.35619449615
        self.laserAngleInc = 0.00880820397288
        self.laserAnglesRad = np.arange(self.laserAngleMinRad,(self.laserAngleMaxRad+self.laserAngleInc),self.laserAngleInc)        
        self.laserRanges = []

        #==============================================================================
        #         OBSTACLE AVOIDANCE --> POTENTIAL FIELDS VARIABLES        
        #==============================================================================
        # Robot radi limits for obstacle avoidance
        # soflim initiates a stepped slowdown of the linear vel until hardlim is reached
        self.hardLim = 0.7 # This is the radius from the centre of the robot that you wish to consider as the hard safety limit
        self.softLim = 1.6 # This is the radius from the centre of the robot that you wish to consider as the soft safety limit 
        self.goalspeed = 1.5 # This is the maximum speed (in m/s) that you wish the robot to travel at
        self.r = 0.5 # This is the radius of the mobile robot (measured from the centre)
        self.s = 1.0 # This is the safety distance as whihc to start affecting the turning
        self.pf_strength = 1.3 # This is the magnitude of the potential field. Changing this will alter the effect it has on the robot as it approaches an obstacle
        self.rot_speed = math.pi/6 # This is maximum speed at which you want the robot to rotate when it has activated the hard safety zopne and linear velocity is at zero until it rotates to a safe direction to drive away from the obstacle(s)
        
    
    def process_laser(self,laserData):
        '''
        Configure this callback function to process the LaserScan messages from your robot
        @param laserData: sensor_msgs.msg.LaserScan
        '''
        Ranges = np.array(laserData.ranges)
        self.laserRanges = Ranges[90:(len(Ranges)-90)]
        self.aheadOnly = Ranges[228:(len(Ranges)-228)] # Currently set at 10 degrees --> 20 readings --> 10 either side of zero
        self.modded_laserAnglesRad = self.laserAnglesRad[90:(len(Ranges)-90)]
        self.laserCount = len(laserData.ranges)
    

    def oba_potential_fields(self):
        beta = self.pf_strength    # Stength of the potential fields i.e. the stronger this is, the more forceful the obstacle avoidance
        dx = 0.0
        dy = 0.0            
        goal_speed = 1.5 # change this to the maximum speed you want your robot to travel at during obstacle avoidance (in m/s)
  
        warning = False
        
        for i in range(len(self.laserRanges)):
            if self.laserRanges[i] <= self.softLim:
                if self.laserRanges[i] <= self.hardLim:
                    warning = True
                
                theta = self.modded_laserAnglesRad[i]
                dx += -beta*((self.s+self.r-self.laserRanges[i])*np.cos(theta))
                dy += -beta*((self.s+self.r-self.laserRanges[i])*np.sin(theta))
                assert(np.isfinite(dx) & np.isfinite(dy))
        
        dx += goal_speed
        vel = np.sqrt(dx**2 + dy**2)
        
        if np.fabs(vel) > goal_speed:
            vel = np.sign(vel)*goal_speed
        if (vel < 0.1):
            rospy.loginfo("Velocity limited")
            vel = 0.1
        
        if (dx != 0.0):
            rot = (np.arctan2(dy,dx))/2.0
        else:
            rospy.loginfo("OBA - DX = 0")
            rot = ((-1*np.sign(dy))*(np.pi/2.0))/2.0
        
        if (np.isnan(vel) or np.isnan(rot)):
            rospy.loginfo("NAN: vel: %2f, rot: %2f" %(vel, rot))
        if warning:
            vel = 0.0
            rot = np.sign(rot)*0.25
            rospy.loginfo("WARNING - OBSTACLE DETECTED")
            warning = False
     
        
if __name__ == "__main__":
    nodeName = "NAME_OF_EXPERIMENT"
    rospy.init_node(nodeName, anonymous=True)
    rospy.sleep(2.) # listener stabilising time
    rospy.Rate(10) # Vary the ros rate
    
Obstacle_Avoidance()
""" LOG FILES STORED IN ~/.ros/log/ """