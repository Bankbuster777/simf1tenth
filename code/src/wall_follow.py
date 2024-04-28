#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = rospy.get_param("kp")
kd = rospy.get_param("kd")
ki = rospy.get_param("ki")
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
MAX_SCAN_RANGE = 30 # meters for utm-30lx-ew
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 1 # was 0.55
VELOCITY = 2.50 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
LOOKAHEAD_DISTANCE = 0.05
THETA = 45 # degrees

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        # scan_topic = rospy.get_param('scan_topic')
        
        rospy.loginfo("Initial wallfollower")
        scan_topic = '/scan'
        wall_drive_topic = '/wall'
        err_topic = '/err'
        rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback)
        self.max_steering_angle = rospy.get_param("max_steering_angle")
        
        self.drive_pub = rospy.Publisher(wall_drive_topic, AckermannDriveStamped, queue_size=10)
        self.err_pub = rospy.Publisher(err_topic, Float32, queue_size=10)
        

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.
        # TODO: implement
        if (angle > 225) or (angle < -45):
            rospy.loginfo("Out of Angle")
            return 0.0
        
        dist = 0.0
        scan_beam = len(data.ranges)
        # rospy.loginfo("%f, %s", scan_beam, type(scan_beam))
        theta_incr = 360/scan_beam
        for i in range(scan_beam):
            theta = theta_incr * i - 90
            #if ((theta - angle) <= theta_incr/2) and ((angle -theta) <= theta_incr/2):
            #    if data.ranges[i] > MAX_SCAN_RANGE:
            #        dist = MAX_SCAN_RANGE
            #    dist  = data.ranges[i]
            #    break
            if (theta == angle):
                dist = data.ranges[i]
                break

        return dist

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for 

        angle = angle - (kp*error + kd*(prev_error - error) + ki*integral) * self.max_steering_angle

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        # if (angle < 0.1*self.max_steering_angle) or (angle > -0.1*self.max_steering_angle):
        #     drive_msg.drive.speed = velocity
        # elif (angle < 0.15*self.max_steering_angle) or (angle > -0.15*self.max_steering_angle):
        #     drive_msg.drive.speed = velocity*3/4
        # elif (angle < 0.2*self.max_steering_angle) or (angle > -0.2*self.max_steering_angle):
        #     drive_msg.drive.speed = velocity*2/4
        # else:
        #     drive_msg.drive.speed = velocity*1/4
                       
        self.drive_pub.publish(drive_msg)

        prev_error = error
        integral = integral*0.95 + error
        # rospy.loginfo("Error : %f,\tAngle : %f", error, angle*180/math.pi)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0 

    def lidar_callback(self, data):
        global error
        #TODO: replace with error returned by followLeft
        a = self.getRange(data, 180 - THETA)
        b = self.getRange(data, 180)
        alpha = math.atan((a*math.cos(math.radians(THETA))-b)/(a*math.sin(math.radians(THETA))))
        error = DESIRED_DISTANCE_LEFT - b * math.cos(alpha) - LOOKAHEAD_DISTANCE * math.sin(alpha)
        # rospy.loginfo("a   : %f\t b:\t %f",a, b) 
        # rospy.loginfo("alp : %f\t err:\t %f",alpha*180/math.pi, error) 

        #send error to pid_control
        self.pid_control(error, VELOCITY)
        self.err_pub.publish(error)


def main(args):
    rospy.init_node("WallFollow", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)