#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        odom_topic = '/odom'

        self.current_velocity = 0
        self.current_steering_angle = 0

        self.theta_low_limit = -45 # degree
        self.theta_high_limit = 225 # degree
        self.max_radius = 3 # meters
        self.deg2rad = math.pi/180
        self.scan_range = 360.0 # degree
        self.scan_beams = 1080
        self.theta_incr = self.scan_range/self.scan_beams
        self.width = 0.3 # meters
        self.max_steering_angle = 0.4189

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window(on 3 decimal)
            2.Rejecting high values (eg. > 3m)
        """
        # Scan Starts from the backside / Counter-clockwise
        proc_ranges = list(ranges)
        for i in range(len(proc_ranges)):
            theta = self.theta_incr *i - 90
            if (theta > self.theta_high_limit) or (theta < self.theta_low_limit):
                proc_ranges[i] = 0
            elif (proc_ranges[i] > self.max_radius):
                proc_ranges[i] = self.max_radius
            else:
                proc_ranges[i] = 0.001 * int(1000*proc_ranges[i])
        
        # print("Processed: ", proc_ranges) # masked well
        return proc_ranges

    def check_disparity(self, preprocessed_ranges):
        """ Check disparity point and Separate the zones
            1. Check disparity point
            2. Make a list for disparity indexes 
            3. List the median value of disparity zone
        """
        # Check disparity points
        disparity_idxes = list()
        disparity_thres = self.theta_incr * self.deg2rad * self.max_radius * 3 # 5 times of margin
        for i in range(len(preprocessed_ranges)):
            if ((i > 0) & (((preprocessed_ranges[i] - preprocessed_ranges[i-1]) > disparity_thres) 
                or ((preprocessed_ranges[i-1] - preprocessed_ranges[i]) > disparity_thres))):
                disparity_idxes.append(i)        

        # List the target point
        target_idxes = list()
        idx_threshold = self.width / self.max_radius / (self.theta_incr * self.deg2rad) # 18
        for i in range(len(disparity_idxes)):
            if ( i > 0 ) :  # & ((disparity_idxes[i] - disparity_idxes[i-1]) > idx_threshold)
                idx = (disparity_idxes[i] + disparity_idxes[i-1]) // 2
                target_idxes.append(idx)

        # Check the distance
        target_dist = list()
        for i in range(len(target_idxes)):
            dist = preprocessed_ranges[target_idxes[i]] 
            target_dist.append(dist)

        # verify target_idxes & target_dist
        # print("dispar_indexes: ", *disparity_idxes)
        # print("target_indexes: ", *target_idxes)
        # print("target_distanc: ", *target_dist)
        
        # Return max median index
        max_idx = 0
        if len(target_dist):
            max_dist = max(target_dist)             # rospy.loginfo("max dist: %d", max_dist)
            trg_idx = target_dist.index(max_dist)   # rospy.loginfo("idx dist: %d", trg_idx)
            max_idx = target_idxes[trg_idx]         # rospy.loginfo("max indx: %d", max_idx)

        return max_idx

    def idx2str_angle(self, scan_index):
        scan_degree = 180.0 - scan_index * self.theta_incr
        scan_rad = float(self.deg2rad * scan_degree)
        # rospy.loginfo("scan_idx: %0.2f", scan_index)       # laser scan index
        # rospy.loginfo("scan_dgr: %0.2f", scan_degree)      # index to car degree (left is minus)
        # rospy.loginfo("scan_rad: %0.2f", float(scan_rad))   # target angle same
        # rospy.loginfo("type_rad: %s", type(scan_rad))
    
        return scan_rad

    def odom_callback(self, data):
        self.current_velocity = data.twist.twist.linear.x
        self.current_steering_angle = data.twist.twist.angular.z

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        target_index = self.check_disparity(proc_ranges)

        # rospy.loginfo(proc_ranges)
        # rospy.loginfo(np.array(disparity_index)*self.theta_incr)
        # rospy.loginfo(np.array(target_index)*self.theta_incr)
        # rospy.loginfo("target is %d", target_index*self.theta_incr)
         
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 
        target_angle = self.idx2str_angle(target_index)

        rospy.loginfo("target_angle : %0.2f",target_angle)
        
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = - target_angle * self.max_steering_angle # why minus sign
        drive_msg.drive.speed = 1 # for now 

        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)