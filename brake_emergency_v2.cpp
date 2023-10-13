#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

// for printing
#include <iostream>

#include "f1tenth_simulator/precompute.hpp"
using namespace racecar_simulator;


#include <algorithm>


class BrakeEmergency {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Listen for odom, laser messages
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    // Publish brake_drive, brake_bool data
    ros::Publisher brake_drive_pub;
    ros::Publisher brake_bool_pub;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;
    double ttc_threshold;
    double velocity, projected_velocity;

public:
    BrakeEmergency() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string brake_drive_topic, brake_bool_topic, scan_topic, odom_topic;
        n.getParam("brake_drive_topic", brake_drive_topic);
        n.getParam("brake_bool_topic", brake_bool_topic); // brake toggle
        n.getParam("scan_topic", scan_topic);
        n.getParam("odom_topic", odom_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Make a publisher for drive messages
        brake_drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(brake_drive_topic, 10);
        brake_bool_pub = n.advertise<std_msgs::Bool>(brake_bool_topic, 10);

        // Start a subscriber to listen to laser, odometry messages
        laser_sub = n.subscribe(scan_topic, 1, &BrakeEmergency::laser_callback, this);
        odom_sub = n.subscribe(odom_topic, 1, &BrakeEmergency::odom_callback, this);

	    // Get car distance
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width, 
        	scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);
        
    }

    void odom_callback(const nav_msgs::Odometry & msg) {
        // Keep track of state to be used elsewhere
        velocity = msg.twist.twist.linear.x;
    }

    void laser_callback(const sensor_msgs::LaserScan & msg) {
        ackermann_msgs::AckermannDriveStamped brake_drive_st_msg;
        ackermann_msgs::AckermannDrive brake_drive_msg;
        std_msgs::Bool brake_bool;

        double  ttc_n, ttc;

        for (size_t i = 0 ; i < msg.ranges.size(); i++){
            ttc_n = msg.ranges[i] - car_distances[i];
            projected_velocity = velocity * cosines[i];
            if (velocity != 0){
                ttc = ttc_n / projected_velocity;
                if ((ttc < ttc_threshold) && (ttc > 0)){
                    if (!brake_bool.data){
                        brake_bool.data = true;
                        brake_bool_pub.publish(brake_bool);

                        brake_drive_msg.speed = 0.0;
                        brake_drive_msg.steering_angle = 0.0;
                        brake_drive_st_msg.drive = brake_drive_msg;
                        brake_drive_pub.publish(brake_drive_st_msg);
                        ROS_INFO("Brake Engaged");
                        ROS_INFO("ttc  : %f", ttc);
                        brake_bool.data = false;
                    }
                }
            }

        }
    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "brake_emergency");
    BrakeEmergency be;
    ros::spin();
    return 0;
}
