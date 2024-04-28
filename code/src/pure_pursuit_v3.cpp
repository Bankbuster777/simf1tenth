#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream> 
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
// TODO: include ROS msg type headers and libraries you need
    // Subscriber
#include <nav_msgs/Odometry.h>
    // Publisher
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

class PurePursuit {
private:
    ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers
        // Subscriber
    ros::Subscriber particle_sub;

        // Puslisher
    ros::Publisher pure_drive_pub;
    ros::Publisher curr_marker_pub;
    ros::Publisher wplist_marker_pub;

    // Drive parameters
    double wheelbase, max_steering_angle, drive_speed;
    double dist_thres = 1.0;

    // Waypoints
    int wp_iter =0;
    std::string file_path = "/home/dawgs/dawgs_car/sim_f1tenth_ws/src/labs_f1tenth/lab6/waypoints/";
    geometry_msgs::PointStamped curr_wp_pose;
    std::vector<geometry_msgs::PointStamped> vec_waypoints;

    // Transform
    tf::TransformListener listener;
    tf::StampedTransform map_to_base_link_tf;
    std::string err;

public:
    PurePursuit() {
        n = ros::NodeHandle("~");

        // get topic names
        std::string pf_pose_topic, curr_marker_topic, wp_list_marker_topic, pure_drive_topic, nav_drive_topic;
        n.getParam("pf_pose_topic", pf_pose_topic);
        n.getParam("curr_marker_topic", curr_marker_topic);
        n.getParam("wp_list_marker_topic", wp_list_marker_topic);
        n.getParam("pure_drive_topic", pure_drive_topic);
        n.getParam("nav_drive_topic", nav_drive_topic);

        // get car parameters
        n.getParam("wheelbase", wheelbase);
        n.getParam("max_steering_angle", max_steering_angle);
        n.param("drive_speed", drive_speed, 1.0);

        std::string map_frame, base_frame, scan_frame;
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);
        n.getParam("scan_frame", scan_frame);

        std::string wp_csv ="waypoints_240204";
        read_waypoints(wp_csv, vec_waypoints);
        // comments out for segmentation failure
        curr_wp_pose.header.frame_id = "map";
        curr_wp_pose.point.x = vec_waypoints[wp_iter].point.x;
        curr_wp_pose.point.y = vec_waypoints[wp_iter].point.y;

        listener.waitForTransform(base_frame, map_frame, ros::Time(0), ros::Duration(3.0));
        if (listener.canTransform(base_frame, map_frame, ros::Time(0), &err)){
            listener.lookupTransform(base_frame, map_frame, ros::Time(0), map_to_base_link_tf);
            ROS_INFO("Transform done");
        }
        else{
            ROS_INFO_STREAM("Transform Error: " << err);
        }

        // TODO: create ROS subscribers and publishers
        // SubScriber
        particle_sub = n.subscribe(pf_pose_topic, 1, &PurePursuit::pose_callback, this);

        // Publisher
        pure_drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(nav_drive_topic, 1);
        curr_marker_pub = n.advertise<visualization_msgs::Marker>(curr_marker_topic, 1);
        wplist_marker_pub = n.advertise<visualization_msgs::MarkerArray>(wp_list_marker_topic, 1, true);

        ROS_INFO("Purepursuit Initialized");
    }

    void read_waypoints(std::string csv_file, std::vector<geometry_msgs::PointStamped>& v_waypoints){
        std::fstream csv_wp;
        std::string x_str, y_str, file_name;
        geometry_msgs::PointStamped single_waypoint;
        file_name = file_path + csv_file + ".csv";
        
        csv_wp.open(file_name, std::ios::in);
        if(!csv_wp.is_open()){
            ROS_INFO("Cannot open the file : %s", csv_file);
        }
        else{
            while(!csv_wp.eof()){                
                std::getline(csv_wp, x_str, ',');
                std::getline(csv_wp, y_str, '\n');
                if(!x_str.empty()){
                    single_waypoint.header.frame_id = "map";
                    single_waypoint.point.x = std::stof(x_str);
                    single_waypoint.point.y = std::stof(y_str);
                    v_waypoints.push_back(single_waypoint);
                }
            }
            ROS_INFO("Read csv done");
            ROS_INFO("First : %f %f", v_waypoints.front().point.x, v_waypoints.front().point.y);
            ROS_INFO("Number of wp: %d", v_waypoints.size());
            csv_wp.close();
        }
    }

    void visualize_marker_array(std::vector<geometry_msgs::PointStamped>& v_waypoints){
        visualization_msgs::MarkerArray array_marker;
        int array_size = v_waypoints.size();
        array_marker.markers.resize(array_size);
        for(int i=0; i < array_size; i++){
            array_marker.markers[i].header.frame_id = v_waypoints[i].header.frame_id;
            array_marker.markers[i].header.stamp = v_waypoints[i].header.stamp;
            array_marker.markers[i].ns = "vis";
            array_marker.markers[i].id = i;
            array_marker.markers[i].lifetime = ros::Duration(2.0);
            array_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
            array_marker.markers[i].action = visualization_msgs::Marker::ADD;
            array_marker.markers[i].pose.position.x = v_waypoints[i].point.x;
            array_marker.markers[i].pose.position.y = v_waypoints[i].point.y;
            array_marker.markers[i].pose.position.z = 0;
            array_marker.markers[i].pose.orientation.x = 0.0;
            array_marker.markers[i].pose.orientation.y = 0.0;
            array_marker.markers[i].pose.orientation.z = 0.0;
            array_marker.markers[i].pose.orientation.w = 1.0;
            array_marker.markers[i].scale.x = 0.2;
            array_marker.markers[i].scale.y = 0.2;
            array_marker.markers[i].scale.z = 0.2;
            array_marker.markers[i].color.a = 0.6;
            array_marker.markers[i].color.r = 0.0;
            array_marker.markers[i].color.g = 0.0;
            array_marker.markers[i].color.b = 1.0 - 0.4 * i/array_size;
        }

        wplist_marker_pub.publish(array_marker);
    }

    void visualize_marker(geometry_msgs::PointStamped waypoint_){
        visualization_msgs::Marker marker;
        marker.header.frame_id = waypoint_.header.frame_id;
        marker.header.stamp = waypoint_.header.stamp;
        marker.ns = "mak";
        marker.id = 0;
        marker.lifetime = ros::Duration(3.0);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = waypoint_.point.x;
        marker.pose.position.y = waypoint_.point.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.1;
        marker.color.a = 0.9;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        curr_marker_pub.publish(marker);
    }

    void pose_callback(const nav_msgs::Odometry &pose_msg) {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        visualize_marker_array(vec_waypoints);
        geometry_msgs::PointStamped transformed_wp_pose;
        transformed_wp_pose.header.frame_id = "base_link";
        float pos_x, pos_y, pos_theta;
        pos_x     = pose_msg.pose.pose.position.x;
        pos_y     = pose_msg.pose.pose.position.y;
        pos_theta = pose_msg.pose.pose.orientation.z;
        
        float lookup_dist, lateral_dist, target_steering_angle, turning_radius, curvature, dist, dist_err_sq; 

        // TODO: transform goal point to vehicle frame of reference
        dist_err_sq = (pos_x - curr_wp_pose.point.x) * (pos_x - curr_wp_pose.point.x) +
                      (pos_y - curr_wp_pose.point.y) * (pos_y - curr_wp_pose.point.y);
        dist = sqrt(dist_err_sq);
        
        if(dist_err_sq < dist_thres){
            wp_iter += 1;
            curr_wp_pose.point.x = vec_waypoints[wp_iter].point.x;
            curr_wp_pose.point.y = vec_waypoints[wp_iter].point.y;
            ROS_INFO("iter: %d", wp_iter);
        }
        listener.transformPoint("base_link", curr_wp_pose, transformed_wp_pose);
        // ROS_INFO("transformed : %f , %f", transformed_wp_pose.point.x, transformed_wp_pose.point.y);
        visualize_marker(curr_wp_pose);

        // TODO: calculate curvature/steering angle
        lookup_dist = transformed_wp_pose.point.x * transformed_wp_pose.point.x +
                      transformed_wp_pose.point.y * transformed_wp_pose.point.y;
        lateral_dist = transformed_wp_pose.point.y;
        curvature = (2 * lateral_dist)/lookup_dist;
        target_steering_angle = atan2(curvature, 1/wheelbase);

        // don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        if(target_steering_angle > max_steering_angle){
            ROS_INFO("too much + steering %f", target_steering_angle);
            target_steering_angle = max_steering_angle;
        }
        else if(target_steering_angle < - max_steering_angle){
            ROS_INFO("too much - steering %f", target_steering_angle);
            target_steering_angle = -1 * max_steering_angle;
        }

        // TODO: publish drive message
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = target_steering_angle;
        drive_msg.speed = drive_speed;
        drive_st_msg.drive = drive_msg;
        pure_drive_pub.publish(drive_st_msg);
    }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit");
    PurePursuit pp;
    ros::spin();
    return 0;
}