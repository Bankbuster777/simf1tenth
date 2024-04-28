// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>
#include <tuple>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;

typedef std::tuple<Node, Node, double> Edge;
typedef struct cell {
    double x,y;
} cell;

class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;

    ros::Publisher drive_pub_;
    ros::Publisher path_pub_;
    ros::Publisher map_pub_;

    // Occupancy grid map
    ros::ServiceClient occupmap_client_;
    ros::Publisher occup_env_lyr_;
    ros::Publisher occup_dyn_lyr_;
    ros::Publisher occup_sta_lyr_;
    double marker_space;

    // tf stuff
    tf::TransformListener listener;
    std::string map_frame, base_frame, scan_frame, err;
    tf::StampedTransform map2scan_tf;
    double pi;

    // TODO: create RRT params
    std::string pose_topic, scan_topic, nav_drive_topic, map_topic, map_service, path_topic, goal_topic;
    std::string occup_env_topic, occup_dyn_topic, occup_sta_topic, occup_map_topic;
    std::string occup_env_topic_arr, occup_dyn_topic_arr, occup_sta_topic_arr;
    double goal_threshold, max_expansion_dist, vehicle_width, large_number;
    int occup_threshold, occupied, vacant;
    geometry_msgs::PoseStamped inferred_pose, origin_pose, transformed_origin;
    geometry_msgs::PointStamped origin_point, scan_point, transformed_point;
    geometry_msgs::PointStamped goal_point;
    nav_msgs::Path rrt_path;

    nav_msgs::OccupancyGrid occup_map;
    double map_resolution, map_posx, map_posy, sampling_width, sampling_size;
    int map_width, map_height;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

    // Helper Functions
    // visualizer
    void pub_occ_env_layer(geometry_msgs::PointStamped env_point);
    void pub_occ_dyn_layer(geometry_msgs::PointStamped dyn_point);
    void pub_occ_sta_layer(geometry_msgs::PointStamped sta_point);
    double dist_point2node(Node &node, double &point_x, double &point_y);
    bool is_occupied(double& point_x, double& point_y);
    bool line_collision(Node &n1, Node &n2);
    bool boundary_collision(Node &n1, Node &n2);
    std::vector<int> point2cell(double& x_point, double& y_point);
    std::vector<double> cell2point(int x_cell, int y_cell);
    void set_vacant(int x_cell, int y_cell);
    void set_occupied(int x_cell, int y_cell);
    bool get_occupied(int x_cell, int y_cell);
    int8_t cell2data(int x_cell, int y_cell);
    void bresenham_line(int x0, int y0, int x1, int y1);
    std::vector<cell> bresenham_low(int x0, int y0, int x1, int y1);
    std::vector<cell> bresenham_high(int x0, int y0, int x1, int y1);

    // utils
    

};

