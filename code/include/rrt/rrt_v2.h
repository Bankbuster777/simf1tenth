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
#include <chrono>

// Struct defining the Node object in the RRT tree.
// More fields could be added to this struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;

typedef struct Edge {
    Node n1, n2;
} Edge;

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
    nav_msgs::OccupancyGrid mini_map;
    geometry_msgs::Pose map_origin, dynamic_map_origin;
    double map_resolution, map_posx, map_posy;
    int map_width, map_height, occup_threshold;
    double vacant, occupied;

    // tf stuff
    tf::TransformListener listener;
    std::string map_frame, scan_frame, err;
    tf::StampedTransform map2scan_tf;
    // TODO: create RRT params
    std::string pose_topic, scan_topic, nav_drive_topic, map_topic, map_service, path_topic, goal_topic;
    double goal_threshold, max_expansion_dist, vehicle_width, large_number, min_delta;
    
    // laser scan
    int scan_size;
    geometry_msgs::PoseStamped scan_origin;

    // visualizer
    ros::Publisher viz_env_layer, viz_dyn_layer, viz_sta_layer;
    ros::Publisher viz_rrt_vert, viz_rrt_edge, viz_rrt_path;
    std::string  env_topic, sta_topic, dyn_topic;
    std::string rrt_vert_topic, rrt_edge_topic, rrt_path_topic;
    int sta_ids, dyn_ids;

    // pf
    int max_iteration;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    double sampling_width, sampling_size;

    // path_reader
    std::string file_path;
    std::vector<geometry_msgs::PointStamped> waypoints;

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

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
    void pub_env_layers(nav_msgs::OccupancyGrid occupancy_map);
    void pub_dyn_layers(geometry_msgs::PointStamped dyn_points);
    void pub_sta_layers(geometry_msgs::PointStamped sta_points);
    void pub_rrt_vertices(std::vector<Node> vertices);
    void pub_rrt_edges(std::vector<Edge> edges);
    void pub_rrt_path(std::vector<Node> tree);

    // utils
    std::vector<int> point2cell(double x_point, double y_point);
    std::vector<double> cell2point(int x_cell, int y_cell);
    std::vector<int> data2cell(int data_index);
    int cell2data(int x_cell, int y_cell);
    std::vector<int> extended_cell2data(int x_cell, int y_cell);
    double dist_point2node(Node& node, double& x_point, double& y_point);
 
    std::vector<cell> bresenham_line(int x0, int y0, int x1, int y1);
    std::vector<cell> bresenham_low(int x0, int y0, int x1, int y1);
    std::vector<cell> bresenham_high(int x0, int y0, int x1, int y1);

    bool is_occupied(double& x_point, double& y_point);
    bool is_cell_occupied(int x_cell, int y_cell);
    void set_vacant(int x_cell, int y_cell);
    void set_occupied(int x_cell, int y_cell);

    bool line_collision(double start_x, double start_y, double end_x, double end_y);
    bool boarder_collision(double start_x, double start_y, double end_x, double end_y);
    bool cage_collision(double start_x, double start_y, double end_x, double end_y);

    std::vector<geometry_msgs::PointStamped> read_csv(std::string file_name);
};

