// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {
    nh_ = ros::NodeHandle("~");
    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file

    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("path_topic", path_topic);
    nh_.getParam("goal_topic", goal_topic);
    nh_.getParam("map_topic", map_topic);
    nh_.getParam("map_service", map_service);
    nh_.getParam("nav_drive_topic", nav_drive_topic);
    // nh_.getParam("occup_env_topic", occup_env_topic);
    // nh_.getParam("occup_dyn_topic", occup_dyn_topic);
    // nh_.getParam("occup_sta_topic", occup_sta_topic);
    nh_.getParam("occup_env_topic_arr", occup_env_topic_arr);
    nh_.getParam("occup_dyn_topic_arr", occup_dyn_topic_arr);
    nh_.getParam("occup_sta_topic_arr", occup_sta_topic_arr);
    nh_.getParam("occup_map_topic", occup_map_topic);
    nh_.getParam("goal_threshold", goal_threshold);
    nh_.getParam("max_expansion_dist", max_expansion_dist);
    nh_.getParam("occup_threshold", occup_threshold);
    nh_.getParam("occupied", occupied);
    nh_.getParam("vacant", vacant);
    nh_.getParam("vehicle_width", vehicle_width);
    nh_.getParam("large_number", large_number);
    nh_.getParam("sampling_width", sampling_width);
    nh_.getParam("sampling_size", sampling_size);
    nh_.getParam("map_frame", map_frame);
    nh_.getParam("base_frame", base_frame);
    nh_.getParam("scan_frame", scan_frame);
    nh_.getParam("marker_space", marker_space);
    nh_.getParam("pi", pi);
    
    transformed_point.header.frame_id = map_frame;
    transformed_origin.header.frame_id = map_frame;
    scan_point.header.frame_id = scan_frame;
    origin_pose.header.frame_id = scan_frame;
    origin_pose.pose.position.x = 0;
    origin_pose.pose.position.y = 0;
    // origin need direction of laser

    x_dist.param(std::uniform_real_distribution<>::param_type((-1)*sampling_width, sampling_width));
    y_dist.param(std::uniform_real_distribution<>::param_type((-1)*sampling_width, sampling_width));

    ROS_INFO("Check sampling %lf", sample()[0]);
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    drive_pub_  = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(nav_drive_topic, 1);
    path_pub_   = nh_.advertise<nav_msgs::Path>(path_topic, 1);
    map_pub_    = nh_.advertise<nav_msgs::OccupancyGrid>(occup_map_topic, 1);
    
    // // This is for Marker
    // occup_env_lyr_ = nh_.advertise<visualization_msgs::Marker>(occup_env_topic, 1);
    // occup_dyn_lyr_ = nh_.advertise<visualization_msgs::Marker>(occup_dyn_topic, 1);
    // occup_sta_lyr_ = nh_.advertise<visualization_msgs::Marker>(occup_sta_topic, 1);

    occup_env_lyr_ = nh_.advertise<visualization_msgs::MarkerArray>(occup_env_topic_arr, 1);
    occup_dyn_lyr_ = nh_.advertise<visualization_msgs::MarkerArray>(occup_dyn_topic_arr, 1);
    occup_sta_lyr_ = nh_.advertise<visualization_msgs::MarkerArray>(occup_sta_topic_arr, 1);

    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_     = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_   = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    goal_sub_   = nh_.subscribe(goal_topic, 10, &RRT::goal_callback, this);

    listener.waitForTransform(map_frame, scan_frame, ros::Time(0), ros::Duration(3.0));
    if (listener.canTransform(map_frame, scan_frame, ros::Time(0), &err)){
        listener.lookupTransform(map_frame, scan_frame, ros::Time(0), map2scan_tf);
        ROS_INFO("Transform : %s 2 %s",scan_frame.c_str(), map_frame.c_str());
    }
    else{
        ROS_INFO_STREAM("Transform Error: " << err);
    }

    // TODO: create a occupancy grid
    occupmap_client_ = nh_.serviceClient<nav_msgs::GetMap>(map_service);
    nav_msgs::GetMap srv;
    if (ros::service::waitForService(map_service, ros::Duration(5))){
        occupmap_client_.call(srv);
        occup_map = srv.response.map; 
        map_resolution = occup_map.info.resolution;
        map_posx = occup_map.info.origin.position.x;
        map_posy = occup_map.info.origin.position.y;
        map_width = occup_map.info.width;
        map_height = occup_map.info.height;
        ROS_INFO("Occupancy Map Loaded : %s", occup_map.header.frame_id.c_str()); 
    }
    else{
        ROS_INFO("Failed to Load Occupancy Map : %s", map_service.c_str());
    }

    geometry_msgs::PoseStamped::ConstPtr sharedPtr;
    sharedPtr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(pose_topic, nh_, ros::Duration(15.0));
    if(sharedPtr != NULL){
        origin_pose.pose.orientation = sharedPtr->pose.orientation;
        ROS_INFO("Orientation Received : %lf , %lf, %lf, %lf", origin_pose.pose.orientation.w, origin_pose.pose.orientation.x, origin_pose.pose.orientation.y, origin_pose.pose.orientation.z);
    }
    else{
        ROS_INFO("Failed to receive Orientation");
    }

    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    // TODO: update your occupancy grid
    double angle;
    int scan_size = scan_msg->ranges.size();
    // For bresenham line
    int x0, y0, x1, y1;
    listener.waitForTransform(map_frame, scan_frame, ros::Time(0), ros::Duration(3.0));
    if (listener.canTransform(map_frame, scan_frame, ros::Time(0), &err)){
        geometry_msgs::PoseStamped::ConstPtr sharedPose;
        sharedPose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(pose_topic, nh_, ros::Duration(15.0));
        if(sharedPose != NULL){
            origin_pose.pose.orientation = sharedPose->pose.orientation;
        }
        listener.transformPose(map_frame, origin_pose, transformed_origin);
        ROS_INFO("Transform : (%lf, %lf) ",transformed_origin.pose.position.x, transformed_origin.pose.position.y);
    }
    else{
        ROS_INFO_STREAM("Transform Error: " << err);
    }

    for (int i =0; i < scan_size;i++){
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        scan_point.point.x = scan_msg->ranges[i] * cos(angle) * (-1);
        scan_point.point.y = scan_msg->ranges[i] * sin(angle);
        listener.transformPoint(scan_frame, scan_point, transformed_point);

        // x0: transformed_origin.point.x
        // y0: transformed_origin.point.y
        // x1: transformed_point.point.x
        // y1: transformed_point.point.y
        x0 = point2cell(transformed_origin.pose.position.x, transformed_origin.pose.position.y)[0];
        y0 = point2cell(transformed_origin.pose.position.x, transformed_origin.pose.position.y)[1];
        x1 = point2cell(transformed_point.point.x, transformed_point.point.y)[0];
        y1 = point2cell(transformed_point.point.x, transformed_point.point.y)[1];
        set_vacant(x0, y0);
        set_occupied(x1, y1);
        ROS_INFO("start bresnham : %d", i);
        bresenham_line(x0,y0,x1,y1);
    }
}

void RRT::bresenham_line(int x0, int y0, int x1, int y1){
    std::vector<cell> unoccupied;
    int dx, dy;
    dx = x1 - x0;
    dy = y1 - y0;
    if (abs(dx) > abs(dy)){
        unoccupied.resize(abs(dx));
        if(x1 > x0){
            unoccupied = bresenham_low(x0,y0,x1,y1);
        }
        else{
            unoccupied = bresenham_low(x1,y1,x0,y0);
        }
    }
    else{
        unoccupied.resize(abs(dy));
        if(x1 > x0){
            unoccupied = bresenham_high(x0,y0,x1,y1);
        }
        else{
            unoccupied = bresenham_high(x1,y1,x0,y0);
        }
    }
    for(int i=0; i < unoccupied.size();i++){
        set_vacant(unoccupied[i].x, unoccupied[i].y);
    }
    
}
std::vector<cell> RRT::bresenham_low(int x0, int y0, int x1, int y1){
    std::vector<cell> unoccupied;
    cell s_grid;
    int dx, dy, D, y, yi;
    dx = x1 - x0;
    dy = y1 - y0;
    y = y0;
    yi = 1;
    unoccupied.resize(abs(dx));
    if (dy < 0){
        yi = -1;
        dy =  (-1) * dy;
    }
    D = 2*dy - dx;
    for (int xi =1; xi<abs(dx); xi++){
        s_grid.x = x0+xi;   s_grid.y= y;
        unoccupied.push_back(s_grid);
        if (D > 0){
            y = y + yi;
            D = D - 2*dx + 2*dy; // was -2*dx
        }
        else{
            D = D + 2*dy;
        }
    }
    return unoccupied;
}
std::vector<cell> RRT::bresenham_high(int x0, int y0, int x1, int y1){
    std::vector<cell> unoccupied;
    cell s_grid;
    int dx, dy, D, x, xi;
    dx = x1 - x0;
    dy = y1 - y0;
    x = x0;
    xi = 1;

    unoccupied.resize(abs(dy));
    if (dx < 0){
        xi = -1;
        dx =  (-1) * dx;
    }
    D = 2*dy - dx;
    for (int yi =1; yi<abs(dy); yi++){
        s_grid.x = x;   s_grid.y= y0+yi;
        unoccupied.push_back(s_grid);
        if (D > 0){
            x = x + xi;
            D = D - 2*dx + 2*dy; // was -2*dx
        }
        else{
            D = D + 2*dy;
        }
    }
    return unoccupied;
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    Node node_init;
    node_init.x = pose_msg->pose.position.x;
    node_init.y = pose_msg->pose.position.y;
    node_init.is_root = true;
    node_init.parent = 0;

    geometry_msgs::PoseStamped::ConstPtr sharedGoal;

    std::vector<double> random_point(2);
    int nearest_idx;
    Node node_new;
    std::vector<Node> path;
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;
    tree.clear();
    tree.push_back(node_init);

    ROS_INFO("pf init done");
    // TODO: fill in the RRT main loop
     for(int i =0; i < sampling_size; i++){
        do{
            random_point[0] = pose_msg->pose.position.x + sample()[0];
            random_point[1] = pose_msg->pose.position.y + sample()[1];
            ROS_INFO("sampled: %lf, %lf", random_point[0], random_point[1]);
        }while(!is_occupied(random_point[0], random_point[1]));
        nearest_idx = nearest(tree, random_point);
        node_new = steer(tree[nearest_idx], random_point);
        ROS_INFO("steered: (%lf, %lf)", node_new.x, node_new.y);
        if(!check_collision(tree[nearest_idx], node_new)){
            tree.push_back(node_new);
            node_new.parent = nearest_idx; // Edge
            ROS_INFO("Wait for goal topic");
            // sharedGoal = ros::topic::waitForMessage(goal_topic,nh_);
            // if(sharedGoal != NULL){
            //
            // }
            // if(is_goal(node_new, goal_point.point.x, goal_point.point.y)){
            //     ROS_INFO("Path found");
            //     path = find_path(tree, node_new);
            //     break;
            // }
        }
     }

    // path found as Path message

}

void RRT::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg){
    goal_point.header.frame_id = goal_msg->header.frame_id;
    goal_point.point.x = goal_msg->pose.position.x;
    goal_point.point.y = goal_msg->pose.position.y;
    ROS_INFO("Goal_received, (%lf, %lf)", goal_point.point.x, goal_point.point.y);

}


std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point(2);
    sampled_point.clear();
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    
    sampled_point[0] = x_dist(gen);
    sampled_point[1] = y_dist(gen);
    
    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method
    double dist = large_number;
    double temp;
    for (int i =0; i< tree.size(); i++){
        temp = dist_point2node(tree[i], sampled_point[0], sampled_point[1]);
        if(temp < dist){
            dist = temp;
            nearest_node = i;
        }
    }
    
    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method
    double slope = (sampled_point[1] - nearest_node.y)/(sampled_point[0] - nearest_node.x);
    if (dist_point2node(nearest_node, sampled_point[0], sampled_point[1]) < max_expansion_dist){
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else if(nearest_node.x < sampled_point[0]){
        new_node.x = nearest_node.x + sqrt(max_expansion_dist/(1+slope*slope));
        new_node.y = slope*(new_node.x - nearest_node.x) + nearest_node.y;
    }
    else{
        new_node.x = nearest_node.x - sqrt(max_expansion_dist/(1+slope*slope));
        new_node.y = slope*(new_node.x - nearest_node.x) + nearest_node.y;   
    }

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method
    if(line_collision(nearest_node, new_node)){
        collision = boundary_collision(nearest_node, new_node);
    }
    else{
        collision = true;
    }

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    double dist = goal_threshold + 1;   // cannot recognize goal_threshold when use param
    dist = dist_point2node(latest_added_node, goal_x, goal_y);
    if (dist < goal_threshold){    
        close_enough = true;
    }

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    // TODO: fill in this method
    found_path.clear();
    if(!(latest_added_node.is_root)){
        found_path = find_path(tree, tree[latest_added_node.parent]);
        found_path.push_back(latest_added_node);
    }
    else{
        found_path.insert(found_path.begin(), tree[0]);
    }

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method
    cost = dist_point2node(n1,n2.x,n2.y);

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}

void RRT::pub_occ_env_layer(geometry_msgs::PointStamped env_point){
    // visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_arr;
    marker_arr.markers.resize(9);
    int x_idx, y_idx;
    for(int i=0; i < 9; i++){
        x_idx = i % 3 - 1;
        y_idx = (int) i/3 - 1;
        marker_arr.markers[i].header.frame_id = map_frame;
        marker_arr.markers[i].header.stamp = ros::Time();
        marker_arr.markers[i].ns = "environment";
        marker_arr.markers[i].id = int(ros::Time::now().toNSec()) * 100 + i;
        marker_arr.markers[i].lifetime = ros::Duration(15.0);
        marker_arr.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
        marker_arr.markers[i].pose.position.x = env_point.point.x + x_idx * marker_space;
        marker_arr.markers[i].pose.position.y = env_point.point.y + y_idx * marker_space;
        marker_arr.markers[i].pose.position.z = 0;
        marker_arr.markers[i].pose.orientation.x = 0.0;
        marker_arr.markers[i].pose.orientation.y = 0.0;
        marker_arr.markers[i].pose.orientation.z = 0.0;
        marker_arr.markers[i].pose.orientation.w = 1.0;
        marker_arr.markers[i].scale.x = map_resolution;
        marker_arr.markers[i].scale.y = map_resolution;
        marker_arr.markers[i].scale.z = map_resolution;
        marker_arr.markers[i].color.a = 1.0;
        marker_arr.markers[i].color.r = 0.0;
        marker_arr.markers[i].color.g = 0.0;
        marker_arr.markers[i].color.b = 0.0;
    }
    occup_env_lyr_.publish(marker_arr); 
}

void RRT::pub_occ_dyn_layer(geometry_msgs::PointStamped dyn_point){
    visualization_msgs::MarkerArray marker_arr;
    marker_arr.markers.resize(25);
    int x_idx, y_idx;
    for(int i=0; i < 25; i++){
        x_idx = i % 5 - 2;
        y_idx = (int) i/5 - 2;
        marker_arr.markers[i].header.frame_id = map_frame;
        marker_arr.markers[i].header.stamp = ros::Time();
        marker_arr.markers[i].ns = "dynamic";
        marker_arr.markers[i].id = (int) ros::Time::now().toNSec() * 100 + i;
        marker_arr.markers[i].lifetime = ros::Duration(15.0);
        marker_arr.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
        marker_arr.markers[i].pose.position.x = dyn_point.point.x + x_idx * marker_space;
        marker_arr.markers[i].pose.position.y = dyn_point.point.y + y_idx * marker_space;
        marker_arr.markers[i].pose.position.z = 0;
        marker_arr.markers[i].pose.orientation.x = 0.0;
        marker_arr.markers[i].pose.orientation.y = 0.0;
        marker_arr.markers[i].pose.orientation.z = 0.0;
        marker_arr.markers[i].pose.orientation.w = 1.0;
        marker_arr.markers[i].scale.x = map_resolution;
        marker_arr.markers[i].scale.y = map_resolution;
        marker_arr.markers[i].scale.z = map_resolution;
        marker_arr.markers[i].color.a = 1.0;
        marker_arr.markers[i].color.r = 0.0;
        marker_arr.markers[i].color.g = 0.0;
        marker_arr.markers[i].color.b = 1.0;
    }
    occup_dyn_lyr_.publish(marker_arr); 
    ROS_INFO("dyn layer about to publish");
}

void RRT::pub_occ_sta_layer(geometry_msgs::PointStamped sta_point){
    visualization_msgs::MarkerArray marker_arr;
    marker_arr.markers.resize(16);
    int x_idx, y_idx;
    for(int i=0; i < 16; i++){
        x_idx = i % 4 - 1.5;
        y_idx = (int) i/4 - 1.5;
        marker_arr.markers[i].header.frame_id = map_frame;
        marker_arr.markers[i].header.stamp = ros::Time();
        marker_arr.markers[i].ns = "static";
        marker_arr.markers[i].id = int(ros::Time::now().toNSec()) * 100 + i;
        marker_arr.markers[i].lifetime = ros::Duration(15.0);
        marker_arr.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_arr.markers[i].action = visualization_msgs::Marker::ADD;
        marker_arr.markers[i].pose.position.x = sta_point.point.x + x_idx * marker_space;
        marker_arr.markers[i].pose.position.y = sta_point.point.y + y_idx * marker_space;
        marker_arr.markers[i].pose.position.z = 0;
        marker_arr.markers[i].pose.orientation.x = 0.0;
        marker_arr.markers[i].pose.orientation.y = 0.0;
        marker_arr.markers[i].pose.orientation.z = 0.0;
        marker_arr.markers[i].pose.orientation.w = 1.0;
        marker_arr.markers[i].scale.x = map_resolution;
        marker_arr.markers[i].scale.y = map_resolution;
        marker_arr.markers[i].scale.z = map_resolution;
        marker_arr.markers[i].color.a = 1.0;
        marker_arr.markers[i].color.r = 1.0;
        marker_arr.markers[i].color.g = 0.0;
        marker_arr.markers[i].color.b = 0.0;
    }
    occup_sta_lyr_.publish(marker_arr); 
}
double RRT::dist_point2node(Node &node, double &point_x, double &point_y){
    double dist = 0;
    dist = (node.x - point_x)*(node.x - point_x)-(node.y - point_y)*(node.y - point_y);
    dist = sqrt(dist);
    return dist;
}

bool RRT::is_occupied(double& point_x, double& point_y){
    bool occupied = false;
    int8_t x1 = point2cell(point_x, point_y)[0];
    int8_t y1 = point2cell(point_x, point_y)[1];
    if(occup_map.data[y1*map_width + x1] > occup_threshold){
        occupied = true;
    }
    return occupied;
}

bool RRT::line_collision(Node &n1, Node &n2){
    bool check_occupancy = false;
    std::vector<int8_t> point;
    int8_t x1 = point2cell(n1.x, n1.y)[0];
    int8_t y1 = point2cell(n1.x, n1.y)[1];
    // global coordinate to occupancy grid cell

    double slope = (n2.y - n1.y)/(n2.x - n1.x);
    int8_t x_cell = (int) abs(n2.x-n1.x)/map_resolution;
    for(int i=1; i < x_cell; i++){
        if (n1.x < n2.x){
            point[0] = x1 + i;
        }
        else{
            point[0] = x1 - i;
        }
        point[1] = (int) (slope*(point[0] - x1)) + y1;
        if(occup_map.data[point[1]*map_width + point[0]] > occup_threshold){
            check_occupancy = true;
            break;
        }
    }
    return check_occupancy;
}

bool RRT::boundary_collision(Node &n1, Node &n2){
    // Cannot detect small object
    bool check_occupancy = false;
    bool big_x1 = n1.x > n2.x;
    bool big_y1 = n1.y > n2.y;
    Node corner_n11, corner_n12, corner_n21, corner_n22;
    double half_wid = vehicle_width/2;
    if(big_x1 == big_y1){
        corner_n11.x = n1.x - half_wid;  corner_n11.y = n1.y + half_wid;
        corner_n21.x = n2.x - half_wid;  corner_n21.y = n2.y + half_wid;
        corner_n12.x = n1.x + half_wid;  corner_n12.y = n1.y - half_wid;
        corner_n22.x = n2.x + half_wid;  corner_n22.y = n2.y - half_wid;
    }
    else{
        corner_n11.x = n1.x + half_wid;  corner_n11.y = n1.y + half_wid;
        corner_n21.x = n2.x + half_wid;  corner_n21.y = n2.y + half_wid;
        corner_n12.x = n1.x - half_wid;  corner_n12.y = n1.y - half_wid;
        corner_n22.x = n2.x - half_wid;  corner_n22.y = n2.y - half_wid;
    }
    
    check_occupancy = line_collision(corner_n11, corner_n21)
                   && line_collision(corner_n12, corner_n22);
    return check_occupancy;
}
void RRT::set_vacant(int x_cell, int y_cell){
    occup_map.data[y_cell*map_width + x_cell] = vacant;

    // if(occup_map.data[y_cell*map_width + x_cell] > occup_threhold){
    // }
}
void RRT::set_occupied(int x_cell, int y_cell){
    geometry_msgs::PointStamped viz_point;
    viz_point.header.frame_id = map_frame;
    viz_point.point.x = cell2point(x_cell, y_cell)[0];
    viz_point.point.y = cell2point(x_cell, y_cell)[1];

    if(occup_map.data[y_cell*map_width + x_cell] < occup_threshold){
        pub_occ_dyn_layer(viz_point);
    }
    else{
        pub_occ_sta_layer(viz_point);
    }
    occup_map.data[y_cell*map_width + x_cell] = occupied;

}

std::vector<int> RRT::point2cell(double& x_point, double& y_point){
    std::vector<int> cell_(2);
    cell_[0] = (x_point - map_posx)/map_resolution;
    cell_[1] = (y_point - map_posy)/map_resolution;

    return cell_;
}

std::vector<double> RRT::cell2point(int x_cell, int y_cell){
    std::vector<double> point_(2);
    point_[0] = x_cell * map_resolution + map_posx;
    point_[1] = y_cell * map_resolution + map_posy;

    return point_;
}
bool RRT::get_occupied(int x_cell, int y_cell){
    bool occupancy = false;
    return occupancy;
}