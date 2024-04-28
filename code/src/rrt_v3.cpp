// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt_v2.h"

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
    nh_.getParam("nav_drive_topic", nav_drive_topic);
    nh_.getParam("path_topic", path_topic);
    nh_.getParam("goal_topic", goal_topic);
    nh_.getParam("map_topic", map_topic);

    // frames
    nh_.getParam("map_frame", map_frame);
    nh_.getParam("scan_frame", scan_frame);

    // service
    nh_.getParam("map_service", map_service);

    // Paramters
    nh_.getParam("large_number", large_number);
    nh_.getParam("min_delta", min_delta);
    nh_.getParam("goal_threshold", goal_threshold);
    nh_.getParam("occup_threshold", occup_threshold);
    
    // map
    nh_.getParam("vacant", vacant);
    nh_.getParam("occupied", occupied);
    nh_.getParam("sampling_width", sampling_width);
    nh_.getParam("map_resolution", map_resolution);
    nh_.getParam("occup_threshold", occup_threshold);

    // visualizer
    nh_.getParam("env_topic", env_topic);
    nh_.getParam("dyn_topic", dyn_topic);
    nh_.getParam("sta_topic", sta_topic);
    nh_.getParam("rrt_vert_topic", rrt_vert_topic);
    nh_.getParam("rrt_edge_topic", rrt_edge_topic);
    nh_.getParam("rrt_path_topic", rrt_path_topic);
    nh_.getParam("sta_ids", sta_ids);
    nh_.getParam("dyn_ids", dyn_ids);
    sta_ids = 0;
    dyn_ids = 0;

    // pf 
    nh_.getParam("max_iteration", max_iteration);

    // path reader
    nh_.getParam("file_path", file_path);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    viz_env_layer = nh_.advertise<visualization_msgs::Marker>(env_topic, 1);
    viz_dyn_layer = nh_.advertise<visualization_msgs::Marker>(dyn_topic, 1);
    viz_sta_layer = nh_.advertise<visualization_msgs::Marker>(sta_topic, 1);
    viz_rrt_vert = nh_.advertise<visualization_msgs::Marker>(rrt_vert_topic, 1);
    viz_rrt_edge = nh_.advertise<visualization_msgs::Marker>(rrt_edge_topic, 1);
    viz_rrt_path = nh_.advertise<visualization_msgs::Marker>(rrt_path_topic, 1);

    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(nav_drive_topic, 1);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);

    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    // sample
    x_dist.param(std::uniform_real_distribution<>::param_type((-1)*sampling_width, sampling_width));
    y_dist.param(std::uniform_real_distribution<>::param_type((-1)*sampling_width, sampling_width));

    // scan
    scan_origin.header.frame_id = scan_frame;
    scan_origin.pose.position.x = 0;
    scan_origin.pose.position.y = 0;

    // TODO: create a occupancy grid
    mini_map.header.frame_id = scan_frame;
    mini_map.info.origin.position.x = -1 * sampling_width;
    mini_map.info.origin.position.y = -1 * sampling_width;
    mini_map.info.origin.position.z = 0;
    mini_map.info.origin.orientation.w = 1;
    mini_map.info.origin.orientation.x = 0;
    mini_map.info.origin.orientation.y = 0;
    mini_map.info.origin.orientation.z = 0;
    mini_map.info.resolution = map_resolution;
    mini_map.info.width = (int) 2 * sampling_width/map_resolution;
    mini_map.info.height = (int) 2 * sampling_width/map_resolution;
    map_posx = mini_map.info.origin.position.x;
    map_posy = mini_map.info.origin.position.y;
    map_width = mini_map.info.width;
    map_height = mini_map.info.height;
    mini_map.data.resize((int) map_width * map_height);
    for (int i = 0; i < mini_map.data.size(); i++){
        mini_map.data[i] = occup_threshold;
    }

    waypoints = read_csv("");


    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid
    std::vector<geometry_msgs::PointStamped> scanned_point, transformed_point;
    double angle;
    std::vector<int> scanned_cell(2), origin_cell(2);
    std::vector<cell> unoccupied;
    scan_size = scan_msg->ranges.size();
    scanned_point.resize(scan_size);
    transformed_point.resize(scan_size);
    origin_cell = point2cell(scan_origin.pose.position.x, scan_origin.pose.position.y);

    for(int i=0; i<scan_size; i++){
        angle = scan_msg->angle_increment * i;
        scanned_point[i].header.frame_id = scan_frame;
        scanned_point[i].point.x = scan_msg->ranges[i] * cos(angle) * (-1);
        scanned_point[i].point.y = scan_msg->ranges[i] * sin(angle) * (-1);
        
        scanned_cell = point2cell(scanned_point[i].point.x, scanned_point[i].point.y);
        unoccupied = bresenham_line(origin_cell[0], origin_cell[1], scanned_cell[0], scanned_cell[1]);

        for(int i=1; i < (unoccupied.size()-1) ;i++){
            set_vacant(unoccupied[i].x, unoccupied[i].y);
        }
        set_occupied(scanned_cell[0], scanned_cell[1]);
        // transformed_point[i].header.frame_id = map_frame;
    }

    mini_map.header.stamp = scan_msg->header.stamp;
    map_pub_.publish(mini_map);
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;
    std::vector<Edge> edges;
    std::vector<double> random_point(2);
    geometry_msgs::PointStamped pf_point, transformed_point;
    Edge new_edge;
    Node init_node, nearest_node, new_node;
    pf_point.header.frame_id = map_frame;
    pf_point.point.x = pose_msg->pose.position.x;
    pf_point.point.y = pose_msg->pose.position.y;
    transformed_point.header.frame_id = scan_frame;

    listener.waitForTransform(scan_frame, map_frame, ros::Time(0), ros::Duration(3.0));
    if (listener.canTransform(scan_frame, map_frame, ros::Time(0), &err)){
        listener.transformPoint(scan_frame, pf_point, transformed_point);
        init_node.x = transformed_point.point.x;
        init_node.y = transformed_point.point.y;
        tree.push_back(init_node);
    }
    else{
        ROS_INFO_STREAM("PF Transform Error: " << err);
    }

    // TODO: fill in the RRT main loop
    int i =0;
    do{
        random_point = sample();
        nearest_node = tree[nearest(tree, random_point)];
        new_node = steer(nearest_node, random_point);

        if(!check_collision(nearest_node, new_node)){
            tree.push_back(new_node);
            ROS_INFO("free : %lf, %lf, %d", new_node.x, new_node.y, i);
            new_edge.n1 = nearest_node;
            new_edge.n2 = new_node;
            edges.push_back(new_edge);
            pub_rrt_vertices(tree);
            // pub_rrt_edges(edges);
        }
        i++;
    }while(i<max_iteration);    // is_goal(new_node, ) || 


    // path found as Path message

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double>  sampled_point(2);
    geometry_msgs::PointStamped sampled_, transformed_;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    sampled_point[0] = 0;
    sampled_point[1] = 0;
    sampled_.header.frame_id = scan_frame;
    gen.seed(static_cast<long unsigned int>(time(0)));

    do{
        sampled_.point.x = x_dist(gen);
        sampled_.point.y = y_dist(gen);
    }while(is_occupied(sampled_.point.x, sampled_.point.y));
    // transformed_.header.frame_id = map_frame;
    // if(listener.canTransform(map_frame, scan_frame, ros::Time(0)), &err){
    //     listener.transformPoint(map_frame, sampled_, transformed_);
    // }
    // else{
    //     ROS_INFO_STREAM("Sampling Error: " << err);
    // }
    // sampled_point[0] = transformed_.point.x;
    // sampled_point[1] = transformed_.point.y;
    
    sampled_point[0] = sampled_.point.x;
    sampled_point[1] = sampled_.point.y;

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
    double prev_dist;
    for( int i = 0; i <tree.size(); i++){
        prev_dist = dist_point2node(tree[i], sampled_point[0], sampled_point[1]);
        if(prev_dist < dist){
            dist = prev_dist;
            nearest_node = i;
        }
    }

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is ¡°closer¡± 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z?y|| while at the same time maintaining 
    //||z?x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0
    // x: nearest, y: sampled, z: new

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    double x_slope, y_slope;
    // TODO: fill in this method
    if (dist_point2node(nearest_node, sampled_point[0], sampled_point[1]) < max_expansion_dist){
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else{
        if(abs(nearest_node.x - sampled_point[0])< min_delta){
            y_slope = (sampled_point[0]-nearest_node.x)/(sampled_point[1]-nearest_node.y);
            new_node.y = nearest_node.y + max_expansion_dist/sqrt(1 + y_slope*y_slope);
            new_node.x = nearest_node.x + y_slope *(new_node.y - nearest_node.y);
        }
        else{
            x_slope = (sampled_point[1]-nearest_node.y)/(sampled_point[0]-nearest_node.x);
            new_node.x = nearest_node.x + max_expansion_dist *sqrt(1 + x_slope*x_slope);
            new_node.y = nearest_node.y + x_slope *(new_node.x - nearest_node.x);
        }
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

    collision = line_collision(nearest_node.x, nearest_node.y, new_node.x, new_node.y);
    if(!collision){
        collision = boarder_collision(nearest_node.x, nearest_node.y, new_node.x, new_node.y);
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
    double dist;
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