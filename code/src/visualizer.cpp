#include "rrt/rrt_v2.h"

void RRT::pub_env_layers(nav_msgs::OccupancyGrid occupancy_map){
    visualization_msgs::Marker marker;
    std::vector<double> point(2);
    int x_idx, y_idx;
    for(int j=0; j < occupancy_map.data.size(); j++){
        point = cell2point(data2cell(j)[0], data2cell(j)[1]);
        if(is_occupied(point[0], point[1])){
            marker.header.frame_id = map_frame;
            marker.header.stamp = ros::Time();
            marker.ns = "environment";
            marker.id = j;
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = map_resolution * 0.9;
            marker.scale.y = map_resolution * 0.9;
            marker.scale.z = map_resolution * 0.9;

            marker.points.resize(9);
            marker.colors.resize(9);
            for(int i=0; i < 9; i++){
                x_idx = i % 3 - 1;
                y_idx = (int) i/3 - 1;
                
                // marker_arr.markers[i].lifetime = ros::Duration(15.0);
                marker.points[i].x = point[0] + x_idx * map_resolution;
                marker.points[i].y = point[1] + y_idx * map_resolution;
                marker.colors[i].a = 1.0;
                marker.colors[i].r = 0.0;
                marker.colors[i].g = 0.0;
                marker.colors[i].b = 0.0;
            }
            viz_env_layer.publish(marker); 
        }
    }
}

void RRT::pub_dyn_layers(geometry_msgs::PointStamped dyn_points){
    visualization_msgs::Marker marker;
    int x_idx, y_idx;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "dynamics";
    marker.id = dyn_ids;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(3.0);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_resolution * 0.9;
    marker.scale.y = map_resolution * 0.9;
    marker.scale.z = map_resolution * 0.9;

    marker.points.resize(25);
    marker.colors.resize(25);
    for(int i=0; i < 16; i++){
        x_idx = i % 5 - 2;
        y_idx = (int) i/5 - 2;
                
        marker.points[i].x = dyn_points.point.x + x_idx * map_resolution;
        marker.points[i].y = dyn_points.point.y + y_idx * map_resolution;
        marker.colors[i].a = 1.0;
        marker.colors[i].r = 0.0;
        marker.colors[i].g = 0.0;
        marker.colors[i].b = 1.0;
    }
    viz_dyn_layer.publish(marker); 
    dyn_ids ++; 

}

void RRT::pub_sta_layers(geometry_msgs::PointStamped sta_points){
    visualization_msgs::Marker marker;
    int x_idx, y_idx;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "statics";
    marker.id = sta_ids;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(3.0);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_resolution * 0.9;
    marker.scale.y = map_resolution * 0.9;
    marker.scale.z = map_resolution * 0.9;

    marker.points.resize(16);
    marker.colors.resize(16);
    for(int i=0; i < 16; i++){
        x_idx = i % 4 - 1.5;
        y_idx = (int) i/4 - 1.5;
                
        marker.points[i].x = sta_points.point.x + x_idx * map_resolution;
        marker.points[i].y = sta_points.point.y + y_idx * map_resolution;
        marker.colors[i].a = 1.0;
        marker.colors[i].r = 1.0;
        marker.colors[i].g = 0.0;
        marker.colors[i].b = 0.0;
    }
    viz_sta_layer.publish(marker); 
    sta_ids ++;
}

void RRT::pub_rrt_vertices(std::vector<Node> vertices){
    visualization_msgs::Marker marker;
    std::vector<double> scan_point(2);

    marker.header.frame_id = scan_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "vertices";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_resolution * 0.9;
    marker.scale.y = map_resolution * 0.9;
    marker.scale.z = map_resolution * 0.9;
    marker.lifetime = ros::Duration(3.0);
    for(int j=0; j < vertices.size(); j++){
        scan_point[0] = vertices[j].x;
        scan_point[1] = vertices[j].y;
        
        marker.points.resize(vertices.size());
        marker.colors.resize(vertices.size());
            
        marker.points[j].x = scan_point[0];
        marker.points[j].y = scan_point[1];
        marker.colors[j].a = 1.0;
        marker.colors[j].r = 0.0;
        marker.colors[j].g = 0.0;
        marker.colors[j].b = 1.0;
    } 
    viz_rrt_vert.publish(marker);  
}

void RRT::pub_rrt_edges(std::vector<Edge> edges){
    visualization_msgs::Marker marker;
    std::vector<double> edge_start(2), edge_end(2);

    marker.header.frame_id = scan_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "edges";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_resolution * 0.7;
    for(int j=0; 2*j < edges.size(); j++){
        edge_start[0] = edges[2*j].n1.x;
        edge_start[1] = edges[2*j].n1.y;
        edge_end[0]   = edges[2*j+1].n2.x;
        edge_end[1]   = edges[2*j+1].n2.y;
        
        marker.points.resize(edges.size());
        marker.colors.resize(edges.size());
            
        // marker_arr.markers[i].lifetime = ros::Duration(15.0);
        marker.points[2*j].x = edge_start[0];
        marker.points[2*j].y = edge_start[1];
        marker.colors[2*j].a = 1.0;
        marker.colors[2*j].r = 0.0;
        marker.colors[2*j].g = 0.0;
        marker.colors[2*j].b = 1.0;
        marker.points[2*j+1].x = edge_end[0];
        marker.points[2*j+1].y = edge_end[1];
        marker.colors[2*j+1].a = 1.0;
        marker.colors[2*j+1].r = 0.0;
        marker.colors[2*j+1].g = 0.0;
        marker.colors[2*j+1].b = 1.0;
    } 
    viz_rrt_edge.publish(marker);  

}

void RRT::pub_rrt_path(std::vector<Node> tree){
    visualization_msgs::Marker marker;
    std::vector<double> node_point(2);

    marker.header.frame_id = scan_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "path_vertex";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_resolution * 0.8;
    for(int j=0; j < tree.size(); j++){
        node_point[0] = tree[j].x;
        node_point[1] = tree[j].y;
        
        marker.points.resize(tree.size());
        marker.colors.resize(tree.size());
            
        // marker_arr.markers[i].lifetime = ros::Duration(15.0);
        marker.points[j].x = node_point[0];
        marker.points[j].y = node_point[1];
        marker.colors[j].a = 1.0;
        marker.colors[j].r = 0.0;
        marker.colors[j].g = 0.0;
        marker.colors[j].b = 1.0;
    } 
    viz_rrt_path.publish(marker);  


}