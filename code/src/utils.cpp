#include "rrt/rrt_v2.h"

std::vector<int> RRT::point2cell(double x_point, double y_point){
    std::vector<int> cell_(2);
    cell_[0] = (x_point - map_posx)/map_resolution;
    cell_[1] = (y_point - map_posy)/map_resolution;

    return cell_;
}
std::vector<double> RRT::cell2point(int x_cell, int y_cell){
    std::vector<double> point_(2);
    point_[0] = x_cell*map_resolution + map_posx;
    point_[1] = y_cell*map_resolution + map_posy;

    return point_;
}

std::vector<int> RRT::data2cell(int data_index){
    std::vector<int> cell_(2);
    cell_[0] = data_index % map_width;
    cell_[1] = data_index/map_width;
    
    return cell_;
}

int RRT::cell2data(int x_cell, int y_cell){
    int cell_index_ = -1;
    if ((0 <= x_cell && x_cell < map_width) && 
       (0 <= y_cell && y_cell < map_height)){
        cell_index_ = x_cell + y_cell * map_width;
    }

    return cell_index_;
}

std::vector<int> RRT::extended_cell2data(int x_cell, int y_cell){
    std::vector<int> cell_index_(9);
    int near_x, near_y;
    for(int i=0; i< cell_index_.size();i++){
        near_x = x_cell + i%3 - 1;
        near_y = y_cell + i/3 - 1;
        cell_index_[i] = cell2data(near_x, near_y);
    }

    return cell_index_;
}

double RRT::dist_point2node(Node& node, double& x_point, double& y_point){
    double distance = 0;
    distance = sqrt((node.x - x_point)*(node.x - x_point)+ (node.y - y_point)*(node.y - y_point));
    
    return distance;
}

void RRT::set_vacant(int x_cell, int y_cell){
    int cell_index_;
    cell_index_ = cell2data(x_cell, y_cell);
    if(cell_index_ >= 0){
        mini_map.data[cell_index_] = vacant;
    }
}

void RRT::set_occupied(int x_cell, int y_cell){
    // int cell_index_;
    // cell_index_ = cell2data(x_cell, y_cell);
    // if(cell_index_ >= 0){
    //     if (mini_map.data[cell_index_] < occupied){
    //         mini_map.data[cell_index_] = occupied - 10;
    //     }
    //     else{
    //         mini_map.data[cell_index_] = occupied;
    //     }
    // }

    std::vector <int> e_cell_index_;
    e_cell_index_ = extended_cell2data(x_cell, y_cell);
    for(int i =0; i< e_cell_index_.size(); i++){
        if(e_cell_index_[i] >= 0){
            if (mini_map.data[e_cell_index_[i]] < occupied){
                mini_map.data[e_cell_index_[i]] = occupied - 10;
            }
            else{
                mini_map.data[e_cell_index_[i]] = occupied;
            }
    }
    }

}

std::vector<cell> RRT::bresenham_line(int x0, int y0, int x1, int y1){
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
        if(y1 > y0){
            unoccupied = bresenham_high(x0,y0,x1,y1);
        }
        else{
            unoccupied = bresenham_high(x1,y1,x0,y0);
        }
    }
    return unoccupied;
}
std::vector<cell> RRT::bresenham_low(int x0, int y0, int x1, int y1){
    std::vector<cell> unoccupied;
    cell s_grid;
    int dx, dy, D, y, yi;
    dx = x1 - x0;
    dy = y1 - y0;
    y = y0;
    yi = 1;
    unoccupied.clear();
    if (dy < 0){
        yi = -1;
        dy =  (-1) * dy;
    }
    D = 2*dy - dx;
    for (int xi =0; xi<abs(dx); xi++){
        s_grid.x = x0+xi;   s_grid.y = y;
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

    unoccupied.clear();
    if (dx < 0){
        xi = -1;
        dx =  (-1) * dx;
    }
    D = dy - 2*dx; //
    for (int yi =0; yi<abs(dy); yi++){
        s_grid.x = x;   s_grid.y = y0+yi;
        unoccupied.push_back(s_grid);
        if (D > 0){
            D = D - 2*dx; // -2*dx
        }
        else{
            x = x + xi;
            D = D - 2*dx + 2*dy; // was -2*dx
        }
    }
    return unoccupied;
}

bool RRT::is_occupied(double& x_point, double& y_point){
    std::vector<int> cell_(2);
    int cell_index;
    bool occupancy = false;
    cell_ = point2cell(x_point, y_point);
    cell_index = cell2data(cell_[0], cell_[1]);
    if(mini_map.data[cell_index] > occup_threshold){
        occupancy = true;
    }

    return occupancy;
}

bool RRT::is_cell_occupied(int x_cell, int y_cell){
    int cell_index;
    bool occupancy = false;
    cell_index = cell2data(x_cell, y_cell);
    if(mini_map.data[cell_index] > occup_threshold){
        occupancy = true;
    }

    return occupancy;
}

bool RRT::line_collision(double start_x, double start_y, double end_x, double end_y){
    bool is_collision = false;
    std::vector<int> start_cell(2), end_cell(2);
    std::vector<cell> inbetween_cell;
    start_cell = point2cell(start_x, start_y);
    end_cell = point2cell(end_x, end_y);

    inbetween_cell = bresenham_line(start_cell[0], start_cell[1], end_cell[0], end_cell[1]);
    int i = 0;
    while((i < inbetween_cell.size()) && !(is_collision)){
        is_collision = is_cell_occupied(inbetween_cell[i].x, inbetween_cell[i].y);
        i++;
    }

    return is_collision;
}

bool RRT::boarder_collision(double start_x, double start_y, double end_x, double end_y){
    bool is_collision = false;

    if ((start_x - end_x)*(start_y - end_y) > 0){
        is_collision = line_collision(start_x + vehicle_width/2, start_y - vehicle_width/2, end_x + vehicle_width/2, end_y - vehicle_width/2)
                    || line_collision(start_x - vehicle_width/2, start_y + vehicle_width/2, end_x - vehicle_width/2, end_y + vehicle_width/2);
    }
    else{
        is_collision = line_collision(start_x + vehicle_width/2, start_y + vehicle_width/2, end_x + vehicle_width/2, end_y + vehicle_width/2)
                    || line_collision(start_x - vehicle_width/2, start_y - vehicle_width/2, end_x - vehicle_width/2, end_y - vehicle_width/2);

    }

    return is_collision;
}

bool RRT::cage_collision(double start_x, double start_y, double end_x, double end_y){
    bool is_collision = false;

    if ((start_x - end_x)*(start_y - end_y) > 0){
        is_collision = line_collision(start_x + vehicle_width/4, start_y - vehicle_width/4, end_x + vehicle_width/4, end_y - vehicle_width/4)
                    || line_collision(start_x - vehicle_width/4, start_y + vehicle_width/4, end_x - vehicle_width/4, end_y + vehicle_width/4);
    }
    else{
        is_collision = line_collision(start_x + vehicle_width/4, start_y + vehicle_width/4, end_x + vehicle_width/4, end_y + vehicle_width/4)
                    || line_collision(start_x - vehicle_width/4, start_y - vehicle_width/4, end_x - vehicle_width/4, end_y - vehicle_width/4);
    }

    return is_collision;
}

std::vector<geometry_msgs::PointStamped> RRT::read_csv(std::string csv_file){
    std::fstream csv_wp;
    std::string x_str, y_str, file_name;
    std::vector<geometry_msgs::PointStamped> v_trace;
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
                single_waypoint.header.frame_id = map_frame;
                single_waypoint.point.x = std::stof(x_str);
                single_waypoint.point.y = std::stof(y_str);
                v_trace.push_back(single_waypoint);
            }
        }
        ROS_INFO("Read csv done");
        ROS_INFO("First : %f %f", v_trace.front().point.x, v_trace.front().point.y);
        ROS_INFO("Number of wp: %d", v_trace.size());
        csv_wp.close();
    }

    return v_trace;
}