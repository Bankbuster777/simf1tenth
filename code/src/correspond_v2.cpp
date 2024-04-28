#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include <ros/ros.h>

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      // Do for each point
      for(int i = 0; i<n; ++i){
        for(int j = 0; j<m; ++j){
          float dist = old_points[j].distToPoint2(&trans_points[i]); // change i and j 
          if(dist < min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = (j>0) ? (j-1) : 1;

          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input
  //  old_points   : vector of struct points containing the old points (points of the previous frame)
  //  trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  //  points       : vector of struct points containing the new points
  //  jump_table   : jump table computed using the helper functions from the transformed and old points
  //  c            : vector of struct correspondences .
  //                 This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output 
  //  c            : update the correspondence vector in place which is provided as a reference.
  //                 you need to find the index of the best and the second best point. 
  
  // Initializecorrespondences
  c.clear();
  int j_last_best = -1;
  const int n = trans_points.size();  // index of i
  const int m = old_points.size();    // index of j

  // Do for each transformed point
  for(int i = 0; i<n; ++i){
    int j_best = 0;
    float dist_best = 10000;
    int j_second_best = 0;
    float dist_second_best = 10000;

    int j_start = i;
    int j_up = ((j_last_best > 0) ? ((j_last_best == n) ? n : j_last_best + 1) : i);
    int j_down = (j_up > 0) ? j_up - 1 : 0;
    float last_dist_up = 10000;
    float last_dist_down = 20000;
    bool up_stopped = false;
    bool down_stopped = false; 

    while(!(up_stopped && down_stopped)){
      bool now_up = !up_stopped & (last_dist_up < last_dist_down);
      // In the up search
      if(now_up){
        if(j_up > m-1) {up_stopped = true; continue;}
        last_dist_up = trans_points[i].distToPoint2(&old_points[j_up]);
        if(last_dist_up < dist_best){
          j_best = j_up;
          dist_best = last_dist_up;
          j_second_best = (j_best > 0) ? (j_best - 1):(1);
          dist_second_best = trans_points[i].distToPoint2(&old_points[j_second_best]);
        }
        // Early termination
        if(j_up > j_start){
          float delta_pi = trans_points[i].radialGap(&old_points[j_up]);
          float min_dist_up = sin(delta_pi) * trans_points[i].r;
          if(min_dist_up*min_dist_up > dist_best) {up_stopped = true; continue;}
          if(trans_points[i].distToPoint2(&old_points[j_up]) != 0){
            float theta = acosf((old_points[j_up].r*old_points[j_up].r + trans_points[i].distToPoint2(&old_points[j_up]) -trans_points[i].r*trans_points[i].r)/
                                (2*trans_points[i].distToPoint(&old_points[j_up])*old_points[j_up].r));
            // Using jump table
            j_up = ((theta > M_PI/2) ? jump_table[j_up][UP_BIG] : jump_table[j_up][UP_SMALL]);
          }
          else{
            j_up++;
          }
        }
        else {
          j_up++;
        }
      }// if(now_up)
      if(!now_up){
        if(j_down < 1) {down_stopped = true; continue;}
        last_dist_down = trans_points[i].distToPoint2(&old_points[j_down]);
        if(last_dist_down < dist_best){
          j_best = j_down;
          dist_best = last_dist_down;
          j_second_best = j_best + 1;
          dist_second_best = trans_points[i].distToPoint2(&old_points[j_second_best]);
        }

        if(j_down < j_start){
          float delta_pi = trans_points[i].radialGap(&old_points[j_down]);
          float min_dist_down = sin(delta_pi) * trans_points[i].r;
          if(min_dist_down*min_dist_down > dist_best) {down_stopped = true; continue;}
          if(trans_points[i].distToPoint2(&old_points[j_down]) != 0){
            float theta = acosf((old_points[j_down].r*old_points[j_down].r + trans_points[i].distToPoint2(&old_points[j_down]) -trans_points[i].r*trans_points[i].r)/
                                (2*trans_points[i].distToPoint(&old_points[j_down])*old_points[j_down].r));
            // Using jump table
            j_down = ((theta > M_PI/2) ? jump_table[j_down][DOWN_BIG] : jump_table[j_down][DOWN_SMALL]);
          }
          else{
            j_down--;
          }
      
        }
        else{
          j_down--;
        }
      }
    }// while(!(up_stopped && down_stopped))
    // Set null correspondence if no point matched

    // For the next point, we will start at best
    j_last_best = j_best;

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[j_best], &old_points[j_second_best]));
  }
}

void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
