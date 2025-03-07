#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include <iostream>

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                            vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{

  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();
  int min_index = 0;
  int second_min_index = 0;

  //Do for each point
  for (int ind_trans = 0; ind_trans < n; ++ind_trans)
  {
    float min_dist = 100000.00;
    for (int ind_old = 0; ind_old < m; ++ind_old)
    {
      float dist = old_points[ind_trans].distToPoint2(&trans_points[ind_old]);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_index = ind_old;
        if (ind_old == 0)
        {
          second_min_index = ind_old + 1;
        }
        else
        {
          second_min_index = ind_old - 1;
        }
      }
    }
    c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[min_index], &old_points[second_min_index]));
  }
}
/*
void getCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                       vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point.
  //Initializecorrespondences
  c.clear();
  const int trans_size = trans_points.size();
  const int old_size = old_points.size();
  
  // cout << "trans_size = " << trans_size << "\n";
  // cout << "old_size   = " << old_size << "\n";
  cout.flush();

  // for (int i = 0; i < old_points.size(); i++) {
  //   Point& p = old_points[i];
  //   cout << "old_points[" << i << "] = " << p.r << ", " << p.theta << "\n";
  // }
  // cout.flush();

  // Find a p0
  const Point &p0 = old_points[0];

  // const Point& p0 = old_points[0];

  //Do for each point

  int last_best = -1;
  int min_size = min(old_size, trans_size);
  for (int ind_trans = 0; ind_trans < min_size; ++ind_trans)
  {
    /// TODO: Implement Fast Correspondence Search

    Point& piw = trans_points.at(ind_trans);
    if (isnan(piw.theta))
      continue;

    int best = -1;  // invalid
    int second_best = -1;
    float best_dist = INFINITY;
    
    int nrays = min_size;
    int start_index = (piw.theta - p0.theta) * (nrays / 2.0 / M_PI);
    if (start_index < 0) start_index = 0;
    if (start_index >= min_size) start_index = min_size - 1;
    // cout << "start_index = " << start_index << "\n";
    // cout << "piw.theta   = " << piw.theta << "\n";
    // cout << "p0.theta   = " << p0.theta << "\n";
    
    int we_start_at = (last_best != -1) ? (last_best + 1) : start_index;
    // cout << "we_start_at = " << we_start_at << "\n";
    // cout << "last_best   = " << last_best << "\n";
    // cout << "start_index = " << start_index << "\n";
    
    int up = we_start_at + 1;
    int down = we_start_at;

    float last_dist_up = INFINITY;
    float last_dist_down = INFINITY;

    bool up_stopped = false;
    bool down_stopped = false;

    while (!(up_stopped && down_stopped)) {

      // std::cout << up_stopped << " & " << down_stopped << std::endl;
      // std::cout << "up   = " << up << std::endl;
      // std::cout << "down = " << down << std::endl;
      
      bool now_up = (!up_stopped & (last_dist_up < last_dist_down)) | down_stopped;
      if (now_up) {
        // std::cout << "up\n";
        if (up >= nrays) { up_stopped = true; continue; }
        Point& pup = trans_points.at(up);  // correct?
        last_dist_up = piw.distToPoint2(&pup);  // correct?
        bool corr_acceptable = true;
        if (corr_acceptable && last_dist_up < best_dist) {
          second_best = best;
          best = up;
          best_dist = last_dist_up;
        }
        if (up > start_index) {
          float delta_dir = pup.theta - piw.theta;
          float min_dist_up = sin(delta_dir) * piw.r;
          if (min_dist_up*min_dist_up > best_dist) {
            up_stopped = true; continue;
          }
          float newUp = (pup.r < piw.r)
              ? jump_table.at(up).at(UP_BIG)
              : jump_table.at(up).at(UP_SMALL);
          up = (newUp == up) ? up+1 : newUp;
          // std::cout << "up   = " << up << std::endl;
        } else
          up++;
      }  // end if (now_up)
      if (!now_up) {
        // std::cout << "down\n";
        if (down < 0) { down_stopped = true; continue; }
        Point& pdown = trans_points.at(down);  // correct?
        last_dist_down = piw.distToPoint2(&pdown);  // correct?
        bool corr_acceptable = true;
        if (corr_acceptable && last_dist_down < best_dist)
          second_best = best, best = down, best_dist = last_dist_down;
        if (down > start_index) {
          float delta_dir = pdown.theta - piw.theta;
          float min_dist_down = sin(delta_dir) * piw.r;
          if (min_dist_down*min_dist_down > best_dist) {
            down_stopped = true; continue;
          }
          down = (pdown.r < piw.r)
              ? jump_table.at(down).at(DOWN_BIG)
              : jump_table.at(down).at(DOWN_SMALL);
        } else
          down--;
      }  // end if (now_up)

      

    }  // end while loop
    
    last_best = best;

    // END

    // int second_best = 0;
    if (second_best == -1) second_best = best;

    // int second_best = best;  // for now

    c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[best], &old_points[second_best]));
  }

  // std::cout << "For loop done?" << std::endl;

}

void computeJump(vector<vector<int>> &table, vector<Point> &points)
{
  table.clear();
  int n = points.size();
  for (int i = 0; i < n; ++i)
  {
    vector<int> v = {n, n, -1, -1};
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r < points[i].r)
      {
        v[UP_SMALL] = j;
        break;
      }
    }
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r > points[i].r)
      {
        v[UP_BIG] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r < points[i].r)
      {
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r > points[i].r)
      {
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
*/