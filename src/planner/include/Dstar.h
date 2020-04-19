/* Dstar.h
 * James Neufeld (neufeld@cs.ualberta.ca)
 * Compilation fixed by Arek Sredzki (arek@sredzki.com)
 */

#ifndef DSTAR_H
#define DSTAR_H

#include <cmath>
#include <stack>
#include <queue>
#include <list>
#include <cstdio>
#include <backward/hash_map>
#include <unordered_map>

//ROS Inclusions:
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
//End ROS Inclusions

using namespace std;
using namespace __gnu_cxx;

class state { //
 public:
  int x;
  int y;
  pair<double,double> k;

  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }

  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }

  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;
  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;
  }


  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;
  }

  state operator - (const state &s2) const { //MIGHT BE USING FOR DRIVE DIRECTIONS IN LocalPlanningNode.CPP
    return state{(x-s2.x), (y-s2.y)} ;
  }

};

struct ipoint2 {
  int x,y;
};

struct cellInfo {

  double g;
  double rhs;
  double cost;

};

class state_hash {
 public:
  size_t operator()(const state &s) const {
   // return (s.x + 34245*s.y);// % 999983; //TODO: Double hashing, or remove hashinig all together
    return (s.x + 250807*s.y);// % 999983; //TODO: Double hashing, or remove hashinig all together
  }
};


typedef priority_queue<state, vector<state>, greater<state> > ds_pq;
typedef hash_map<state, cellInfo, state_hash, equal_to<state> > ds_ch;
typedef hash_map<state, float, state_hash, equal_to<state> > ds_oh;


class Dstar {

 public:
  Dstar();
  void   init(int sX, int sY, int gX, int gY);
  void   updateCell(int x, int y, double val);
  void   updateStart(int x, int y);
  void   updateGoal(int x, int y);
  bool   replan();

  list<state> getPath();

 private:

  list<state> path;

  double C1;
  double k_m;
  state s_start, s_goal, s_last;
  int maxSteps;

  ds_pq openList; // cells to be expanded
  ds_ch cellHash; //uses state_hash ; cells in the graph
  ds_oh openHash; //uses state_hash ; cells to be expanded

  bool   close(double x, double y);
  void   makeNewCell(state u);
  double getG(state u);
  double getRHS(state u);
  void   setG(state u, double g);
  double setRHS(state u, double rhs);
  double eightCondist(state a, state b);
  int    computeShortestPath();
  void   updateVertex(state u);
  void   insert(state u);
  void   remove(state u);
  double trueDist(state a, state b);
  double heuristic(state a, state b);
  state  calculateKey(state u);
  void   getSucc(state u, list<state> &s);
  void   getPred(state u, list<state> &s);
  double cost(state a, state b);
  bool   occupied(state u);
  bool   isValid(state u);
  float  keyHashCode(state u);
};

#endif
