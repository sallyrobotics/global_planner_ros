#include <stdio.h>

/** include ros libraries**********************/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/** ********************************************/ 


/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using namespace std;
using std::string;

#ifndef STRAIGHT_ROS_CPP
#define STRAIGHT_ROS_CPP


namespace straight_planner_ns {
  
class straight_planner : public nav_core::BaseGlobalPlanner {
public:
  
  straight_planner (ros::NodeHandle &); //this constructor is may be not needed
  straight_planner ();
  straight_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  
  ros::NodeHandle ROSNodeHandle;
  
  /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, 
		std::vector<geometry_msgs::PoseStamped>& plan
	       );
 

  void getCorrdinate (float& x, float& y);
  int convertToCellIndex (float x, float y);
  void convertToCoordinate(int index, float& x, float& y);
  bool isCellInsideMap(float x, float y);
  void mapToWorld(double mx, double my, double& wx, double& wy);
  vector<int> planner_func(int startCell, int goalCell);
  vector<int> findPath(int startCell, int goalCell, int width, int height);
  vector <int> findFreeNeighborCell (int CellID);
  bool isStartAndGoalCellsValid(int startCell,int goalCell); 
  bool isFree(int CellID); //returns true if the cell is Free
  bool isFree(int i, int j); 

  int getCellIndex(int i,int j) //get the index of the cell to be used in Path
  {
   return (i*width)+j;  
  }
  int getCellRowID(int index)//get the row ID from cell index
  {
     return index/width;
  }
  int getCellColID(int index)//get colunm ID from cell index
  {
    return index%width;
  }

  float originX;
  float originY;
  float resolution;
  costmap_2d::Costmap2DROS* costmap_ros_;
  double step_size_, min_dist_from_robot_;
  costmap_2d::Costmap2D* costmap_;
  bool initialized_;
  int width;
  int height;
};

};
#endif
