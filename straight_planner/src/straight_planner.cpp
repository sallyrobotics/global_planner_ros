#include <stdio.h>

#include "straight_planner.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(straight_planner_ns::straight_planner, nav_core::BaseGlobalPlanner)


int value;
int mapSize;
bool* OGM;


inline vector <int> findFreeNeighborCell (int CellID);

namespace straight_planner_ns
{

//Default Constructor
straight_planner::straight_planner()
{

}
straight_planner::straight_planner(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;

}

straight_planner::straight_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void straight_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

  if (!initialized_)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);

    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();



	width = costmap_->getSizeInCellsX();
	height = costmap_->getSizeInCellsY();
	resolution = costmap_->getResolution();
	mapSize = width*height;
	value =0;


	OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        if (cost == 0)
          OGM[iy*width+ix]=true;
        else
          OGM[iy*width+ix]=false;
      }
    }



    ROS_INFO("Straight line planner initialized successfully");
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

bool straight_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{

  if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  tf::Stamped < tf::Pose > goal_tf;
  tf::Stamped < tf::Pose > start_tf;

  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  // convert the start and goal positions

  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;

  getCorrdinate(startX, startY);
  getCorrdinate(goalX, goalY);

  int startCell;
  int goalCell;

  if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
  {
    startCell = convertToCellIndex(startX, startY);

    goalCell = convertToCellIndex(goalX, goalY);


  }
  else
  {
    ROS_WARN("the start or goal is out of the map");
    return false;
  }

  /////////////////////////////////////////////////////////

  // call global planner

  if (isStartAndGoalCellsValid(startCell, goalCell)){

        vector<int> bestPath;
	bestPath.clear();

    bestPath = planner_func(startCell, goalCell);

//if the global planner find a path
    if ( bestPath.size()>0)
    {

// convert the path

      for (int i = 0; i < bestPath.size(); i++)
      {

        float x = 0.0;
        float y = 0.0;

        int index = bestPath[i];

        convertToCoordinate(index, x, y);

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
      }


	float path_length = 0.0;
	
	std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
	
	geometry_msgs::PoseStamped last_pose;
	last_pose = *it;
	it++;
	for (; it!=plan.end(); ++it) {
	   path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
		                 (*it).pose.position.y - last_pose.pose.position.y );
	   last_pose = *it;
	}
	cout <<"The global path length: "<< path_length<< " meters"<<endl;
      //publish the plan

      return true;

    }

    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }

  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }

}
void straight_planner::getCorrdinate(float& x, float& y)
{

  x = x - originX;
  y = y - originY;

}

int straight_planner::convertToCellIndex(float x, float y)
{

  int cellIndex;

  float newX = x / resolution;
  float newY = y / resolution;

  cellIndex = getCellIndex(newY, newX);

  return cellIndex;
}

void straight_planner::convertToCoordinate(int index, float& x, float& y)
{

  x = getCellColID(index) * resolution;

  y = getCellRowID(index) * resolution;

  x = x + originX;
  y = y + originY;

}

bool straight_planner::isCellInsideMap(float x, float y)
{
  bool valid = true;

  if (x > (width * resolution) || y > (height * resolution))
    valid = false;

  return valid;
}

void straight_planner::mapToWorld(double mx, double my, double& wx, double& wy){
   costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
}

vector<int> straight_planner::planner_func(int startCell, int goalCell){

   vector<int> bestPath;

cout<<endl<<"MapSize is: "<<mapSize<<endl;




  bestPath=findPath(startCell, goalCell, width, height);

  return bestPath;

}


/*******************************************************************************/
//Function Name: findPath
//Inputs: Start and goal cells in form of row-major index of single dimentional 1xwidth*height array, width and height of the grid.
//Output: the best path
//Description: it is used to generate the robot free path
/*********************************************************************************/
vector<int> straight_planner::findPath(int startCell, int goalCell, int width, int height)
{
	cout<<endl<<"Width & Height of the map: "<<width<<" "<<height<<endl;
	int start_x = getCellRowID(startCell);
	int start_y = getCellColID(startCell);
	int goal_x = getCellRowID(goalCell);
	int goal_y = getCellColID(goalCell);
	vector <int> bestPath;
	bestPath.push_back(startCell);
	int iterator = 0;
	float dx = goal_x - start_x;
	float dy = goal_y - start_y;
	float abs_dx = dx;
	float abs_dy = dy;
	(abs_dx<0)?(abs_dx = -1*abs_dx):(abs_dx = abs_dx);
	(abs_dy<0)?(abs_dy = -1*abs_dy):(abs_dy = abs_dy);
	int x = start_x;
	int y = start_y;
	float x_est;
	float y_est;
	while ((x != goal_x) && (y != goal_y))
	{
		if(abs_dx > abs_dy)
		{
			if ((goal_x > start_x) && (goal_y > start_y))
			{
				x+=1;
				y_est = start_y + (dy/dx)*(x - start_x);
				((y_est - y)>(y+1 - y_est)) ? (y = y+1) : (y = y);
				bestPath.push_back(getCellIndex(x, y));
			}
			else if ((goal_x > start_x) && (goal_y <= start_y))
			{
				x+=1;
				y_est = start_y + dy/dx*(x - start_x);
				((y - y_est) > (y_est - y-1)) ? (y = y-1) : (y = y);
				bestPath.push_back(getCellIndex(x, y));
			}
			else if ((goal_x <= start_x) && (goal_y > start_y))
			{
				x-=1;
				y_est = start_y + dy/dx*(x - start_x);
				((y_est - y)>(y+1 - y_est)) ? (y = y+1) : (y = y);
				bestPath.push_back(getCellIndex(x, y));
			}
			else if ((goal_x <= start_x) && (goal_y <= start_y))
			{
				x-=1;
				y_est = start_y + dy/dx*(x - start_x);
				((y - y_est) > (y_est - y-1)) ? (y = y-1) : (y = y);
				bestPath.push_back(getCellIndex(x, y));
			}

		}
		else
		{
			if ((goal_x > start_x) && (goal_y > start_y))
			{
				y+=1;
				x_est = start_x + (dx/dy)*(y - start_y);
				((x_est - x)>(x+1 - x_est)) ? (x = x+1) : (x = x);
				bestPath.push_back(getCellIndex(x, y));
			}
			else if ((goal_x > start_x) && (goal_y <= start_y))
			{
				y+=1;
				x_est = start_x + (dx/dy)*(y - start_y);
				((x_est - x)>(x+1 - x_est)) ? (x = x+1) : (x = x);
				bestPath.push_back(getCellIndex(x, y));
			}
			else if ((goal_x <= start_x) && (goal_y > start_y))
			{
				y-=1;
				x_est = start_x + (dx/dy)*(y - start_y);
				((x - x_est)>(x_est - x-1)) ? (x = x-1) : (x = x);
				bestPath.push_back(getCellIndex(x, y));
			}
			else if ((goal_x <= start_x) && (goal_y <= start_y))
			{
				y-=1;
				x_est = start_x + (dx/dy)*(y - start_y);
				((x - x_est)>(x_est - x-1)) ? (x = x-1) : (x = x);
				bestPath.push_back(getCellIndex(x, y));
			}
		}
	}
	return bestPath;
}


  /*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/

vector <int> straight_planner::findFreeNeighborCell (int CellID){
 
  int rowID=getCellRowID(CellID);
  int colID=getCellColID(CellID);
  int neighborIndex;
  vector <int>  freeNeighborCells;

  for (int i=-1;i<=1;i++)
    for (int j=-1; j<=1;j++){
      //check whether the index is valid
     if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
	neighborIndex = getCellIndex(rowID+i,colID+j);
        if(isFree(neighborIndex) )
	    freeNeighborCells.push_back(neighborIndex);
	}
    }
    return  freeNeighborCells;
 
}

/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool straight_planner::isStartAndGoalCellsValid(int startCell,int goalCell)
{ 
 bool isvalid=true;
 bool isFreeStartCell=isFree(startCell);
 bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
    //cout << "The Start and the Goal cells are the same..." << endl; 
    isvalid = false;
    }
   else
   {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
	//cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
	if (!isFreeStartCell)
	{
	  //cout << "The start is an obstacle..." << endl;
	  isvalid = false;
	}
	else
	{
	    if(!isFreeGoalCell)
	    {
	      //cout << "The goal cell is an obstacle..." << endl;
	      isvalid = false;
	    }
	    else
	    {
	      if (findFreeNeighborCell(goalCell).size()==0)
	      {
		//cout << "The goal cell is encountred by obstacles... "<< endl;
		isvalid = false;
	      }
	      else
	      {
		if(findFreeNeighborCell(startCell).size()==0)
		{
		  //cout << "The start cell is encountred by obstacles... "<< endl;
		  isvalid = false;
		}
	      }
	    }
	}
      }
  }
 return isvalid;
}


 //verify if the cell(i,j) is free
 bool  straight_planner::isFree(int i, int j){
   int CellID = getCellIndex(i, j);
 return OGM[CellID];

 } 

  //verify if the cell(i,j) is free
 bool  straight_planner::isFree(int CellID){
 return OGM[CellID];
 } 
}
;
