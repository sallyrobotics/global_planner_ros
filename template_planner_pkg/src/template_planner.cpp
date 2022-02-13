#include <pluginlib/class_list_macros.h>
#include "template_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(template_planner_ns::template_planner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace template_planner_ns {

	template_planner::template_planner (){

	}

	template_planner::template_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
			initialize(name, costmap_ros);
	}


	void template_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	}

	bool template_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
		plan.push_back(start);
		plan.push_back(goal);
		return true;
	}
};
