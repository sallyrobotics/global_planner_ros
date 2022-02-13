/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>
using std::string;

namespace template_planner_ns {

	class template_planner : public nav_core::BaseGlobalPlanner {
		public:

			template_planner();
			template_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

			/** overridden classes from interface nav_core::Basetemplate_planner **/
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped& start,
					const geometry_msgs::PoseStamped& goal,
					std::vector<geometry_msgs::PoseStamped>& plan
				     );
	};
};

//register this planner as a Basetemplate_planner plugin
PLUGINLIB_EXPORT_CLASS(template_planner_ns::template_planner, nav_core::BaseGlobalPlanner)
