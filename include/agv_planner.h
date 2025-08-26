#ifndef AGV_PLANNER_H_
#define AGV_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace agv_planner
{
    class AgvPlanner: public nav_core::BaseLocalPlanner
    {
        public:
            AgvPlanner();
            ~AgvPlanner ();

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);
    };
}

#endif //AGV_PLANNER_H_