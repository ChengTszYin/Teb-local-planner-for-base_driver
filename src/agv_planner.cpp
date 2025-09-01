#include "agv_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS( agv_planner::AgvPlanner, nav_core::BaseLocalPlanner)

namespace agv_planner
{
    AgvPlanner::AgvPlanner()
    {
        // setlocale(LC_ALL, "");
    }
    AgvPlanner::~AgvPlanner()
    {

    }

    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    void AgvPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("STARTED");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
    }
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index;
    bool xy_approaching;
    bool pose_adjusting_;
    bool goal_reached;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_ ;
    bool AgvPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        xy_approaching = true;
        goal_reached = false;

        
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        ros::Rate rate(10.0);

        return true;
    }

    bool AgvPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link", global_plan_[final_index], pose_final);
        if(xy_approaching)
        {
            if(pose_adjusting_==false)
            {
                double dx = pose_final.pose.position.x;
                double dy = pose_final.pose.position.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if(dist < 0.05)
                {
                    pose_adjusting_=true;
                    xy_approaching = false;
                    cmd_vel.linear.x = 0.00;
                    cmd_vel.angular.z = 0.00; 
                }
            }

            geometry_msgs::PoseStamped target_pose;
            for(int i=target_index;i<global_plan_.size();i++)
            {
                geometry_msgs::PoseStamped pose_base;
                global_plan_[i].header.stamp = ros::Time(0);
                tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
                double dx = pose_base.pose.position.x;
                double dy = pose_base.pose.position.y;
                double dist = std::sqrt(dx*dx + dy*dy);

                if (dist > 0.2)
                {
                    target_pose = pose_base;
                    target_index = i;
                    ROS_WARN("target_index at %d, distance=%.2f", target_index, dist);
                    break;
                }
                if (i==global_plan_.size()-1)
                {
                    target_pose = pose_base;
                }
            }
                cmd_vel.linear.x = target_pose.pose.position.x;
                cmd_vel.angular.z = target_pose.pose.position.y * 5.0;
        }

        if(pose_adjusting_)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("Final_yaw=%.2f", final_yaw);
            // cmd_vel.linear.x = pose_final.pose.position.x * 1.0;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.3;
            if(abs(final_yaw) < 0.2)
            {
                goal_reached = true;
                ROS_WARN("Goal Reached");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0; 
            }
        }
        

        return true;
    }

    bool AgvPlanner::isGoalReached()
    {
        return goal_reached;
    }
}
