#include "agv_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
    bool pose_adjusting_;
    bool goal_reached;
    bool AgvPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached = false;
        return true;
    }

    bool AgvPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128,128,128));
        for (unsigned int y = 0; y < size_y; y++)
        {
            for (unsigned int x=0; x < size_x; x++)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);

                if(cost == 0)
                    pixel = cv::Vec3b(128, 128, 128);
                else if (cost == 254)
                    pixel = cv::Vec3b(0, 0, 0);
                else if (cost == 253)
                    pixel = cv::Vec3b(255, 255, 0);
                else
                {
                    unsigned char blue = 255 - cost;
                    unsigned char red = cost;
                    pixel = cv::Vec3b(blue, 0, red);  
                }
            }
        }

        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_odom; 
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("odom", global_plan_[i], pose_odom);
            double odom_x = pose_odom.pose.position.x;
            double odom_y = pose_odom.pose.position.y;

            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x;
            double local_y = odom_y - origin_y;
            int x = local_x / costmap->getResolution();
            int y = local_y / costmap->getResolution();
            cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(255,0,255));

            if(i >= target_index && i < target_index + 10)
            {
                cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(0, 255, 255));
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index]; // get cost value to find obstacle
                if(cost >= 253)
                {
                    return false;
                }
            }
        }

        map_image.at<cv::Vec3b> (size_y/2, size_x/2) = cv::Vec3b(0, 255, 0);

        cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128,128,128));
        for (unsigned int y = 0; y < size_y; y++)
        {
            for (unsigned int x=0; x < size_x; x++)
            {
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                flipped_image.at<cv::Vec3b>((size_x - 1 - x), (size_y - 1 - y)) = pixel;
            }
        }
        map_image = flipped_image;
        
        cv::namedWindow("Map");
        cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST);
        cv::resizeWindow("Map", size_y*5, size_x*5);
        cv::imshow("Map", map_image);

        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link", global_plan_[final_index], pose_final);
        if(pose_adjusting_==false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.05)
            {
                pose_adjusting_=true;
            }
        }
        if(pose_adjusting_==true){
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("Final_yaw=%.2f", final_yaw);
            // cmd_vel.linear.x = pose_final.pose.position.x * 1.0;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = final_yaw * 20;
            if(abs(final_yaw) < 0.2)
            {
                goal_reached = true;
                ROS_WARN("Goal Reached");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0; 
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

        cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
            int cv_x = 300 - pose_base.pose.position.y*100;
            int cv_y = 300 - pose_base.pose.position.x*100;
            cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
        }
        cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(255, 0, 255));
        cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);
        // cv::namedWindow("Plan");
        // cv::imshow("Plan", plan_image);
        cv::waitKey(1);
        return true;
    }

    bool AgvPlanner::isGoalReached()
    {
        return goal_reached;
    }
}
