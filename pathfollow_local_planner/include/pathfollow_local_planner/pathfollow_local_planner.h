#ifndef PATHFOLLOW_LOCAL_PLANNER_H_
#define PATHFOLLOW_LOCAL_PLANNER_H_

#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "tf2/LinearMath/Quaternion.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <pathfollow_local_planner/PathFollowLocalPlannerConfig.h>

#include <algorithm>

#define PI 3.1415926543

namespace pathfollow_local_planner
{
    class PathFollowLocalPlanner : public nav_core::BaseLocalPlanner
    {
        public:
           PathFollowLocalPlanner();
           ~PathFollowLocalPlanner();

           void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

           bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
           bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
           bool isGoalReached();

        private:
           void publishNextHeading(bool show = true);
           bool rotateToStart(geometry_msgs::Twist& cmd_vel);
           bool move(geometry_msgs::Twist& cmd_vel);
           bool rotateToGoal(geometry_msgs::Twist& cmd_vel);
           void computeNextHeadingIndex();
           double calLinearVel();
           double calRotationVel(double rotation);
           double linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
           void reconfigureCB(PathFollowLocalPlannerConfig &config, uint32_t level);
           bool judgmentIsKeyPoint(std::vector<geometry_msgs::PoseStamped> global_plan);
           bool moveToKeyPoint(geometry_msgs::Twist& cmd_vel);
           double calculateAngleDifference(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);

	       dynamic_reconfigure::Server<PathFollowLocalPlannerConfig> *dsrv_;
           
           typedef enum
           {
               RotatingToStart,
               Moving,
               MovingToKeyPoint,
               RotatingToGoal,
               Finished
           } State;

           State state_;
            
           tf2_ros::Buffer* tf_;
           costmap_2d::Costmap2DROS* costmap_ros_;
           std::vector<geometry_msgs::PoseStamped> global_plan_;
            
           geometry_msgs::PoseStamped robot_pose_;
           ros::Publisher next_heading_pub_;

           int curr_heading_index_, next_heading_index_, next_heading_index_temp_;
           int path_index_;

           ros::Publisher global_plan_pub_;
            
           // Parameters
           std::string map_frame_;

           struct constraint_vel
           {
               double max_vel;
               double min_vel;
           };

           constraint_vel linear_vel_, rotation_vel_;
           
           double heading_lookahead_;
           double yaw_tolerance_, xy_tolerance_;
           double yaw_moving_tolerance_;
           double transform_timeout_;

           bool use_BackForward;

           std::vector<geometry_msgs::Point> footprint_pos;

           geometry_msgs::Point key_point_;
    };

};

#endif