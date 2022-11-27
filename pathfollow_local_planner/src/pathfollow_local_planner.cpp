#include <pathfollow_local_planner/pathfollow_local_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(pathfollow_local_planner::PathFollowLocalPlanner, nav_core::BaseLocalPlanner)

namespace pathfollow_local_planner
{
    PathFollowLocalPlanner::PathFollowLocalPlanner() : tf_(NULL),
									   state_(Finished),
                                       curr_heading_index_(0),
									   next_heading_index_(0),
									   use_BackForward(false)
    {

    }

    PathFollowLocalPlanner::~PathFollowLocalPlanner()
    {
        delete dsrv_;
    }

	// 用作参数动态设置
    void PathFollowLocalPlanner::reconfigureCB(PathFollowLocalPlannerConfig &config, uint32_t level)
    {
        ROS_INFO("PathFollowLocalPlanner reconfigureCB");
		
        map_frame_ = config.map_frame;
        heading_lookahead_ = config.heading_lookahead;
        linear_vel_.max_vel = config.max_linear_vel;
        linear_vel_.min_vel = config.min_linear_vel;
        rotation_vel_.max_vel = config.max_vel_theta;
        rotation_vel_.min_vel = config.min_vel_theta;
        xy_tolerance_ = config.xy_goal_tolerance;
        yaw_tolerance_ = config.yaw_goal_tolerance;
		yaw_moving_tolerance_ = config.yaw_moving_tolerance;
        transform_timeout_ = config.timeout;
    }

	// 初始化局部规划器
    void PathFollowLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
		// 初始化private_nh私有句柄
        ros::NodeHandle private_nh("~/" + name);
		// 发布zm私有别名下的全局路径
        // global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		// 初始化tf_
        tf_ = tf;
		// 初始化代价地图
        costmap_ros_ = costmap_ros;
		// 初始化global_node公有句柄
		ros::NodeHandle global_node;
		// 发布预瞄点
		next_heading_pub_ = private_nh.advertise<visualization_msgs::Marker>("marker", 10);
		// 动态参数加载设置
        dsrv_ = new dynamic_reconfigure::Server<PathFollowLocalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<PathFollowLocalPlannerConfig>::CallbackType cb = boost::bind(&PathFollowLocalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
        ROS_INFO("PathFollowLocalPlanner initialized");
    }

	// 判断到达目标
    bool PathFollowLocalPlanner::isGoalReached()
    {
		// 到达目标返回状态为finished
        return (state_ == Finished);
    }
	
	// 设置局部规划路径
	bool PathFollowLocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& global_plan)
	{
		// 清空类里的局部路径
		global_plan_.clear();
		// Make our copy of the global plan
		// 将全局规划复制给类里的局部路径规划
		global_plan_ = global_plan;

		ROS_INFO_STREAM("Global plan size: " << global_plan_.size());
        ROS_INFO("Got Plan.");
		// 设置当前预瞄点的初值为0
		curr_heading_index_ = 0;
		// 设置下个预瞄点的初值为0
		next_heading_index_ = 0;
		// 全局路径点的最大索引
		path_index_ = global_plan_.size() - 1;
		state_ = RotatingToStart;
		return true;
	}

	// 计算速度
	bool PathFollowLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;

		cmd_vel.angular.x = 0.0; 
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

		// Set the default return value as false
		bool ret = false;
		// 如果没有获取到位置 报错
		if(!costmap_ros_->getRobotPose(robot_pose_))
		{
			ROS_ERROR("path_executer: cannot get robot pose");
			return false;
		}
		// We need to compute the next heading point from the global plan
		// 计算前进的预瞄点索引
		computeNextHeadingIndex();
		// 根据状态机选择进入不同状态函数
		switch(state_)
		{
			case RotatingToStart:
			   ret = rotateToStart(cmd_vel);
			   break;
		    case Moving:
			   
			   ret = move(cmd_vel);
			   break;
			// 中间拐点运动方式
			case MovingToKeyPoint:
				
				ret = moveToKeyPoint(cmd_vel);
				break;
		    case RotatingToGoal:
			   std::cout<<"+++++++++++++++++ RotatingToGoal ++++++++++++++++++++"<<std::endl;
			   ret = rotateToGoal(cmd_vel);
			   break;
		    default:
			   return true;
		}

		return ret;
	}

	void PathFollowLocalPlanner::publishNextHeading(bool show)
	{
		const geometry_msgs::PoseStamped& next_pose = global_plan_[next_heading_index_];

		visualization_msgs::Marker marker;
		marker.id = 0;
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id= next_pose.header.frame_id;
		marker.ns = "waypoints";
		marker.type = visualization_msgs::Marker::CYLINDER;

		if(show)
		{
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose = next_pose.pose;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.2;
			marker.color.a = 0.5;
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.0;
		}
		else
		{
			marker.action = visualization_msgs::Marker::DELETE;
		}
		next_heading_pub_.publish(marker);
	}
	bool PathFollowLocalPlanner::rotateToStart(geometry_msgs::Twist& cmd_vel)
	{
		geometry_msgs::PoseStamped rotate_goal;

		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			// 如果local_costmap_params.yaml中global_frame为map，则下面变换：map->map不起作用；global_frame为odom，则下面变换：map->odom
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
			tf2::doTransform(global_plan_[next_heading_index_], rotate_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}

		double rotation = calculateAngleDifference(robot_pose_, rotate_goal);

		if(fabs(rotation) < yaw_moving_tolerance_)
		{
			state_ = Moving;
			return true;
		}

		cmd_vel.angular.z = calRotationVel(rotation);
		return true;
	}

	// 向前移动行驶
	bool PathFollowLocalPlanner::move(geometry_msgs::Twist& cmd_vel)
	{
		publishNextHeading();
		geometry_msgs::PoseStamped move_goal;
		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], move_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}
		// 如果是拐点
		if (judgmentIsKeyPoint(global_plan_))
		{
			std::cout<<"+++++++++++++Moving ---> MovingToKeyPoint++++++++++++"<<std::endl;
			state_ = MovingToKeyPoint;
			return true;
		}

		double rotation = calculateAngleDifference(robot_pose_, move_goal);

		if(fabs(rotation) <= yaw_moving_tolerance_)
		{
			// The robot has rotated to its next heading pose
			cmd_vel.angular.z = 0.0;
		}
		else
		{
			cmd_vel.angular.z = calRotationVel(rotation);
		}
		cmd_vel.linear.x = calLinearVel();
		// We are approaching the goal position, slow down
		if(next_heading_index_ == (int)global_plan_.size() - 1)
		{
			// The distance from the robot's current pose to the next heading pose
			double distance_to_next_heading = linearDistance(robot_pose_.pose.position, move_goal.pose.position);
			// Reached the goal, now we can stop and rotate the robot to the goal position
			if(fabs(distance_to_next_heading) <= xy_tolerance_)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				state_ = RotatingToGoal;
				return true;
			}
		}
		return true;
	}

	bool PathFollowLocalPlanner::moveToKeyPoint(geometry_msgs::Twist& cmd_vel)
	{
		next_heading_index_ = next_heading_index_temp_;
		publishNextHeading();
		geometry_msgs::PoseStamped move_goal;
		ros::Time now = ros::Time::now();
		global_plan_[next_heading_index_].header.stamp = now;

		try
		{
			geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[next_heading_index_].header.frame_id, now, ros::Duration(transform_timeout_));
      		tf2::doTransform(global_plan_[next_heading_index_], move_goal, trans);
		}
		catch(tf2::LookupException& ex)
		{
			ROS_ERROR("Lookup Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ConnectivityException& ex)
		{
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf2::ExtrapolationException& ex)
		{
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return false;
		}
		double distance = linearDistance(robot_pose_.pose.position, key_point_);
		// 拐点距离小于阈值
		if (distance < 0.1)
		{
			state_ = RotatingToStart;
			cmd_vel.angular.z = 0;
			cmd_vel.linear.x = 0;
			return true;
		}

		double rotation = calculateAngleDifference(robot_pose_, move_goal);

		if(fabs(rotation) <= yaw_moving_tolerance_)
		{
			// The robot has rotated to its next heading pose
			cmd_vel.angular.z = 0.0;
		}
		else
		{
			cmd_vel.angular.z = calRotationVel(rotation);
		}
		cmd_vel.linear.x = calLinearVel();
		// We are approaching the goal position, slow down
		if(next_heading_index_ == (int)global_plan_.size() - 1)
		{
			// The distance from the robot's current pose to the next heading pose
			double distance_to_next_heading = linearDistance(robot_pose_.pose.position, move_goal.pose.position);
			// Reached the goal, now we can stop and rotate the robot to the goal position
			if(fabs(distance_to_next_heading) <= xy_tolerance_)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				state_ = RotatingToGoal;
				return true;
			}
		}
		return true;
	}

	bool PathFollowLocalPlanner::rotateToGoal(geometry_msgs::Twist& cmd_vel)
	{
		ros::Time now = ros::Time::now();
    	global_plan_[next_heading_index_].header.stamp = now;

		geometry_msgs::PoseStamped rotate_goal;

		try
		{
			tf_->transform(global_plan_[path_index_], rotate_goal, map_frame_); 
		}
		catch(tf2::TransformException& ex)
		{
			ROS_ERROR("Transform Error: %s\n", ex.what());
			return false;
		}
		double rotation = angles::shortest_angular_distance(tf2::getYaw(robot_pose_.pose.orientation), tf2::getYaw(rotate_goal.pose.orientation));
		if(fabs(rotation) <= yaw_tolerance_)
		{
			state_ = Finished;
			ROS_INFO("Goal reached");
			return true;
		}

		cmd_vel.angular.z = calRotationVel(rotation);

		return true;
	}

	// 计算下个预瞄点的索引
	void PathFollowLocalPlanner::computeNextHeadingIndex()
	{
		geometry_msgs::PoseStamped next_heading_pose;

		for(unsigned int i = curr_heading_index_; i < global_plan_.size(); ++i)
		{
			ros::Time now = ros::Time::now();
			global_plan_[i].header.stamp = now;

			try
			{
				geometry_msgs::TransformStamped trans = tf_->lookupTransform(robot_pose_.header.frame_id, global_plan_[i].header.frame_id, now, ros::Duration(transform_timeout_));
      			tf2::doTransform(global_plan_[i], next_heading_pose, trans);
			}
			catch(tf2::LookupException& ex)
			{
				ROS_ERROR("Lookup Error: %s\n", ex.what());
				return;
			}
			catch(tf2::ConnectivityException& ex)
			{
				ROS_ERROR("Connectivity Error: %s\n", ex.what());
				return;
			}
			catch(tf2::ExtrapolationException& ex)
			{
				ROS_ERROR("Extrapolation Error: %s\n", ex.what());
				return;
			}
			// 如果距离小于设定预瞄点距离 则直到循环至大于设定预瞄点距离 然后将索引设置下个预瞄点的索引
			next_heading_index_ = i;
			// 计算map坐标系下 当前位置和预瞄点的位置距离
			double dist = linearDistance(robot_pose_.pose.position, next_heading_pose.pose.position);
			// 如果距离大于设定预瞄点距离 就跳出
			if(dist > heading_lookahead_)
			{
				break;
			}
			curr_heading_index_ = i;
		}
	}
	
	double PathFollowLocalPlanner::calLinearVel()
	{
		double vel = 0.0;
	    double straight_dist = linearDistance(robot_pose_.pose.position, key_point_);
		vel = straight_dist;

		if(vel > linear_vel_.max_vel)
		{
			vel = linear_vel_.max_vel;
		}
		else if (vel < linear_vel_.min_vel)
		{
			vel = linear_vel_.min_vel;
		}

		return vel;
	}

	double PathFollowLocalPlanner::calRotationVel(double rotation)
	{
		double vel = 0.0;
		if( fabs(rotation) > rotation_vel_.max_vel)
		{
			vel = std::copysign(rotation_vel_.max_vel, rotation);
		}
		else if ( fabs(rotation) < rotation_vel_.min_vel )
		{
			vel = std::copysign(rotation_vel_.min_vel, rotation);
		}
		else
		{
			vel = rotation;
		}
		return vel;
	}

	double PathFollowLocalPlanner::linearDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
	{
		return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
	}

	bool PathFollowLocalPlanner::judgmentIsKeyPoint(std::vector<geometry_msgs::PoseStamped> global_plan)
	{
		if (next_heading_index_ < path_index_)
		{
			// 拐点
			geometry_msgs::PoseStamped Key_point = global_plan[next_heading_index_];
			// 拐点前一个点
			geometry_msgs::PoseStamped Key_point_last = global_plan[next_heading_index_ - 1];
			// 拐点后一个点
			geometry_msgs::PoseStamped Key_point_next = global_plan[next_heading_index_ + 1];
			// 前一个点和拐点连线的斜率角度
			double last_angle = ::atan2(Key_point.pose.position.y-Key_point_last.pose.position.y, 
			Key_point.pose.position.x-Key_point_last.pose.position.x);
			// 拐点和后一个点连线的斜率角度
			double next_angle = ::atan2(Key_point_next.pose.position.y-Key_point.pose.position.y,
				Key_point_next.pose.position.x-Key_point.pose.position.x);

			double delta_angle= std::fabs(next_angle - last_angle);
			if (delta_angle > (PI/6))
			{
				key_point_ = Key_point.pose.position;
				next_heading_index_temp_ = next_heading_index_;
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	
	// p1表示当前位置 p2表示预瞄点位置 
	double PathFollowLocalPlanner::calculateAngleDifference(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
	{
		double x = p2.pose.position.x - p1.pose.position.x;
		double y = p2.pose.position.y - p1.pose.position.y;
		double rotation = angles::shortest_angular_distance(tf::getYaw(p1.pose.orientation), std::atan2(y,x));
		return rotation;
	}

}
