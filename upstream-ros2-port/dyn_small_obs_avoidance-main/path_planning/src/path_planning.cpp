// ROS 2 topic interfaces for the original dynamic obstacle planner.
#include <rclcpp/rclcpp.hpp>
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap_ros/conversions.h>
// #include <octomap/octomap.h>
#include "visualization_msgs/msg/marker.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <iostream>
#include <fstream>

#include <path_searching/kinodynamic_astar.h>
#include <std_msgs/msg/int64.hpp>

using std::unique_ptr;
// Node clock also honors the ROS 2 use_sim_time parameter.
rclcpp::Node::SharedPtr ros_node;
// Declear some global variables
Eigen::Vector3d cur_pos;

//ROS publishers
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub;
rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub;
rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr Bspline_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kino_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr kino_path_pub;

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pos_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vel_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr acc_pub;
rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr path_size_pub;

ofstream outfile;

unique_ptr<KinodynamicAstar> kino_path_finder_;

class planner {
public:
	void init_start(void)
	{
		if(!set_start)
			std::cout << "Initialized" << std::endl;
			// path_size_int64.data = 0;
		set_start = true;		
	}

	void setStart(double x, double y, double z, double vx, double vy, double vz,nav_msgs::msg::Odometry odm)
	{
		start_pt(0) = x;
		start_pt(1) = y;
		start_pt(2) = z;

		start_vel(0) = vx;
		start_vel(1) = vy;
		start_vel(1) = vz;

		uav_odom = odm;
	}

	void setlocalpose(geometry_msgs::msg::PoseStamped pose)
	{
		local_pose = pose;
	}

	void setGoal(double x, double y, double z)
	{
		if(prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != z || prev_start[0] != start_pt(0)|| prev_start[1] != start_pt(1)|| prev_start[2] != start_pt(2))
		{
			goal_pt(0) = x;
			goal_pt(1) = y;
			goal_pt(2) = z;

			std::cout << "Goal point set to: " << x << " " << y << " " << z << std::endl;

			if(set_start)
				plan();
		}
	}

	void update_timeindex(int time_index)
	{
		replan_time_index = time_index;
		RCLCPP_INFO(rclcpp::get_logger("path_planning"), "replan time index IS %d",replan_time_index);
	}

	// Constructor
	planner(void)
	{
		path_size_int64.data = 0;
	}
	// Destructor
	~planner()
	{
	}

	void replan(void)
	{
		if(sqrt(pow((goal_pt(0)-start_pt(0)),2)+pow((goal_pt(1)-start_pt(1)),2)+pow((goal_pt(2)-start_pt(2)),2)) < 0.3)
		{
			return;
		}
		if(path_size_int64.data !=0 && set_start)
		{
			
				for (std::size_t idx = 0; idx < path_size_int64.data; idx = idx + 40)
				{
					if(!replan_flag)
					{
						rclcpp::Time t1 = ros_node->now();
						replan_flag = !kino_path_finder_->isSafe(kino_nav_path.poses[idx].pose.position.x,kino_nav_path.poses[idx].pose.position.y,kino_nav_path.poses[idx].pose.position.z);
						rclcpp::Time t2 = ros_node->now();
						// ROS_INFO("CHECK one position time : %f",(t2-t1).toSec());
						// std::cout << "Replan!" << std::endl;
					}
					else
					{	break;
					}
				}
				if(replan_flag)
					plan();
				else{}
					// std::cout << "Replanning not required" << std::endl;
		}
	}

	void plan(void)
	{
				//kinodynamic path searching

				rclcpp::Time t1 = ros_node->now();

				kino_path_finder_->reset();
				
				// Eigen::Vector3d start_vel(0,0,0);
				Eigen::Vector3d start_acc(0,0,0);
				Eigen::Vector3d goal_vel(0,0,0);

				if(firstplan_flag == true || replan_time_index==-1 )// || (replan_time_index==(path_size_int64.data-1))
				{
				prev_start[0] = start_pt(0);
				prev_start[1] = start_pt(1);
				prev_start[2] = start_pt(2);

				int status = kino_path_finder_->search(start_pt, start_vel, start_acc, goal_pt, goal_vel, true);
				std::cout << "[kino replan]: startpoint kinodynamic search" << std::endl;

				if (status == KinodynamicAstar::NO_PATH) {
					// std::cout << "[kino replan]: startpoint kinodynamic search fail!" << std::endl;

					// retry searching with discontinuous initial state
					kino_path_finder_->reset();
					status = kino_path_finder_->search(start_pt, start_vel, start_acc, goal_pt, goal_vel, false);

					if (status == KinodynamicAstar::NO_PATH) {
					std::cout << "[kino replan]: Can't find path." << std::endl;
					return;
					} else {
					std::cout << "[kino replan]: retry search success." << std::endl;
					firstplan_flag=false;
					}

				} else {
					std::cout << "[kino replan]: kinodynamic search success." << std::endl;
					firstplan_flag=false;
				}					
				}else{

				Eigen::Vector3d replan_startpt;
				Eigen::Vector3d replan_startpt_vel(0,0,0);

				if(fabs(last_replan_time_index - replan_time_index)<10)
				{
					return;
				}

				int used_time_index = replan_time_index + 3;
				last_replan_time_index = replan_time_index;

				//fix overflow bug
				if(used_time_index > path_size_int64.data-5)
				{
					used_time_index = replan_time_index;
				}

				replan_startpt(0) = kino_nav_path.poses[used_time_index].pose.position.x;
				replan_startpt(1) = kino_nav_path.poses[used_time_index].pose.position.y;
				replan_startpt(2) = kino_nav_path.poses[used_time_index].pose.position.z;

				int vel_horizon = 2;
				if(used_time_index + vel_horizon >= path_size_int64.data-1)
				{
				replan_startpt_vel(0) = (kino_nav_path.poses[used_time_index].pose.position.x - kino_nav_path.poses[used_time_index-vel_horizon].pose.position.x)/(0.01*vel_horizon);
				replan_startpt_vel(1) = (kino_nav_path.poses[used_time_index].pose.position.y - kino_nav_path.poses[used_time_index-vel_horizon].pose.position.y)/(0.01*vel_horizon);
				replan_startpt_vel(2) = (kino_nav_path.poses[used_time_index].pose.position.z - kino_nav_path.poses[used_time_index-vel_horizon].pose.position.z)/(0.01*vel_horizon);

				}else if(used_time_index - vel_horizon < 0)
				{
				replan_startpt_vel(0) = (kino_nav_path.poses[used_time_index+vel_horizon].pose.position.x - kino_nav_path.poses[used_time_index].pose.position.x)/(0.01*vel_horizon);
				replan_startpt_vel(1) = (kino_nav_path.poses[used_time_index+vel_horizon].pose.position.y - kino_nav_path.poses[used_time_index].pose.position.y)/(0.01*vel_horizon);
				replan_startpt_vel(2) = (kino_nav_path.poses[used_time_index+vel_horizon].pose.position.z - kino_nav_path.poses[used_time_index].pose.position.z)/(0.01*vel_horizon);

				}else{
				replan_startpt_vel(0) = (kino_nav_path.poses[used_time_index+vel_horizon].pose.position.x - kino_nav_path.poses[used_time_index-vel_horizon].pose.position.x)/(0.01*2*vel_horizon);
				replan_startpt_vel(1) = (kino_nav_path.poses[used_time_index+vel_horizon].pose.position.y - kino_nav_path.poses[used_time_index-vel_horizon].pose.position.y)/(0.01*2*vel_horizon);
				replan_startpt_vel(2) = (kino_nav_path.poses[used_time_index+vel_horizon].pose.position.z - kino_nav_path.poses[used_time_index-vel_horizon].pose.position.z)/(0.01*2*vel_horizon);

				}
				
				prev_start[0] = replan_startpt(0);
				prev_start[1] = replan_startpt(1);
				prev_start[2] = replan_startpt(2);

				RCLCPP_INFO(rclcpp::get_logger("path_planning"), "StartPOINT IS %f,%f,%f",replan_startpt(0),replan_startpt(1),replan_startpt(2));

				RCLCPP_INFO(rclcpp::get_logger("path_planning"), "Before KINODYNAMIC SEARCH TIME: %f",(ros_node->now() - t1).seconds());

				int status = kino_path_finder_->search(replan_startpt, replan_startpt_vel, start_acc, goal_pt, goal_vel, true);

				if (status == KinodynamicAstar::NO_PATH) {
					// std::cout << "[kino replan]: replan kinodynamic search fail!" << std::endl;

					// retry searching with discontinuous initial state
					kino_path_finder_->reset();
					status = kino_path_finder_->search(replan_startpt, replan_startpt_vel, start_acc, goal_pt, goal_vel, false);

					if (status == KinodynamicAstar::NO_PATH) {
					std::cout << "[kino replan]: Can't find path." << std::endl;
					return;
					} else {
					std::cout << "[kino replan]: retry search success." << std::endl;
					}

				} else {
					std::cout << "[kino replan]: kinodynamic search success." << std::endl;
				}
				}

				std::vector<Eigen::Vector3d> kino_path;
				kino_path = kino_path_finder_->getKinoTraj(0.01);

				double t_search = (ros_node->now() - t1).seconds();

				outfile<< t_search <<endl; 

				RCLCPP_INFO(rclcpp::get_logger("path_planning"), "KINODYNAMIC SEARCH TIME: %f,search_count = %d,search average time = %f",t_search,kino_path_finder_->search_count,kino_path_finder_->search_time_amount/kino_path_finder_->search_count);

				//compute transition from px4 local position to camera_init
				Eigen::Quaterniond quat_odom(uav_odom.pose.pose.orientation.w, uav_odom.pose.pose.orientation.x, uav_odom.pose.pose.orientation.y, uav_odom.pose.pose.orientation.z);
				Eigen::Quaterniond quat_px4(local_pose.pose.orientation.w, local_pose.pose.orientation.x, local_pose.pose.orientation.y, local_pose.pose.orientation.z);
				Eigen::Matrix3d rotation_odom;
    			rotation_odom = quat_odom.matrix();
				Eigen::Matrix3d rotation_px4;
    			rotation_px4 = quat_px4.matrix();
				Eigen::Matrix3d rotation_px42odom;
				rotation_px42odom = rotation_odom.transpose() * rotation_px4;

				Eigen::Vector3d T_odom,T_local,T_px42odom,T_odom2px4;
				T_odom << uav_odom.pose.pose.position.x , uav_odom.pose.pose.position.y , uav_odom.pose.pose.position.z;
				T_local << local_pose.pose.position.x,  local_pose.pose.position.y,  local_pose.pose.position.z;
				T_px42odom = rotation_odom.transpose() * (T_local - T_odom);

				T_odom2px4 = -rotation_px42odom.transpose() * T_px42odom;
				Eigen::Matrix3d rotation_odom2px4;
				rotation_odom2px4 = rotation_px42odom.transpose();

				visualization_msgs::msg::Marker kino_marker;
				kino_marker.action = visualization_msgs::msg::Marker::DELETEALL;
				kino_pub->publish(kino_marker);

				kino_nav_path.poses.clear();
				kino_nav_path_px4.poses.clear();
				geometry_msgs::msg::PoseStamped this_pose_stamped;
				geometry_msgs::msg::PoseStamped this_pose_stamped_px4;

				for(int i = 0; i<kino_path.size();i++)
				{
					kino_marker.header.frame_id = "camera_init";
					kino_marker.header.stamp = rclcpp::Time();
					kino_marker.ns = "kino_path";
					kino_marker.id = i;
					kino_marker.type = visualization_msgs::msg::Marker::CUBE;
					kino_marker.action = visualization_msgs::msg::Marker::ADD;
					kino_marker.pose.position.x = kino_path[i](0);
					kino_marker.pose.position.y = kino_path[i](1);
					kino_marker.pose.position.z = kino_path[i](2);

					// ROS_INFO("i = %d,pos = %f,%f,%f",i,kino_path[i](0),kino_path[i](1),kino_path[i](2));

					kino_marker.scale.x = 0.15;
					kino_marker.scale.y = 0.15;
					kino_marker.scale.z = 0.15;
					kino_marker.color.a = 1.0;
					kino_marker.color.r = 0.0;
					kino_marker.color.g = 0.0;
					kino_marker.color.b = 1.0;
					kino_pub->publish(kino_marker);

					Eigen::Vector3d kino_pathpose_px4 = rotation_px42odom * kino_path[i] + T_px42odom;

					this_pose_stamped.pose.position.x = kino_path[i](0);
					this_pose_stamped.pose.position.y = kino_path[i](1);
					this_pose_stamped.pose.position.z = kino_path[i](2);
					this_pose_stamped_px4.pose.position.x = kino_pathpose_px4(0);
					this_pose_stamped_px4.pose.position.y = kino_pathpose_px4(1);
					this_pose_stamped_px4.pose.position.z = kino_pathpose_px4(2);
					kino_nav_path.poses.push_back(this_pose_stamped);
					kino_nav_path_px4.poses.push_back(this_pose_stamped_px4);
				}

				kino_nav_path.header.frame_id = "camera_init";
				kino_nav_path.header.stamp = ros_node->now();
				kino_path_pub->publish(kino_nav_path);

				kino_nav_path_px4.header.frame_id = "camera_init";
				kino_nav_path_px4.header.stamp = ros_node->now();
				pos_pub->publish(kino_nav_path);
				// vel_pub.publish(minjerk_velocity);
				// acc_pub.publish(minjerk_accel);

    			path_size_int64.data = kino_path.size()+1;
				path_size_pub->publish(path_size_int64);

				double t_all = (ros_node->now() - t1).seconds();

				// ROS_INFO("KINODYNAMIC all TIME: %f",t_all);
				replan_flag = false;
	}
private:

	Eigen::Vector3d start_pt;
	Eigen::Vector3d goal_pt;
	Eigen::Vector3d start_vel;
	// Eigen::Vector3d current_setpt;
	// Eigen::Vector3d current_setpt_vel;

	double prev_start[3];

	nav_msgs::msg::Path kino_nav_path;
	std_msgs::msg::Int64 path_size_int64;
	nav_msgs::msg::Path kino_nav_path_px4;

	//odometry and local pose
	geometry_msgs::msg::PoseStamped local_pose;
	nav_msgs::msg::Odometry uav_odom;

	// goal state
	double prev_goal[3];

	bool replan_flag = false;

	// Flag for initialization
	bool set_start = false;

	int replan_time_index = -1;
	int last_replan_time_index = -1;
	bool firstplan_flag = true;
};

void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, planner* planner_ptr)
{
	pcl::PointCloud<pcl::PointXYZ> cloud_input;
  	pcl::fromROSMsg(*msg, (cloud_input));
	// ROS_INFO("FROM ROS TO PCLOUD SUCESS");


	//only input point clouds in 20 meters to reduce computation
  	rclcpp::Time t1 = ros_node->now();
	pcl::PointCloud<pcl::PointXYZ> cloud_cutoff;
	// ROS_INFO("FROM ROS TO PCLOUD Size is %d",int(cloud_input.size()));
	for (size_t i = 0; i < cloud_input.points.size(); i = i+2)
  	{
    if(fabs(cloud_input.points[i].x - cur_pos(0))>20 || fabs(cloud_input.points[i].y - cur_pos(1))>20 || fabs(cloud_input.points[i].z - cur_pos(2))>20)
	{
		continue;
	}else{
		cloud_cutoff.push_back(cloud_input.points[i]);
	}
  	}

	rclcpp::Time t2 = ros_node->now();
 	// ROS_INFO("Pointcloud CUTOFF used %f s",(t2-t1).toSec());
	
	kino_path_finder_->setKdtree(cloud_cutoff);
	planner_ptr->replan();
}

nav_msgs::msg::Odometry uav_odometry;
void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, planner* planner_ptr)
{
	// ROS_INFO("RECEIVED ODOMETRY"); 
	uav_odometry = *msg;
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,uav_odometry);
	planner_ptr->init_start();
	cur_pos(0) = msg->pose.pose.position.x;
	cur_pos(1) = msg->pose.pose.position.y;
	cur_pos(2) = msg->pose.pose.position.z;
}

void startCb(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg, planner* planner_ptr)
{
	planner_ptr->init_start();
}

void poseCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg, planner* planner_ptr)
{
	planner_ptr->setlocalpose(*msg);
}

void goalCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg, planner* planner_ptr)
{
	planner_ptr->setGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void timeindexCallBack(const std_msgs::msg::Int64::ConstSharedPtr &msg, planner* planner_ptr)
{
	std_msgs::msg::Int64 time_index_int64;
	int time_index=0;
	time_index_int64 = *msg;
    time_index = time_index_int64.data;
	planner_ptr->update_timeindex(time_index);
}

// Initialize ROS 2 interfaces and keep callbacks single threaded.
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	ros_node = std::make_shared<rclcpp::Node>("dyn_obs_planner");
	auto n = ros_node;

	planner planner_object;

    //kino astar
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(*n);
    kino_path_finder_->init();

    // outfile.open("/home/dji/kong_ws/RRT-andjerk_ws/test_time/searching_time.txt", ios::out | ios::trunc );

	// Sensor subscriptions accept both reliable and best-effort ROS 2 inputs.
	auto odom_sub = n->create_subscription<nav_msgs::msg::Odometry>("/mavros/odometry/out", rclcpp::SensorDataQoS().keep_last(1), [&planner_object](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCb(msg, &planner_object); });
	auto pointcloud_sub = n->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered", rclcpp::SensorDataQoS().keep_last(1), [&planner_object](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg, &planner_object); });

	auto pose_sub = n->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", rclcpp::SensorDataQoS().keep_last(100), [&planner_object](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { poseCb(msg, &planner_object); });

	auto goal_sub = n->create_subscription<geometry_msgs::msg::PoseStamped>("/goal", 10000, [&planner_object](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { goalCb(msg, &planner_object); });

	auto time_index_sub = n->create_subscription<std_msgs::msg::Int64>("/demo_node/trajectory_time_index", 1000, [&planner_object](std_msgs::msg::Int64::ConstSharedPtr msg) { timeindexCallBack(msg, &planner_object); });

	vis_pub = n->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
	traj_pub = n->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("waypoints", 1);

	kino_pub = n->create_publisher<visualization_msgs::msg::Marker>("kino_marker", 1);
	kino_path_pub = n->create_publisher<nav_msgs::msg::Path>("kino_path", 1);

	Bspline_pub = n->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("smooth_waypoints", 1);

	// Transient-local durability preserves the original ROS 1 latched topics.
	pos_pub = n->create_publisher<nav_msgs::msg::Path>("/search_node/trajectory_position", rclcpp::QoS(1).transient_local());
    vel_pub = n->create_publisher<nav_msgs::msg::Path>("/search_node/trajectory_velocity", rclcpp::QoS(1).transient_local());
    acc_pub = n->create_publisher<nav_msgs::msg::Path>("/search_node/trajectory_accel", rclcpp::QoS(1).transient_local());
    path_size_pub = n->create_publisher<std_msgs::msg::Int64>("/search_node/trajectory_path_size", 1);
	
	rclcpp::spin(n);
    // Release global ROS 2 handles before middleware static teardown.
    kino_path_finder_.reset();
    vis_pub.reset();
    traj_pub.reset();
    Bspline_pub.reset();
    kino_pub.reset();
    kino_path_pub.reset();
    pos_pub.reset();
    vel_pub.reset();
    acc_pub.reset();
    path_size_pub.reset();
    ros_node.reset();
	rclcpp::shutdown();

	return 0;
}
