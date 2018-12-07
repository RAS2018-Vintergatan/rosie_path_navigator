#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <rosie_path_navigator/PathNavigationService.h>
#include <math.h>
#include <tf/transform_listener.h>

boost::shared_ptr<geometry_msgs::PoseStamped> targetPose_ptr;
boost::shared_ptr<nav_msgs::Odometry> lastOdom_ptr;

int currentPathPoint = -1;
int pathLength = 0;
boost::shared_ptr<nav_msgs::Path> pathArray_ptr;
int lastId = -1;

rosie_path_navigator::PathNavigationService::Request request;

boost::shared_ptr<tf::TransformListener> odom_tfl_ptr;
boost::shared_ptr<tf::TransformListener> target_tfl_ptr;

boost::shared_ptr<tf::StampedTransform> current_odom_tf_ptr;
boost::shared_ptr<tf::StampedTransform> target_pose_tf_ptr;

/*
 * Returns the angle between -3.14159 and +3.14159
 */
float capAngle(const float& angleIn){
	float angleOut = angleIn;
	while(angleOut < 0)
			angleOut += 6.28319f;
	while(angleOut > 6.28319f)
		angleOut -= 6.28319f;
	if(angleOut > 3.14159){
		angleOut = (angleOut-6.28319);
	}
	return angleOut;
}

void progressPath(){
	float targetWorldX = (*target_pose_tf_ptr).getOrigin().x() + (*targetPose_ptr).pose.position.x;
	float targetWorldY = (*target_pose_tf_ptr).getOrigin().y() + (*targetPose_ptr).pose.position.y;
	float rosieWorldX = (*current_odom_tf_ptr).getOrigin().x() + (*lastOdom_ptr).pose.pose.position.x;
	float rosieWorldY = (*current_odom_tf_ptr).getOrigin().y() + (*lastOdom_ptr).pose.pose.position.y;
	float deltaX = targetWorldX - rosieWorldX;
	float deltaY = targetWorldY - rosieWorldY;
	float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
	
	tf::Quaternion rosieQuaternion = (*current_odom_tf_ptr).getRotation();
	tf::Matrix3x3 m_w(rosieQuaternion);
	double roll_w, pitch_w, yaw_w;
	m_w.getRPY(roll_w, pitch_w, yaw_w);

	rosieQuaternion = tf::Quaternion((*lastOdom_ptr).pose.pose.orientation.x,
									(*lastOdom_ptr).pose.pose.orientation.y,
									(*lastOdom_ptr).pose.pose.orientation.z,
									(*lastOdom_ptr).pose.pose.orientation.w);
	tf::Matrix3x3 m(rosieQuaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw += yaw_w;

	tf::Quaternion targetQuaternion = (*target_pose_tf_ptr).getRotation();
	tf::Matrix3x3 m_t_w(targetQuaternion);
	double roll_t_w, pitch_t_w, yaw_t_w;
	m_t_w.getRPY(roll_t_w, pitch_t_w, yaw_t_w);

	targetQuaternion = tf::Quaternion((*targetPose_ptr).pose.orientation.x,
													(*targetPose_ptr).pose.orientation.y,
													(*targetPose_ptr).pose.orientation.z,
													(*targetPose_ptr).pose.orientation.w);

	tf::Matrix3x3 m_t(targetQuaternion);
	double roll_t, pitch_t, yaw_t;
	m_t.getRPY(roll_t, pitch_t, yaw_t);
	yaw_t += yaw_t_w;
	
	float deltaAnglePosition = capAngle(yaw - atan2(deltaY, deltaX));
	float deltaAnglePose = capAngle(yaw_t-yaw);

	if(distance < 0.1){
		//if(deltaAnglePose > -0.15 && deltaAnglePose < 0.15){
			if(currentPathPoint < pathLength-1){
				++currentPathPoint;
				targetPose_ptr->header.seq++;
				targetPose_ptr->header.stamp = ros::Time::now();
				targetPose_ptr->header.frame_id = "world";
				targetPose_ptr->pose.position.x = pathArray_ptr->poses[currentPathPoint].pose.position.x;
				targetPose_ptr->pose.position.y = pathArray_ptr->poses[currentPathPoint].pose.position.y;
				targetPose_ptr->pose.orientation.z = pathArray_ptr->poses[currentPathPoint].pose.orientation.z;
				targetPose_ptr->pose.orientation.w = pathArray_ptr->poses[currentPathPoint].pose.orientation.w;
			}else{
				targetPose_ptr->pose.position.x = lastOdom_ptr->pose.pose.position.x;
				targetPose_ptr->pose.position.y = lastOdom_ptr->pose.pose.position.y;
				targetPose_ptr->pose.orientation.z = lastOdom_ptr->pose.pose.orientation.z;
				targetPose_ptr->pose.orientation.w = lastOdom_ptr->pose.pose.orientation.w;
			}
		//}
	}
}

void currentPoseCallback(const nav_msgs::Odometry& msg){
	try{
		(*odom_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
		(*odom_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *current_odom_tf_ptr);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	*lastOdom_ptr = msg;
}

bool pathCallback(rosie_path_navigator::PathNavigationService::Request &req, rosie_path_navigator::PathNavigationService::Response &res){	
	ROS_ERROR("NAVIGATOR --- pathID: %d", req.id);
	res.response = 1;
	if(req.id == lastId){
		return true;
	}
	pathLength = req.path.poses.size();
	currentPathPoint = 0;
	*pathArray_ptr = req.path;
	lastId = req.id;
	if(pathLength > 0){
		targetPose_ptr->header.frame_id = "world";
		targetPose_ptr->pose = pathArray_ptr->poses[currentPathPoint].pose;
	}
	return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_path_navigator");
	
    ros::NodeHandle n;

	odom_tfl_ptr.reset(new tf::TransformListener);
	target_tfl_ptr.reset(new tf::TransformListener);

	current_odom_tf_ptr.reset(new tf::StampedTransform);
	target_pose_tf_ptr.reset(new tf::StampedTransform);

	targetPose_ptr.reset(new geometry_msgs::PoseStamped);
	lastOdom_ptr.reset(new nav_msgs::Odometry);
	pathArray_ptr.reset(new nav_msgs::Path);

	targetPose_ptr->header.seq++;
	targetPose_ptr->header.stamp = ros::Time::now();
	targetPose_ptr->header.frame_id = "world";

    ros::Publisher targetPose_pub = n.advertise<geometry_msgs::PoseStamped>("/rosie_pose_goal",1);
    ros::Subscriber currentPose_sub = n.subscribe("/odom", 1, currentPoseCallback);
    //ros::Subscriber path_sub = n.subscribe("/rosie_path", 1, pathCallback);

    ros::ServiceServer pathService = n.advertiseService<rosie_path_navigator::PathNavigationService::Request, rosie_path_navigator::PathNavigationService::Response>("/rosie_path_service", pathCallback);

    ros::Rate loop_rate(2);

    while(ros::ok()){
		progressPath();
		targetPose_pub.publish(*targetPose_ptr);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
