 #include "ros/ros.h"



 void chatterCallback()
 {
   ROS_INFO("HEY");
 }

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "listener");
 	ros::NodeHandle n;
 	ros::Subscriber sub = n.subscribe("/gazebo/default/selection", 1000, chatterCallback);
 	ros::spin();
 	return 0;
 }