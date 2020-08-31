#include "pothole.hpp"

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pothole");
    Pothole pothole;

	ros::Rate delay(100);
	
	while(ros::ok()) {
		ros::spinOnce();
		delay.sleep();
	}
}