#include "preprocess.hpp"

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "preprocess");
    Preprocess preprocess;
	preprocess.Run();

	return 0;
}