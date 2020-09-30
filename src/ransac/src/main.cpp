#include "ransac.hpp"

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "ransac");
    RANSAC ransac;
	ransac.Run();

	return 0;
}