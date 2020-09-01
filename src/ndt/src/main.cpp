#include "ndt.hpp"

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "ndt");
    NDT ndt;
	ndt.Run();

	return 0;
}