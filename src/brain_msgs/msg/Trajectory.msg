uint8 LANE_KEEPING=0
uint8 CHANGE_LEFT=1
uint8 CHANGE_RIGHT=2
uint8 STOP=3
uint8 PARKING=4

Header header

bool valid
bool desired
bool on_ego
uint8 behavior
brain_msgs/SpatialTrajectory spatial_trajectory
brain_msgs/TemporalTrajectory temporal_trajectory
brain_msgs/TrajectoryBoundary left_boundary
brain_msgs/TrajectoryBoundary right_boundary
brain_msgs/TrajectoryPoint[] point
