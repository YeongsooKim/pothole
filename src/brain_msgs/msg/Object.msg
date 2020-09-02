# Object classification
uint8 CLASSIFICATION_UNKNOWN=0
uint8 CLASSIFICATION_CAR=1
uint8 CLASSIFICATION_BIKE=2
uint8 CLASSIFICATION_MOTORCYCLE=3
uint8 CLASSIFICATION_TRUCK=4
uint8 CLASSIFICATION_PEDESTRIAN=5
uint8 CLASSIFICATION_OTHER_VEHICLE=6

# Object information
uint8 id
bool classification_validation
uint8 classification
uint8 lane_id

bool length_validation
float32 length 
bool width_validation
float32 width 
bool height_validation 
float32 height 
brain_msgs/ObjectState state
brain_msgs/ObjectCovariance covariance 
