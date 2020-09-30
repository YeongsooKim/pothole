# Object classification
uint8 CLASSIFICATION_UNKNOWN=0
uint8 CLASSIFICATION_CAR=1
uint8 CLASSIFICATION_BIKE=2
uint8 CLASSIFICATION_MOTORCYCLE=3
uint8 CLASSIFICATION_TRUCK=4
uint8 CLASSIFICATION_PEDESTRIAN=5
uint8 CLASSIFICATION_OTHER_VEHICLE=6
uint8 TERMINATED=0
uint8 TENTATIVE=1
uint8 CONFIRMED=2
uint8 EGO_LANE=1
uint8 LEFT_LANE=2
uint8 RIGHT_LANE=3

# Object information
bool confirmed

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
bool motion_prediction_validation
float32 time_difference
brain_msgs/ObjectState[] state
brain_msgs/ObjectCovariance[] covariance
uint8[] sensor_types
uint8[] measurement_identification
