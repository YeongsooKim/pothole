# event type
uint8 TRAFFIC_SIGN=0
uint8 TRAFFIC_LIGHT=1
uint8 SPEED_LIMIT=2
uint8 SPEED_BUMP=3
uint8 STOP_LINE=4
uint8 PARKING_SPACE=5
uint8 SCHOOL_ZONE=6
uint8 PARKING_LOT=7

# traffic light value
uint8 RED=0
uint8 RED_LEFT=1
uint8 YELLOW=2
uint8 GREEN=3
uint8 GREEN_LEFT=4
uint8 UNKNOWN=5

int64 id
uint8 type
brain_msgs/EventArea area
float32 value
