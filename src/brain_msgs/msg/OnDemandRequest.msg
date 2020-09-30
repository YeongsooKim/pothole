# *******************************************************************
# This message was defined by Wonteak Lim (lwt1849@gmail.com)
# 2019.09.19 Initial version
# *******************************************************************
Header header

# Request for OnDemand Service
uint8   request_type
WGS84   target_point
float64  target_heading

uint8 RQ_NONE = 0
uint8 RQ_DRIVE = 1
uint8 RQ_STOP = 2
uint8 RQ_EMERGENCY = 3
