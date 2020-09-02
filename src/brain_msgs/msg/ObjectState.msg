uint8 number_of_states            # number of states

bool x_validation                 # validation of x
float32 x                         # x position in vehicle coordinates [m]
bool y_validation                 # validation of y
float32 y                         # y position in vehicle coordinates [m]
bool velocity_x_validation        # validation of velocity in x-direction
float32 velocity_x                # velocity in vehicle coordinates [m/s]
bool velocity_y_validation        # validation of velocity in y-direction
float32 velocity_y                # velocity in vehicle coordinates [m/s]
bool acceleration_x_validation    # validation of acceleration in x-direction
float32 acceleration_x            # acceleration in vehicle coordinates [m/s^2]
bool acceleration_y_validation    # validation of acceleration in y-direction
float32 acceleration_y            # acceleration in vehicle coordinates [m/s^2]
bool orientation_validation       # validation of orientation
float32 orientation               # orientation in vehicle coordinates [rad]

bool s_validation                 # validation of s
float32 s                         # s position in curvilinear coordinates [m]
bool n_validation                 # validation of n
float32 n                         # y position in curvilinear coordinates [m]
bool velocity_s_validation        # validation of velocity in s-direction
float32 velocity_s                # velocity in curvilinear coordinates [m/s]
bool velocity_n_validation        # validation of velocity in n-direction
float32 velocity_n                # velocity in curvilinear coordinates [m/s]
bool acceleration_s_validation    # validation of acceleration in s-direction
float32 acceleration_s            # acceleration in curvilinear coordinates [m/s^2]
bool acceleration_n_validation    # validation of acceleration in n-direction
float32 acceleration_n            # acceleration in curvilinear coordinates [m/s^2]
bool orientation_sn_validation    # validation of orientation
float32 orientation_sn            # orientation in curvilinear coordinates [rad]
