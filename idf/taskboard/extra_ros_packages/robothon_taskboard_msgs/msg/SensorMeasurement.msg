uint8 SENSOR_MEASUREMENT_TYPE_UNKNOWN = 0
uint8 SENSOR_MEASUREMENT_TYPE_BOOL = 1
uint8 SENSOR_MEASUREMENT_TYPE_ANALOG = 2
uint8 SENSOR_MEASUREMENT_TYPE_VECTOR3 = 3
uint8 SENSOR_MEASUREMENT_TYPE_INTEGER = 4
uint8 SENSOR_MEASUREMENT_TYPE_EMPTY = 5

uint8 type

# To emulate a union, we use all fields as vectors and only one is filled
bool[<=1] bool_value
float32[<=1] analog_value
geometry_msgs/Vector3[<=1] vector3_value
int64[<=1] integer_value
