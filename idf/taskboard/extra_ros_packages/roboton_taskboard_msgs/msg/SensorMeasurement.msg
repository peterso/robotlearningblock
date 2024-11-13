uint8 SENSOR_MEASUREMENT_TYPE_UNKNOWN = 0
uint8 SENSOR_MEASUREMENT_TYPE_BOOL = 1
uint8 SENSOR_MEASUREMENT_TYPE_ANALOG = 2
uint8 SENSOR_MEASUREMENT_TYPE_VECTOR3 = 3

uint8 type

# To emulate a union, we use all fields as vectors and only one is filled
bool[] bool_value
float32[] analog_value
geometry_msgs/Vector3[] vector3_value
