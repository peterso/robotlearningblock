uint8 TASK_STEP_TYPE_UNKNOWN = 0
uint8 TASK_STEP_TYPE_EQUAL = 1
uint8 TASK_STEP_TYPE_EQUAL_RANDOM = 2
uint8 TASK_STEP_TYPE_GREATER_EQUAL = 3
uint8 TASK_STEP_TYPE_TRACE_SHAPE = 4

string sensor_name
uint8 type
SensorMeasurement target
float32 tolerance
string clue_trigger_name
builtin_interfaces/Time clue_timeout