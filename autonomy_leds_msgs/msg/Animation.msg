uint8 TIMING_FUNCTION_LINEAR = 0
uint8 TIMING_FUNCTION_EASE_IN = 1
uint8 TIMING_FUNCTION_EASE_OUT = 2
uint8 TIMING_FUNCTION_EASE_INOUT = 3

autonomy_leds_msgs/Keyframe[] keyframes
# 0 infinite
uint32 iteration_count
bool smooth_transition
float64 transition_duration
uint8 timing_function
