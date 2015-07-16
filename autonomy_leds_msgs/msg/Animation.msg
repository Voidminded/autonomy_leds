uint8 TIMING_FUNCTION_LINEAR = 0
uint8 TIMING_FUNCTION_EASE_IN = 1
uint8 TIMING_FUNCTION_EASE_OUT = 2
uint8 TIMING_FUNCTION_EASE_INOUT = 3

# Vector of Keyframes
autonomy_leds_msgs/Keyframe[] keyframes

# Number of iterations, 0 is infinite
uint32 iteration_count

# If set to false, there will be no interpolation between Keyframes 
bool smooth_transition

# The duration of each transition (in seconds)
float64 transition_duration

# Timing function to use (Liear, ease in, ease out, ease in and out)
uint8 timing_function