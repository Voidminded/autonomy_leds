# This array defines the RGB values for each individual LED
# r,g,b should be in [0..1] range, a is ignored
std_msgs/ColorRGBA[] color_pattern

# Number of times the color_pattern should be repeated
# Totoal number of affected LEDs: color_patten.size() * pattern_repeat
uint16 pattern_repeat

# The start index (LED) of this frame
uint16 start_index

# The duration to show this frame
float64 duration
