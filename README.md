# autonomy_leds

Firmware and tools to interface with Autonomy Lab's LED strip driver.

## Packages

- autonomy_leds_avr: rosserial based firmware for the driver board
- autonomy_leds_msgs: Custom message definitions
- autonomy_leds_tools: High level tools, scripts, udev rules and animation API

## Compile instructions

Instructions on how to build the whole stack is provided in `autonomy_leds_avr/README.md` file. If you do not need to [cross] compile the firmware (i.e. you only need to use the pre-programmed board), follow the same instructions but put an empty `.CATKIN_IGNORE` file inside `autonomy_leds_avr` folder after cloning it. That will prevent catkin from [cross] compiling the firmware.

## Running the driver

(One time only) Please install provided udev rules in `autonomy_leds_tools/udev` based on the accompanying README file.

```bash
$ rosrun rosserial_python serial_node.py _port:=/dev/led_strip _baud:=115200
```

## Using the low-level API

### Setting individual LEDs

To set an individual LED, set the _index_ and _color_ of a message of type `autonomy_leds_msgs/LED` to your desired values, then publish it to `leds/set_led` topic.

### Changing the values of all LEDs

Messages published to `leds/set` topic set/manipulate the value of all LEDs in one go. These messages are of type `autonomy_leds_msgs/Command`. The `flag` field of this message specifies the desired action:

- `flag = 0 (FLAG_SET_ALL)` will set the RGB value of each LED based on the color specified by `colors_vec` vector. Each color is a 16 bit value. The encoding is described in _color encoding infomation_ section.
- `flag = 1 (FLAG_CLEAR)` clears the LED strip
- `flag = 2 (FLAG_SHIFTLEFT)` performs a circular shift left on all LEDS
- `flag = 3 (FLAG_SHIFTRIGHT)` performs a circular shift right on all LEDS
- `flag = 4 (FLAG_INVERT)` inverts the color of all LEDs

Check `autonomy_leds_tools/scripts/test.py` file for some demo material.

### Color encoding information

Please refer to `autonomy_leds_msgs/msg/Command.msg` file.

## Using the high-level Animation Engine

`autonomy_leds_tools` package provides a simple Keyframe based animation engine via `leds_animation_engine_node`. The format for Keyframe messages are defined in `autonomy_leds_msgs/msg/Keyframe.msg`:

```
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
```

Each Animation (defined in `autonomy_leds_msgs/msg/Keyframe.msg`), consists of a vector of Keyframes as well as duration and desired timing function of each transition. The engine will co

```
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
```

To display a single Keyframe, publish a message of type `autonomy_leds_msg/Keyframe` to `leds/display` topic. To display an Animation, construct a message of type `autonomy_leds_msg/Animation`, then publish it to `leds/animation` topic.

To see a demo of these APIs, you can check `autonomy_leds_tools/scripts/test_anim.py` script.

## Run Everything (driver + high level engine)

```bash
$ roslaunch autonomy_leds_tools engine.launch
```

## Hardware

### Pins Configuration
```
LED Str ->    CJMCU
VCC     --    5V
CI      --    D9
DI      --    D10
GND     --    GND
```
