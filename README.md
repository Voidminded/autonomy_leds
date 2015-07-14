# autonomy_leds

Firmware and tools to interface with autonomy_lab's LED strip driver.

## Packages

- autonomy_leds_avr: rosserial based firmware for the driver board
- autonomy_leds_msgs: Custom message definitions
- autonomy_leds_tools: High level tools, scripts, udev rules and animation API

## Compile instructions

Instructions on how to build the whole stack is provided in `autonomy_leds_avr/README.md` file. If you do not need to [cross] compile the firmware (i.e. you only need to use a pre-programmed board), follow the same instructions and put an empty `.CATKIN_IGNORE` file inside `autonomy_leds_avr` folder after cloning it. That will prevent the compilation of the firmware. 

## Running the driver

(One time only) Please install provided udev rules in `autonomy_leds_tools/udev` based on accompanying README file.

```bash
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

## Color encoding information

Please refer to `autonomy_leds_msgs/msg/Command.msg` file.

## Using the low-level API

### Setting individual LEDs

To set an individual LED, set the _index_ and _color_ in a message of type `autonomy_leds_msgs/LED` and publish it to `leds/set_led` topic.

### Changing the values of all LEDs

Messages published to `leds/set` topic set/manipulate the value of all LEDs in one go. The type of these messages are `autonomy_leds_msgs/Command`. The `flag` field of this message specifies the command:

- `flag = 0 (FLAG_SET_ALL)` will set the RGB value of each LED based on the values of the vector specified by `colors_vec` field. Each color is a 16 bit value. The encoding is described in _color encoding infomation_ section.
- `flag = 1 (FLAG_CLEAR)` clears the LED strip
- `flag = 2 (FLAG_SHIFTLEFT)` performs a circular shift left on the LED strip
- `flag = 3 (FLAG_SHIFTRIGHT)` performs a circular shift right on the LED strip
- `flag = 4 (FLAG_INVERT)` inverts the color of all LEDs
