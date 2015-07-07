#!/bin/bash
dfu-programmer atmega32u4 erase && dfu-programmer atmega32u4 flash ../../../devel/share/autonomy_leds_avr/firmware/leds_firmware.hex
