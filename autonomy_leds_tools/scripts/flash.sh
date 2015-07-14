#!/bin/bash
SELFDIR="$(dirname $0)"
HEXFILE=$SELFDIR/../../../../devel/share/autonomy_leds_avr/firmware/leds_firmware.hex
echo "Flashing $HEXFILE"
echo "MD5: $(md5sum $HEXFILE)"
dfu-programmer atmega32u4 erase && dfu-programmer atmega32u4 flash $HEXFILE