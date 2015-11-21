#!/bin/bash
SELFDIR="$(dirname $0)"
HEXFILE=$SELFDIR/../../../../devel/share/autonomy_leds_avr/firmware/leds_firmware.hex
echo "Flashing $HEXFILE"
echo "MD5: $(md5sum $HEXFILE)"
avrdude -P /dev/ttyACM0 -v -p m32u4 -c avr109 -b 57600 -D -U flash:w:$HEXFILE:i
