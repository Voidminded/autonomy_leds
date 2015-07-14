## Background

TBA

## Installation

```bash
$ sudo cp 52-led-strip.rules /etc/udev/rules.d && sudo service udev restart
```

Please also add the current user to the `dialout` group.

```bash
# to check which groups the current user is a member of
$ groups
# to add the current user to the dialout group
# you need to logout/login again for the changes to take effect
$ sudo adduser $USER dialout
```

## Misc

Check the attributes of the ttyACM0 device:

    $ udevadm info -a -p $(udevadm info -q path -n /dev/ttyACM0)

https://groups.google.com/forum/#!msg/lufa-support/CP9cy2bc8yo/kBqsOu-RBeMJ

