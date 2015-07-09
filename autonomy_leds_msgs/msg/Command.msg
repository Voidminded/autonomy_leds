uint8 FLAG_SET_ALL = 0
uint8 FLAG_CLEAR = 1
uint8 FLAG_SHIFTLEFT = 2
uint8 FLAG_SHIFTRIGHT = 3
uint8 FLAG_INVERT = 4

# 1-5-5-5
# x-b-g-r
# xbbbbbgggggrrrrr (16bits)
# conversion on u-controller to bbbbb000ggggg000rrrrr000 (32bits)
uint16[] colors_vec
uint8 flag
