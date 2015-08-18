#!/usr/bin/env python
import rospy
from autonomy_leds_msgs.msg import Keyframe, Command
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA
import time, copy

num_leds = 100.0
max_range = 10.0

class LEDRangeNode:
    def range_cb(self, data):
        rospy.loginfo("Got range %s" % (data.range, ))
        self.keyframe_msg.pattern_repeat = round(num_leds / max_range * data.range)
        self.display_pub.publish(self.keyframe_msg)
        time.sleep(0.1)

    def spin(self):
        rospy.spin()

    def __init__(self):
        rospy.init_node('led_range_node', anonymous = True)

        self.display_pub = rospy.Publisher('leds/display', Keyframe, queue_size=1)
        rospy.Subscriber('lidar/range', Range, self.range_cb, queue_size=1)

        self.keyframe_msg = Keyframe()
        self.keyframe_msg.color_pattern.append(ColorRGBA(0.0, 1.0, 0.0, 1.0))
        self.keyframe_msg.start_index = 0
        self.keyframe_msg.duration = 1000.0

        time.sleep(0.5)
        rospy.loginfo("Starting LED range display ...")


    
if __name__ == '__main__':
    led_range_node = LEDRangeNode()
    try:
        led_range_node.spin()
    except rospy.ROSInterruptException:
        pass