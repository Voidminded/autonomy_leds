#!/usr/bin/env python
import rospy
from autonomy_leds_msgs.msg import RGBAInt8, RGBAVector
import time

num_leds = 10

def main():
    pub = rospy.Publisher('leds/set', RGBAVector, queue_size=10)
    rospy.init_node('led_tester', anonymous = True)
    #rate = rospy.Rate(1)
    
    rgba_vec_msg = RGBAVector()
    led = RGBAInt8()
    
    for i in range(0, 50):
        rospy.loginfo("Publishing")
        led.r = 0;
        led.g = 0;
        led.b = 200;
        led.a = 0;
        rgba_vec_msg.colors_vec = [led] * num_leds
        time.sleep(0.03)
        pub.publish(rgba_vec_msg)

        led.r = 200;
        led.g = 0;
        led.b = 0;
        led.a = 0;
        rgba_vec_msg.colors_vec = [led] * num_leds
        time.sleep(0.03)
        pub.publish(rgba_vec_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass