#!/usr/bin/env python
import rospy
from autonomy_leds_msgs.msg import Command
from std_msgs.msg import Empty

import time

num_leds = 100

def main():
    set_pub = rospy.Publisher('leds/set', Command, queue_size=10)
    
    rospy.init_node('led_tester', anonymous = True)
    #rate = rospy.Rate(1)
    
    command_msg = Command()
    
    rospy.loginfo("High freq test ...")
    for i in range(0, 20):
        led = 0b0111110000000000
        command_msg.colors_vec = [led] * num_leds
        command_msg.flag = 0
        time.sleep(0.2)
        set_pub.publish(command_msg)

        led = 0b0000000000011111
        command_msg.colors_vec = [led] * num_leds
        command_msg.flag = 0
        time.sleep(0.2)
        set_pub.publish(command_msg)
    
    rospy.loginfo("Clearing ...")
    command_msg.flag = 1
    command_msg.colors_vec = []
    set_pub.publish(command_msg)
    
    # rospy.loginfo("Ignore test ...")
    # bgra_vec_msg.colors_vec = []
    # for i in range(0, num_leds):
    #     led = BGRAInt8()
    #     led.r = 0
    #     led.g = 100
    #     led.b = 0
    #     led.a = i % 2        
    #     bgra_vec_msg.colors_vec.append(led)
    # set_pub.publish(bgra_vec_msg)

    # time.sleep(0.5)
    # rospy.loginfo("Clear test ...")
    # clear_pub.publish(empty_msg)

    # rospy.loginfo("Single LED test")
    # single_led = LED()
    # single_led.color.r = 255;
    # single_led.color.g = 0;
    # single_led.color.b = 0;
    # for i in range(0, num_leds):
    #     clear_pub.publish(empty_msg)
    #     time.sleep(0.01)
    #     single_led.index = i;
    #     set_led_pub.publish(single_led)
    #     time.sleep(0.2);    

    # rospy.loginfo("Shift right test ...")
    # for i in range(0, num_leds):
    #     time.sleep(0.1)
    #     sr_pub.publish(empty_msg)

    # rospy.loginfo("Shift left test ...")
    # for i in range(0, num_leds):
    #     time.sleep(0.1)
    #     sl_pub.publish(empty_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass