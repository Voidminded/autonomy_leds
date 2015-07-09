/* ROS */
#include "ros.h"
#include "autonomy_leds_msgs/Command.h"
// #include "autonomy_leds_msgs/LED.h"

/* AVR */
#include "Atmega32u4Hardware.h"
extern "C"
{
    #include <util/delay.h>
    #include <LUFA/Drivers/USB/USB.h>
    #include <avr/io.h>
    #include "light_apa102.h"
}

// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

/* CONSTANTS */
#define LED_PIN PC7
#define MAX_MSG_SIZE 8

/* LED Memory */
uint8_t led_counter = 0;
uint16_t* ros_buffer_ptr = 0;
uint8_t ros_buffer_size = 0;

/* Other variables */
char log_str[MAX_MSG_SIZE];

/* ROS */
void set_cb(const autonomy_leds_msgs::Command& cmd_msg);
// void clear_cb(const std_msgs::Empty& msg);
// void set_led_cb(const autonomy_leds_msgs::LED& led_msg);

ros::NodeHandle nh;
ros::Subscriber<autonomy_leds_msgs::Command> set_sub("leds/set", &set_cb);
// ros::Subscriber<autonomy_leds_msgs::LED> set_led_sub("leds/set_led", &set_led_cb);

int get_free_ram () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

inline void ack_led()
{
  for (led_counter = 0; led_counter < 6; led_counter++)
  {
    PORTC ^= (1 << LED_PIN);
    _delay_ms(100);
  }
}

/* ROS Callbacks */

void set_cb(const autonomy_leds_msgs::Command& cmd_msg)
{
  switch (cmd_msg.flag)
  {
    case autonomy_leds_msgs::Command::FLAG_SET_ALL:
    {
      ros_buffer_ptr = cmd_msg.colors_vec;
      ros_buffer_size = cmd_msg.colors_vec_length;
      apa102_setleds_packed(ros_buffer_ptr, ros_buffer_size);
      break;
    }
    case autonomy_leds_msgs::Command::FLAG_CLEAR:
    {
      // Clear everything, it should work even w/o a buffer
      apa102_setleds_packed(0, 255);
      ros_buffer_size = 0;
      break;  
    }
    case autonomy_leds_msgs::Command::FLAG_SHIFTLEFT:
    {
      if (ros_buffer_size == 0) break;
      uint16_t buffer = ros_buffer_ptr[0];
      for (led_counter = 0; led_counter < ros_buffer_size - 1; led_counter ++)
      {
        ros_buffer_ptr[led_counter] = ros_buffer_ptr[led_counter + 1];
      }
      ros_buffer_ptr[led_counter] = buffer;
      apa102_setleds_packed(ros_buffer_ptr, ros_buffer_size);
      break; 
    }
    case autonomy_leds_msgs::Command::FLAG_SHIFTRIGHT:
    {
      if (ros_buffer_size == 0) break;
      uint16_t buffer = ros_buffer_ptr[ros_buffer_size - 1];
      for (led_counter = ros_buffer_size - 1; led_counter > 0; led_counter--)
      {
        ros_buffer_ptr[led_counter] = ros_buffer_ptr[led_counter - 1];
      }
      ros_buffer_ptr[led_counter] = buffer;
      apa102_setleds_packed(ros_buffer_ptr, ros_buffer_size);
      break; 
    }
  }
  
  snprintf(log_str, MAX_MSG_SIZE, "%d", get_free_ram());
  nh.loginfo(log_str);
}

int main()
{
  /* IO */
  DDRC |= (1 << LED_PIN);

  nh.initNode();
  nh.subscribe(set_sub);
  // nh.subscribe(set_led_sub);

  ack_led();

  // Wait for Server side to start
  while (!nh.connected())
  {
    nh.spinOnce();
    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
    _delay_ms(10);
  }

  ack_led();
  
  // Publish some debug information
  snprintf(log_str, MAX_MSG_SIZE, "%s", GIT_VERSION);
  nh.loginfo(log_str);
  snprintf(log_str, MAX_MSG_SIZE, "%d", get_free_ram());
  nh.loginfo(log_str);

  // 50hz loop
  while(1)
  {
    nh.spinOnce();
    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
  }

  return 0;
}
