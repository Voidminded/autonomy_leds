/* ROS */
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "autonomy_leds_msgs/RGBAVector.h"

/* AVR */
#include "Atmega32u4Hardware.h"
extern "C"
{
    #include <util/delay.h>
    #include <LUFA/Drivers/USB/USB.h>
    #include <avr/io.h>
    #include <avr/interrupt.h>
    #include "light_apa102.h"
}

// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

/* CONSTANTS */
#define NUM_LEDS 10
#define LED PC7
#define MAX_MSG_SIZE 64

/* LED Memory */
uint16_t led_counter = 0;
struct cRGB led_strip[NUM_LEDS];

/* Other variables */
char str[MAX_MSG_SIZE];
std_msgs::String str_msg;

/* ROS */
ros::NodeHandle nh;  

int init_io()
{
    DDRC |= (1 << LED);
}

int init_led_strip()
{
    for (led_counter = 0; led_counter < NUM_LEDS; led_counter++)
    {
        led_strip[led_counter].r = 0;
        led_strip[led_counter].g = 0;
        led_strip[led_counter].b = 0;
    }
    apa102_setleds(led_strip, NUM_LEDS);
}

/* ROS Callbacks */
void toggle_led()
{
    PORTC ^= (1 << LED);
}

void set_cb(const autonomy_leds_msgs::RGBAVector& rgba_vec)
{
  uint16_t max_leds = (rgba_vec.colors_vec_length < NUM_LEDS) ? 
    rgba_vec.colors_vec_length: NUM_LEDS;

  for (led_counter = 0; led_counter < max_leds; led_counter++)
  {
    led_strip[led_counter].r = rgba_vec.colors_vec[led_counter].r;
    led_strip[led_counter].g = rgba_vec.colors_vec[led_counter].g;
    led_strip[led_counter].b = rgba_vec.colors_vec[led_counter].b;    
  }

  // snprintf(str, MAX_MSG_SIZE, "r: %d g:%d b: %d a: %d", 
  //   rgba_vec.colors_vec[0].r, 
  //   rgba_vec.colors_vec[0].g,
  //   rgba_vec.colors_vec[0].b,
  //   rgba_vec.colors_vec[0].a
  //   );

  // str_msg.data = str;
  // debug_pub.publish(&str_msg);  
  apa102_setleds(led_strip, max_leds);
}

ros::Publisher debug_pub("leds/debug", &str_msg);
ros::Subscriber<autonomy_leds_msgs::RGBAVector> set_sub("leds/set", &set_cb);

int main()
{
  /* IO */
  init_io();
  init_led_strip();

  /* Variables */
  uint32_t lasttime = 0UL;  

  nh.initNode();
  nh.advertise(debug_pub);
  nh.subscribe(set_sub);

  long int counter = 0;

  toggle_led(); _delay_ms(500); toggle_led();

  while(1)
  {
    // Send the message every second
    if(avr_time_now() - lasttime > 1000)
    {
      snprintf(str, MAX_MSG_SIZE, "Mani %ld", counter++);
      str_msg.data = str;
      debug_pub.publish(&str_msg);
      lasttime = avr_time_now();
    }
    nh.spinOnce();

    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
  }

return 0;
}
