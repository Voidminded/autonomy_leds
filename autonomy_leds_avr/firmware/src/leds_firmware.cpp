/* ROS */
#include "ros.h"
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
char log_str[MAX_MSG_SIZE];

/* ROS */
void set_cb(const autonomy_leds_msgs::RGBAVector& rgba_vec);  // Forward dec
ros::NodeHandle nh;
ros::Subscriber<autonomy_leds_msgs::RGBAVector> set_sub("leds/set", &set_cb);

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

void ack_led()
{
  for (led_counter = 0; led_counter < 6; led_counter++)
  {
    toggle_led(); 
    _delay_ms(100);
  }
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

  apa102_setleds(led_strip, max_leds);
}

int main()
{
  /* IO */
  init_io();
  init_led_strip();

  /* Variables */
  uint32_t lasttime = 0UL;

  nh.initNode();
  nh.subscribe(set_sub);

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
  snprintf(log_str, MAX_MSG_SIZE, "Autonomy LED Fimware (%s)", GIT_VERSION);
  nh.loginfo(log_str);
  
  // 50hz loop
  while(1)
  {
    if(avr_time_now() - lasttime > 20)
    {
      lasttime = avr_time_now();
    }
    nh.spinOnce();
    // LUFA functions that need to be called frequently to keep USB alive
    CDC_Device_USBTask(&Atmega32u4Hardware::VirtualSerial_CDC_Interface);
    USB_USBTask();
  }

  return 0;
}
