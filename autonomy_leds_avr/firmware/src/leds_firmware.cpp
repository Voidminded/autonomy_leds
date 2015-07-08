/* ROS */
#include "ros.h"
#include "std_msgs/Empty.h"

#include "autonomy_leds_msgs/BGRAVector.h"
#include "autonomy_leds_msgs/LED.h"

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
#define LED_PIN PC7
#define MAX_MSG_SIZE 64

/* LED Memory */
uint16_t led_counter = 0;
struct cRGB led_strip[NUM_LEDS];

/* Other variables */
char log_str[MAX_MSG_SIZE];

/* ROS */
void set_cb(const autonomy_leds_msgs::BGRAVector& bgr_vec);
void clear_cb(const std_msgs::Empty& msg);
void set_led_cb(const autonomy_leds_msgs::LED& led_msg);
void shift_left_cb(const std_msgs::Empty& msg);
void shift_right_cb(const std_msgs::Empty& msg);

ros::NodeHandle nh;
ros::Subscriber<autonomy_leds_msgs::BGRAVector> set_sub("leds/set", &set_cb);
ros::Subscriber<std_msgs::Empty> clear_sub("leds/clear", &clear_cb);
ros::Subscriber<std_msgs::Empty> shift_left_sub("leds/shift_left", &shift_left_cb);
ros::Subscriber<std_msgs::Empty> shift_right_sub("leds/shift_right", &shift_right_cb);
ros::Subscriber<autonomy_leds_msgs::LED> set_led_sub("leds/set_led", &set_led_cb);

void init_io()
{
    DDRC |= (1 << LED_PIN);
}

void init_led_strip()
{
    for (led_counter = 0; led_counter < NUM_LEDS; led_counter++)
    {
        led_strip[led_counter].r = 0;
        led_strip[led_counter].g = 0;
        led_strip[led_counter].b = 0;
    }
    apa102_setleds(led_strip, NUM_LEDS);
}

void toggle_led()
{
    PORTC ^= (1 << LED_PIN);
}

void ack_led()
{
  for (led_counter = 0; led_counter < 6; led_counter++)
  {
    toggle_led(); 
    _delay_ms(100);
  }
}

/* ROS Callbacks */

void set_cb(const autonomy_leds_msgs::BGRAVector& bgra_vec)
{
  uint16_t max_leds = (bgra_vec.colors_vec_length < NUM_LEDS) ? 
    bgra_vec.colors_vec_length: NUM_LEDS;

  for (led_counter = 0; led_counter < max_leds; led_counter++)
  {
    // Skip the pixel if a==0
    if (bgra_vec.colors_vec[led_counter].a == 0) continue;
    led_strip[led_counter].r = bgra_vec.colors_vec[led_counter].r;
    led_strip[led_counter].g = bgra_vec.colors_vec[led_counter].g;
    led_strip[led_counter].b = bgra_vec.colors_vec[led_counter].b; 
  }

  apa102_setleds(led_strip, max_leds);
}

void clear_cb(const std_msgs::Empty& msg)
{
  init_led_strip();
}

void set_led_cb(const autonomy_leds_msgs::LED& led_msg)
{
  if (led_msg.index >= NUM_LEDS) return;
  led_strip[led_msg.index].r = led_msg.color.r;
  led_strip[led_msg.index].g = led_msg.color.g;
  led_strip[led_msg.index].b = led_msg.color.b;
  apa102_setleds(led_strip, NUM_LEDS);
}

void shift_left_cb(const std_msgs::Empty& msg)
{
  cRGB buffer = led_strip[0];
  for (led_counter = 0; led_counter < NUM_LEDS - 1; led_counter ++)
  {
    led_strip[led_counter] = led_strip[led_counter + 1];
  }
  led_strip[led_counter] = buffer;
  apa102_setleds(led_strip, NUM_LEDS);
}

void shift_right_cb(const std_msgs::Empty& msg)
{
  cRGB buffer = led_strip[NUM_LEDS - 1];
  for (led_counter = NUM_LEDS - 1; led_counter > 0; led_counter--)
  {
    led_strip[led_counter] = led_strip[led_counter - 1];
  }
  led_strip[led_counter] = buffer;
  apa102_setleds(led_strip, NUM_LEDS);
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
  nh.subscribe(clear_sub);
  nh.subscribe(set_led_sub);
  nh.subscribe(shift_left_sub);
  nh.subscribe(shift_right_sub);

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
  snprintf(log_str, MAX_MSG_SIZE, "Autonomy LED Fimware started.");
  nh.loginfo(log_str);
  snprintf(log_str, MAX_MSG_SIZE, "LEDS: %d Ver: %s", NUM_LEDS, GIT_VERSION);  
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
