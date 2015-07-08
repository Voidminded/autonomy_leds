/* ROS */
#include "ros.h"
#include "std_msgs/Empty.h"
#include "autonomy_leds_msgs/BGRVector.h"
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
autonomy_leds_msgs::BGRInt8* ros_buffer_ptr = 0;
uint8_t ros_buffer_size = 0;

/* Custom Write Function to LED based on apa102 */
#define nop() asm volatile(" nop \n\t")

inline void SPI_init(void) {
  apa102_DDRREG  |=  _BV(apa102_data);
  apa102_DDRREG  |=  _BV(apa102_clk);
  apa102_PORTREG &= ~_BV(apa102_clk);  // initial state of clk is low
}

// Assumed state before call: SCK- Low, MOSI- High
void SPI_write(uint8_t c) {
  uint8_t i;
  for (i=0; i<8 ;i++)
  {
    if (!(c&0x80)) {
      apa102_PORTREG &= ~_BV(apa102_data); // set data low
    } else {
      apa102_PORTREG |=  _BV(apa102_data); // set data high
    }     
  
  apa102_PORTREG |= (1<< apa102_clk); // SCK hi , data sampled here

  c<<=1;
  
  nop();  // Stretch clock
  nop();
  
  apa102_PORTREG &= ~_BV(apa102_clk); // clk low
  }
// State after call: SCK Low, Dat high
}

void inline apa102_setleds_ros(autonomy_leds_msgs::BGRInt8 *ledarray, uint16_t leds)
{
  uint16_t i;
  //uint8_t *rawarray=(uint8_t*)ledarray;
  SPI_init();
  
  SPI_write(0x00);  // Start Frame
  SPI_write(0x00);
  SPI_write(0x00);
  SPI_write(0x00);
 
  for (i = 0; i < leds; i++)
  {
    SPI_write(0xff);  // Maximum global brightness
    SPI_write(ledarray[i].b);
    SPI_write(ledarray[i].g);
    SPI_write(ledarray[i].r);
  }
  
  // End frame: 8+8*(leds >> 4) clock cycles    
  for (i=0; i<leds; i+=16)
  {
    SPI_write(0xff);  // 8 more clock cycles
  }
}

/* Other variables */
char log_str[MAX_MSG_SIZE];

/* ROS */
void set_cb(const autonomy_leds_msgs::BGRVector& bgr_vec);
void clear_cb(const std_msgs::Empty& msg);
// void set_led_cb(const autonomy_leds_msgs::LED& led_msg);
// void shift_left_cb(const std_msgs::Empty& msg);
// void shift_right_cb(const std_msgs::Empty& msg);

ros::NodeHandle nh;
ros::Subscriber<autonomy_leds_msgs::BGRVector> set_sub("leds/set", &set_cb);
ros::Subscriber<std_msgs::Empty> clear_sub("leds/clear", &clear_cb);
// ros::Subscriber<autonomy_leds_msgs::LED> set_led_sub("leds/set_led", &set_led_cb);
//ros::Subscriber<std_msgs::Empty> shift_left_sub("leds/shift_left", &shift_left_cb);
//ros::Subscriber<std_msgs::Empty> shift_right_sub("leds/shift_right", &shift_right_cb);


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

void set_cb(const autonomy_leds_msgs::BGRVector& bgr_vec)
{
  // Store the pointer to the last buffer
  if (ros_buffer_ptr != bgr_vec.colors_vec) ack_led();
  ros_buffer_ptr = bgr_vec.colors_vec;
  ros_buffer_size = bgr_vec.colors_vec_length;
  apa102_setleds_ros(ros_buffer_ptr, ros_buffer_size);
}

void clear_cb(const std_msgs::Empty& msg)
{
  if (ros_buffer_ptr == 0) return;
  for (led_counter = 0; led_counter < ros_buffer_size; led_counter++)
  {
      ros_buffer_ptr[led_counter].r = 0;
      ros_buffer_ptr[led_counter].g = 0;
      ros_buffer_ptr[led_counter].b = 0;
  }
  apa102_setleds_ros(ros_buffer_ptr, ros_buffer_size);
}

// void set_led_cb(const autonomy_leds_msgs::LED& led_msg)
// {
//   if (led_msg.index >= ros_buffer_size)
//   {
//     ack_led();
//     return;
//   }
//   ros_buffer_ptr[led_msg.index].r = led_msg.color.r;
//   ros_buffer_ptr[led_msg.index].g = led_msg.color.g;
//   ros_buffer_ptr[led_msg.index].b = led_msg.color.b;
//   apa102_setleds_ros(ros_buffer_ptr, ros_buffer_size);
// }

// void shift_left_cb(const std_msgs::Empty& msg)
// {
//   cRGB buffer = led_strip[0];
//   for (led_counter = 0; led_counter < NUM_LEDS - 1; led_counter ++)
//   {
//     led_strip[led_counter] = led_strip[led_counter + 1];
//   }
//   led_strip[led_counter] = buffer;
//   apa102_setleds(led_strip, NUM_LEDS);
// }

// void shift_right_cb(const std_msgs::Empty& msg)
// {
//   cRGB buffer = led_strip[NUM_LEDS - 1];
//   for (led_counter = NUM_LEDS - 1; led_counter > 0; led_counter--)
//   {
//     led_strip[led_counter] = led_strip[led_counter - 1];
//   }
//   led_strip[led_counter] = buffer;
//   apa102_setleds(led_strip, NUM_LEDS);
// }

int main()
{
  /* IO */
  DDRC |= (1 << LED_PIN);

  nh.initNode();
  nh.subscribe(set_sub);
  nh.subscribe(clear_sub);
  // nh.subscribe(set_led_sub);
  // nh.subscribe(shift_left_sub);
  // nh.subscribe(shift_right_sub);

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
  snprintf(log_str, MAX_MSG_SIZE, "%d %d", get_free_ram(), sizeof(autonomy_leds_msgs::BGRInt8));
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
