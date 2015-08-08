#ifndef AUTONOMY_LEDS_TOOLS_H
#define AUTONOMY_LEDS_TOOLS_H

#include <stdint.h>
#include <algorithm>

#include "autonomy_leds_msgs/Command.h"
#include "autonomy_leds_msgs/Keyframe.h"
#include "autonomy_leds_msgs/Animation.h"

#include <ros/ros.h>
#include <ros/time.h>

namespace autonomy_leds
{

namespace util
{

// color channel to int8
// [0.0 -> 1.0] -> [0..255] (8 bit)
inline uint8_t cc2int8(const float& c) {
  return static_cast<uint8_t> (255.0f * std::max(0.0f, std::min(c, 1.0f)));
}

template <typename T> inline T clamp (T x, T a, T b)
{
    return ((x) > (a) ? ((x) < (b) ? (x) : (b)) : (a));
}

// t is normalized wrt to duration so it is in [0..1] range (t = t_actual / duration)
double interpolate(double t, const double start, const double diff, const uint8_t itype);

uint16_t RGBAToPackedBGR(const std_msgs::ColorRGBA& color);

}  // namespace util


class AnimationEngine
{
public:
  enum ENGINE_STATES
  {
    STATE_READY = 0,
    STATE_DISPLAY_KF = 1,
    STATE_WAIT_KF = 2,
    STATE_TRANSITION = 3,
    STATE_PAUSED = 4,
    STATE_FINISHED = 5,
    STATE_NUM
  };

protected:
  uint32_t num_leds_;
  autonomy_leds_msgs::AnimationConstPtr anim_ptr_;
  ENGINE_STATES state_;
  ENGINE_STATES prev_state_;
  uint16_t kf_index_;
  ros::Time kf_start_time_;
  ros::Time transition_start_time_;
  uint32_t iteration_counter_;

  ros::NodeHandle nh_;
  ros::Publisher led_pub_;

  void DisplayFrame(const autonomy_leds_msgs::Keyframe &kf, const bool clear = false);

private:
  autonomy_leds_msgs::Command clear_msg_;
  autonomy_leds_msgs::Command leds_msg_;

  autonomy_leds_msgs::Keyframe transition_kf_;
  // Since diff values can be negative, we can't piggy back on KeyFrame message
  int32_t diff_index;
  int32_t diff_repeat;
  autonomy_leds_msgs::Keyframe::_color_pattern_type diff_color;


public:
  AnimationEngine(ros::NodeHandle& nh, const uint16_t num_leds);

  inline uint16_t GetNumLEDs() const {return num_leds_;}

  void ClearLEDS();
  void Reset();
  void Restart();
  void ShowSingleFrame(const autonomy_leds_msgs::KeyframeConstPtr& frame_ptr);
  void SetAnimation(const autonomy_leds_msgs::AnimationConstPtr& anim_ptr);
  bool Step();
};

}  // namespace autonomy_leds

#endif
