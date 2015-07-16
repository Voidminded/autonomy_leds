#include <cmath>

#include "autonomy_leds_tools/animation_engine.h"
namespace autonomy_leds
{

namespace util
{

uint16_t RGBAToPackedBGR(const std_msgs::ColorRGBA &color)
{
  uint16_t pbgr = 0;
  pbgr |= ((util::cc2int8(color.b) & 0b11111000) << 7);
  pbgr |= ((util::cc2int8(color.g) & 0b11111000) << 2);
  pbgr |= ((util::cc2int8(color.r) & 0b11111000) >> 3);
  return pbgr;
}

double interpolate(double t, const double start, const double diff, const uint8_t itype)
{
  switch (itype)
  {
  case (autonomy_leds_msgs::Animation::TIMING_FUNCTION_LINEAR):
  {
    return t * diff + start;
    break;
  }
  case (autonomy_leds_msgs::Animation::TIMING_FUNCTION_EASE_IN):
  {
    return diff * t * t * t + start;
  }
  case (autonomy_leds_msgs::Animation::TIMING_FUNCTION_EASE_OUT):
  {
    t -= 1.0;
    return diff * (t * t * t + 1.0) + start;
  }
  case (autonomy_leds_msgs::Animation::TIMING_FUNCTION_EASE_INOUT):
  {
    t *= 2.0;
    if (t < 1.0) return diff / 2.0 * t * t * t + start;
    t -= 2.0;
    return diff / 2.0 * (t * t * t + 2.0) + start;
  }
  }
}

}  // namespace util

AnimationEngine::AnimationEngine(ros::NodeHandle& nh, const uint16_t num_leds)
  : num_leds_(num_leds),
    anim_ptr_(autonomy_leds_msgs::AnimationConstPtr()),
    state_(STATE_READY),
    prev_state_(STATE_NUM),
    kf_index_(0),
    kf_start_time_(0.0),
    iteration_counter_(0.0),
    nh_(nh),
    led_pub_(nh_.advertise<autonomy_leds_msgs::Command>("leds/set", 30))
{
  leds_msg_.colors_vec.resize(num_leds_, 0);
  clear_msg_.flag = autonomy_leds_msgs::Command::FLAG_CLEAR;
}



void AnimationEngine::ClearLEDS()
{
  // Clear the internal buffer
  leds_msg_.colors_vec.resize(num_leds_, 0);

  // When clearing there is no need to send a full vector of values
  led_pub_.publish(clear_msg_);
}

void AnimationEngine::DisplayFrame(const autonomy_leds_msgs::Keyframe &kf, const bool clear)
{
  leds_msg_.flag = autonomy_leds_msgs::Command::FLAG_SET_ALL;
  uint32_t index = 0;
  while (index < kf.start_index && index < num_leds_ && clear) leds_msg_.colors_vec[index++] = 0;
  index = kf.start_index;
  for (uint32_t i = 0; i < kf.pattern_repeat && index < num_leds_; i++)
  {
    for (uint32_t j = 0; j < kf.color_pattern.size() && index < num_leds_; j++)
    {
      leds_msg_.colors_vec[index] = util::RGBAToPackedBGR(kf.color_pattern[j]);
      index++;
    }
  }
  while (index < num_leds_ && clear) leds_msg_.colors_vec[index++] = 0;

  led_pub_.publish(leds_msg_);
}

void AnimationEngine::ShowSingleFrame(const autonomy_leds_msgs::KeyframeConstPtr &frame_ptr)
{
  Reset();
  autonomy_leds_msgs::AnimationPtr single_frame =
      autonomy_leds_msgs::AnimationPtr(new autonomy_leds_msgs::Animation);
  single_frame->keyframes.push_back(*frame_ptr);
  single_frame->iteration_count = 1;
  single_frame->smooth_transition = false;
  anim_ptr_ = single_frame;
}

void AnimationEngine::SetAnimation(const autonomy_leds_msgs::AnimationConstPtr &anim_ptr)
{
  Reset();
  anim_ptr_ = anim_ptr;
}

void AnimationEngine::Reset()
{
  anim_ptr_ = autonomy_leds_msgs::AnimationConstPtr();
  state_ = STATE_READY;
  prev_state_ = STATE_NUM;
  kf_index_ = 0.0;
  kf_start_time_ = ros::Time(0.0);
  transition_start_time_ = ros::Time(0.0);
  iteration_counter_ = 0;
}

void AnimationEngine::Restart()
{
  state_ = STATE_READY;
  prev_state_ = STATE_NUM;
  kf_index_ = 0.0;
  kf_start_time_ = ros::Time(0.0);
  transition_start_time_ = ros::Time(0.0);
  iteration_counter_ = 0;
}

bool AnimationEngine::Step()
{
  if (!anim_ptr_ || !anim_ptr_->keyframes.size()) return false;
  if ((state_ == STATE_FINISHED) || (state_ == STATE_PAUSED)) return false;

  ros::Time t_now = ros::Time::now();
  const bool is_state_changed = (prev_state_ != state_);
  prev_state_ = state_;
  uint32_t prev_kf_index = kf_index_;

  switch (state_)
  {
  case STATE_READY:
  {
    kf_index_ = 0;
    state_ = STATE_DISPLAY_KF;
    break;
  }
  case STATE_DISPLAY_KF:
  {
    DisplayFrame(anim_ptr_->keyframes[kf_index_], anim_ptr_->keyframes.size() > 0);
    kf_start_time_ = t_now;
    state_ = STATE_WAIT_KF;
    break;
  }
  case STATE_WAIT_KF:
  {
    if ((t_now - kf_start_time_).toSec() > anim_ptr_->keyframes[kf_index_].duration)
    {
      kf_index_++;
      if (kf_index_ == anim_ptr_->keyframes.size())
      {
        iteration_counter_++;
        if (anim_ptr_->iteration_count && iteration_counter_ >= anim_ptr_->iteration_count)
        {
          ClearLEDS();
          state_ = STATE_FINISHED;
        }
        else
        {
          // Loop
          kf_index_ = 0;
          transition_start_time_ = t_now;
          state_ = STATE_TRANSITION;
        }
      }
      else
      {
        transition_start_time_ = t_now;
        state_ = STATE_TRANSITION;
      }
    }
    break;
  }
  case STATE_TRANSITION:
  {
    const uint32_t prev_kf_index = (kf_index_ + anim_ptr_->keyframes.size() - 1) % anim_ptr_->keyframes.size();
    const autonomy_leds_msgs::Keyframe& from_kf = anim_ptr_->keyframes[prev_kf_index];
    const autonomy_leds_msgs::Keyframe& to_kf = anim_ptr_->keyframes[kf_index_];

    if (is_state_changed)
    {
      diff_index = to_kf.start_index - from_kf.start_index;
      if (from_kf.color_pattern.size() == to_kf.color_pattern.size())
      {
        transition_kf_.color_pattern.resize(from_kf.color_pattern.size());
        diff_color.resize(from_kf.color_pattern.size());
        for (uint32_t i = 0; i < from_kf.color_pattern.size(); i++)
        {
          diff_color[i].r = to_kf.color_pattern[i].r - from_kf.color_pattern[i].r;
          diff_color[i].g = to_kf.color_pattern[i].g - from_kf.color_pattern[i].g;
          diff_color[i].b = to_kf.color_pattern[i].b - from_kf.color_pattern[i].b;
        }
        diff_repeat = to_kf.pattern_repeat - from_kf.pattern_repeat;
      }
      else
      {
        ROS_WARN("Keyframes have color patterns of different size, can not interpolate colors or repeat");
      }
    }
    const double t_progress = (t_now - transition_start_time_).toSec() / anim_ptr_->transition_duration;
    // TODO: fabs()??
    if (anim_ptr_->smooth_transition && t_progress >= 0.0 && t_progress <= 1.0)
    {
      transition_kf_.start_index = round(util::interpolate(t_progress, from_kf.start_index, diff_index, anim_ptr_->timing_function));

      if (from_kf.color_pattern.size() == to_kf.color_pattern.size())
      {
        for (uint32_t i = 0; i < from_kf.color_pattern.size(); i++)
        {
          transition_kf_.color_pattern[i].r = util::interpolate(t_progress, from_kf.color_pattern[i].r,
                                                                diff_color[i].r, anim_ptr_->timing_function);
          transition_kf_.color_pattern[i].g = util::interpolate(t_progress, from_kf.color_pattern[i].g,
                                                                diff_color[i].g, anim_ptr_->timing_function);
          transition_kf_.color_pattern[i].b = util::interpolate(t_progress, from_kf.color_pattern[i].b,
                                                                diff_color[i].b, anim_ptr_->timing_function);
        }
        transition_kf_.pattern_repeat = round(util::interpolate(t_progress, from_kf.pattern_repeat, diff_repeat, anim_ptr_->timing_function));
      }

      DisplayFrame(transition_kf_, true);
    }
    else
    {
      state_ = STATE_DISPLAY_KF;
    }
    break;
  }

  }

}

}  // namespace autonomy_leds

