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

}  // namespace util

AnimationEngine::AnimationEngine(ros::NodeHandle& nh, const uint16_t num_leds)
  : num_leds_(num_leds),
    anim_ptr_(autonomy_leds_msgs::AnimationConstPtr()),
    state_(STATE_READY),
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
  while (index < kf.start_index && clear) leds_msg_.colors_vec[index++] = 0;
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
  kf_index_ = 0.0;
  kf_start_time_ = ros::Time(0.0);
  transition_start_time_ = ros::Time(0.0);
  iteration_counter_ = 0;
}

void AnimationEngine::Restart()
{
  state_ = STATE_READY;
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

  ROS_INFO_STREAM("In step() " << iteration_counter_);

  switch (state_)
  {
  case STATE_READY:
  {
    ROS_INFO("Ready ...");
    kf_index_ = 0;
    state_ = STATE_DISPLAY_KF;
    break;
  }
  case STATE_DISPLAY_KF:
  {
    ROS_INFO("Display KF");
    DisplayFrame(anim_ptr_->keyframes[kf_index_], anim_ptr_->keyframes.size() > 0);
    kf_start_time_ = t_now;
    state_ = STATE_WAIT_KF;
    break;
  }
  case STATE_WAIT_KF:
  {
    ROS_INFO("Wait KF");
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
    ROS_INFO("Transition");
    double t_progress = (t_now - transition_start_time_).toSec() / anim_ptr_->transition_duration;
    // TODO: fabs()??
    if (anim_ptr_->smooth_transition && t_progress >= 0.0 && t_progress <= 1.0)
    {
      const autonomy_leds_msgs::Keyframe& from_kf = anim_ptr_->keyframes[(kf_index_ - 1) % anim_ptr_->keyframes.size()];
      const autonomy_leds_msgs::Keyframe& to_kf = anim_ptr_->keyframes[kf_index_];
      transition_kf_ = from_kf;
      transition_kf_.start_index = from_kf.start_index * (1.0 - t_progress) + to_kf.start_index * (t_progress);

      if (from_kf.color_pattern.size() == to_kf.color_pattern.size())
      {
        for (uint32_t i = 0; i < from_kf.color_pattern.size(); i++)
        {
          transition_kf_.color_pattern[i].r = from_kf.color_pattern[i].r * (1.0 - t_progress) +
              to_kf.color_pattern[i].r * t_progress;
          transition_kf_.color_pattern[i].g = from_kf.color_pattern[i].g * (1.0 - t_progress) +
              to_kf.color_pattern[i].g * t_progress;
          transition_kf_.color_pattern[i].b = from_kf.color_pattern[i].b * (1.0 - t_progress) +
              to_kf.color_pattern[i].b * t_progress;
        }
        transition_kf_.pattern_repeat = from_kf.pattern_repeat * (1.0 - t_progress) + to_kf.pattern_repeat * t_progress;
      }
      else
      {
        ROS_WARN_THROTTLE(1, " Keyframes have color patterns of different size, can not interpolate colors");
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

