#include <boost/bind.hpp>
#include <exception>
#include <string>

#include <ros/ros.h>

#include "autonomy_leds_tools/animation_engine.h"

class AnimationEngineNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber kf_sub_;
  ros::Subscriber anim_sub_;

  double update_rate_;  // hz
  autonomy_leds::AnimationEngine anim_engine_;

public:
  AnimationEngineNode(ros::NodeHandle& nh)
    : nh_(nh),
      private_nh_("~"),
      kf_sub_(nh_.subscribe("leds/display", 30, &AnimationEngineNode::DisplayFrameCallback, this)),
      anim_sub_(nh_.subscribe("leds/animation", 5, &AnimationEngineNode::AnimationCallback, this)),
      update_rate_(private_nh_.param<double>(std::string("update_rate"), 25.0)),
      anim_engine_(nh_, private_nh_.param<int>(std::string("num_leds"), 50))
  {
    ROS_WARN_STREAM("[ANIM] Number of LEDs: " << anim_engine_.GetNumLEDs() << " Update Rate: " << update_rate_ << " Hz");
  }

  void DisplayFrameCallback(const autonomy_leds_msgs::KeyframeConstPtr& kf_ptr)
  {
    ROS_DEBUG("[ANIM] Request to display a frame received ...");
    anim_engine_.ShowSingleFrame(kf_ptr);
  }

  void AnimationCallback(const autonomy_leds_msgs::AnimationConstPtr& anim_ptr)
  {
    ROS_DEBUG("[ANIM] Request to display an animation received ...");
    anim_engine_.SetAnimation(anim_ptr);
  }

  void Process()
  {
    anim_engine_.Step();
  }

  virtual void SpinOnce()
  {
    Process();
    ros::spinOnce();
  }

  virtual void Spin()
  {
    ros::Rate rate(update_rate_);
    while (ros::ok())
    {
      SpinOnce();
      // TODO: Graceful shutdown:
      // http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
      if (!rate.sleep())
      {
        ROS_WARN_STREAM("[ANIM] missed target loop rate of " << update_rate_ << " hz.");
      }
    }

  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "leds_animation_engine");
  ros::NodeHandle nh;

  AnimationEngineNode anim_node(nh);

  ROS_INFO("[ANIM] LEDS Animation Engine started ...");

  anim_node.Spin();

  return 0;
}
