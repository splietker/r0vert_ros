#ifndef _CAMERA_RECORDER_IMAGE_SEQUENCE_RECORDER_H_
#define _CAMERA_RECORDER_IMAGE_SEQUENCE_RECORDER_H_

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <turtle_actionlib/Velocity.h>
#include <r0vert_msgs/WheelVelocity.h>
#include "sequence.h"

namespace pt = boost::property_tree;

using namespace std::chrono;

namespace sequence_recorder
{

class ImageSequenceRecorder
{
public:
  ImageSequenceRecorder(ros::NodeHandle &nh);

private:
  void StartRecording();

  void StopRecording();

  void JoyCallback(const sensor_msgs::Joy::ConstPtr &ptr);

  void WheelVelocityCallback(const r0vert_msgs::WheelVelocity::ConstPtr &ptr);

  void Record();

  ros::NodeHandle &n_;
  ros::Subscriber joystick_sub_;
  ros::Subscriber wheel_velocity_sub_;

  std::thread recording_thread_;
  std::atomic_bool is_recording_;

  std::unique_ptr<Sequence> sequence_;

  // Variables for measuring fps
  system_clock::time_point start_time_;
  system_clock::time_point last_frame_time_;
  milliseconds frame_time_min_;
  milliseconds frame_time_max_;

  // Configuration parameters
  int capture_device_;
  std::string output_directory_;
  int image_width_;
  int image_height_;
  int framerate_;
  bool grayscale_;

};

} // namespace sequence_recorder

#endif // _CAMERA_RECORDER_IMAGE_SEQUENCE_RECORDER_H_
