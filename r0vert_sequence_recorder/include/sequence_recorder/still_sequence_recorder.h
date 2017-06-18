/* Copyright (c) 2017, Malte Splietker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The names of contributors may not be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CAMERA_RECORDER_STILL_SEQUENCE_RECORDER_H
#define CAMERA_RECORDER_STILL_SEQUENCE_RECORDER_H


#include <atomic>

#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <r0vert_msgs/WheelVelocity.h>
#include <sensor_msgs/Joy.h>
#include <turtle_actionlib/Velocity.h>
#include <thread>
#include "sequence.h"

namespace pt = boost::property_tree;

namespace sequence_recorder
{

/**
 * Sequence recorder that stops the robot before taking a picture.
 *
 * The Recorder tracks the traveled distance (odometry). If the distance exceeds a threshold
 * the robot is stopped, an picture is taken and the robot continues.
 */
class StillSequenceRecorder
{
public:
  StillSequenceRecorder(ros::NodeHandle &nh);

private:
  void JoyCallback(const sensor_msgs::Joy::ConstPtr &ptr);

  void WheelVelocityCallback(const r0vert_msgs::WheelVelocity::ConstPtr &ptr);

  /**
   * Callback used to pass messages from "/velocity_intercept" to "/velocity"
   * @param ptr
   */
  void VelocityInterceptCallback(const turtle_actionlib::Velocity::ConstPtr &ptr);

  void ImageRecordingThread();

  void TakePicture();

  ros::NodeHandle &n_;

  ros::Subscriber joystick_sub_;
  ros::Subscriber wheel_velocity_sub_;
  ros::Subscriber velocity_intercept_sub_;
  ros::Publisher velocity_pub_;

  cv::VideoCapture capture_;
  std::unique_ptr<Sequence> sequence_;

  std::thread recording_thread_;
  std::atomic_bool recording_;
  std::atomic_bool robot_stopped_;

  double traveled_distance_;
  double turned_angle_;

  int capture_device_;
  std::string output_directory_;
  int image_width_;
  int image_height_;
  bool grayscale_;
};

} // namespace sequence_recorder

#endif //CAMERA_RECORDER_STILL_SEQUENCE_RECORDER_H
