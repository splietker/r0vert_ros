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

#include <chrono>
#include <thread>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
#include <sensor_msgs/Joy.h>

#include<sequence_recorder/still_sequence_recorder.h>

namespace fs = boost::filesystem;

using namespace std::chrono;

namespace sequence_recorder
{

StillSequenceRecorder::StillSequenceRecorder(ros::NodeHandle &nh)
    : n_(nh), recording_(false), robot_stopped_(false)
{
  n_.param<int>("capture_device", capture_device_, 0);
  n_.param<std::string>("output_directory", output_directory_, "/tmp/recordings");
  n_.param<int>("image_width", image_width_, 640);
  n_.param<int>("image_height", image_height_, 480);
  n_.param<bool>("grayscale", grayscale_, false);

  joystick_sub_ = n_.subscribe<sensor_msgs::Joy>(
      "/joy", 10, &StillSequenceRecorder::JoyCallback, this);
  velocity_intercept_sub_ = n_.subscribe<turtle_actionlib::Velocity>(
      "/velocity_intercept", 10, &StillSequenceRecorder::VelocityInterceptCallback, this);
  wheel_velocity_sub_ = n_.subscribe<r0vert_msgs::WheelVelocity>(
      "/wheel_velocity", 10, &StillSequenceRecorder::WheelVelocityCallback, this);

  velocity_pub_ = n_.advertise<turtle_actionlib::Velocity>("/velocity", 10);
}


void StillSequenceRecorder::VelocityInterceptCallback(const turtle_actionlib::Velocity::ConstPtr &ptr)
{
  if (not robot_stopped_)
  {
    velocity_pub_.publish(ptr);
  }
}

void StillSequenceRecorder::JoyCallback(const sensor_msgs::Joy::ConstPtr &ptr)
{
  if (ptr->buttons[14] and not recording_)
  {
    sequence_ = std::make_unique<Sequence>(output_directory_, grayscale_);
    robot_stopped_ = false;
    recording_ = true;

    recording_thread_ = std::thread(&StillSequenceRecorder::ImageRecordingThread, this);

    ROS_INFO("Recording started");
  }
  else if (ptr->buttons[15] and recording_)
  {
    recording_ = false;
    robot_stopped_ = false;
    sequence_->WriteMetadata();

    recording_thread_.join();
    ROS_INFO("Recording stopped");
  }
}

void StillSequenceRecorder::WheelVelocityCallback(const r0vert_msgs::WheelVelocity::ConstPtr &ptr)
{
  static high_resolution_clock::time_point last_callback = high_resolution_clock::now();
  high_resolution_clock::time_point now = high_resolution_clock::now();

  if (recording_)
  {
    sequence_->AddWheelVelocity(ptr);
    if (not robot_stopped_)
    {
      duration<double> time_diff = now - last_callback;
      double forward_velocity = 0.032 / 2 * (ptr->right + ptr->left);
      double turn_velocity = 0.032 / 0.18 * (ptr->right - ptr->left);
      traveled_distance_ += std::abs(ptr->time * forward_velocity);
      turned_angle_ += std::abs(ptr->time * turn_velocity);
    }

    if (traveled_distance_ > 0.02 or turned_angle_ > 0.03)
    {
      TakePicture();
      traveled_distance_ = 0;
      turned_angle_ = 0;
    }
  }

  last_callback = now;
}

void StillSequenceRecorder::TakePicture()
{
  // Stop robot
  robot_stopped_ = true;
  turtle_actionlib::Velocity velocity;
  velocity.angular = 0;
  velocity.linear = 0;
  velocity_pub_.publish(velocity);
}

void StillSequenceRecorder::ImageRecordingThread()
{
  capture_.open(0);
  capture_.set(CV_CAP_PROP_FRAME_WIDTH, image_width_);
  capture_.set(CV_CAP_PROP_FRAME_HEIGHT, image_height_);
  capture_.set(CV_CAP_PROP_FPS, 5);

  cv::Mat frame;
  while (recording_)
  {
    capture_ >> frame;

    if (robot_stopped_)
    {
      // Wait for robot to stop
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      // Clear camera buffer
      while (true)
      {
        system_clock::time_point time_start = system_clock::now();
        capture_ >> frame;
        std::cout << duration<double>(system_clock::now() - time_start).count() << ", "
                  << capture_.get(cv::CAP_PROP_FPS)
                  << std::endl;
        if (duration<double>(system_clock::now() - time_start).count() * capture_.get(cv::CAP_PROP_FPS) > 0.5)
        {
          break;
        }
      }

      // Store newest image
      capture_ >> frame;
      sequence_->AddFrame(frame);

      // Resume driving
      robot_stopped_ = false;
    }
  }

  capture_.release();
}

} // namespace sequence_recorder