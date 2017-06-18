#include <ctime>
#include <chrono>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <opencv/cv.hpp>

#include <sequence_recorder/image_sequence_recorder.h>

namespace fs = boost::filesystem;
using namespace cv;

namespace sequence_recorder
{

ImageSequenceRecorder::ImageSequenceRecorder(ros::NodeHandle &nh) :
    n_(nh), is_recording_(false)
{
  n_.param<int>("capture_device", capture_device_, 0);
  n_.param<std::string>("output_directory", output_directory_, "/tmp/recordings");
  n_.param<int>("image_width", image_width_, 640);
  n_.param<int>("image_height", image_height_, 480);
  n_.param<int>("framerate", framerate_, 15);
  n_.param<bool>("grayscale", grayscale_, false);

  joystick_sub_ = n_.subscribe<sensor_msgs::Joy>(
      "/joy", 10, &ImageSequenceRecorder::JoyCallback, this);
  wheel_velocity_sub_ = n_.subscribe<r0vert_msgs::WheelVelocity>(
      "/wheel_velocity", 10, &ImageSequenceRecorder::WheelVelocityCallback, this);

  fs::path output_path(output_directory_);
  fs::create_directories(output_path);
}

void ImageSequenceRecorder::JoyCallback(const sensor_msgs::Joy::ConstPtr &ptr)
{
  if (ptr->buttons[14])
  {
    StartRecording();
  }
  else if (ptr->buttons[15])
  {
    StopRecording();
  }
}

void ImageSequenceRecorder::StartRecording()
{
  if (is_recording_)
  {
    return;
  }
  recording_thread_ = std::thread(&ImageSequenceRecorder::Record, this);
  ROS_INFO("Recording started");
}

void ImageSequenceRecorder::StopRecording()
{
  if (not is_recording_)
  {
    return;
  }
  is_recording_ = false;
  recording_thread_.join();
  ROS_INFO("Recording stopped");
}

void ImageSequenceRecorder::Record()
{
  is_recording_ = true;

  sequence_ = std::make_unique<Sequence>(output_directory_, grayscale_);

  std::string gst_pipeline = "v4l2src ! video/x-raw,width=320,height=240,framerate=%d/1,format=RGB ! videoconvert ! appsink";
  VideoCapture capture(boost::str(boost::format(gst_pipeline) % framerate_));
  assert(capture.isOpened());

  start_time_ = system_clock::now();
  frame_time_min_ = milliseconds::max();
  frame_time_max_ = milliseconds::min();
  unsigned long frame_count = 0;
  Mat frame;
  while (is_recording_)
  {
    capture >> frame;
    system_clock::time_point new_frame_time = system_clock::now();

    try
    {
      sequence_->AddFrame(frame);
    }
    catch (std::runtime_error &e)
    {
      ROS_ERROR("Error: exception while writing image: ", e.what());
    }

    milliseconds diff = duration_cast<milliseconds>(new_frame_time - last_frame_time_);
    if (diff < frame_time_min_)
    {
      frame_time_min_ = diff;
    }
    else if (diff > frame_time_max_)
    {
      frame_time_max_ = diff;
    }
    last_frame_time_ = new_frame_time;
    frame_count += 1;
  };

  milliseconds total_time = duration_cast<milliseconds>(system_clock::now() - start_time_);
  milliseconds frame_time_avg = total_time / frame_count;
  ROS_INFO("FPS avg: %.2f, min: %.2f, max: %.2f",
           1000.0 / frame_time_avg.count(),
           1000.0 / frame_time_max_.count(),
           1000.0 / frame_time_min_.count());

  sequence_->WriteMetadata();
}

void ImageSequenceRecorder::WheelVelocityCallback(const r0vert_msgs::WheelVelocity::ConstPtr &ptr)
{
  if (is_recording_)
  {
    sequence_->AddWheelVelocity(ptr);
  }
}

} // namespace sequence_recorder
