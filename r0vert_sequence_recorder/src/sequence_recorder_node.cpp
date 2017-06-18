#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sequence_recorder/still_sequence_recorder.h>
#include <sequence_recorder/image_sequence_recorder.h>

using namespace sequence_recorder;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sequence_recorder");

  ros::NodeHandle nh("~");
  bool still_images;
  nh.param<bool>("still_images", still_images, false);
  if (still_images)
  {
    StillSequenceRecorder still_sequence_recorder(nh);
    ros::spin();
  }
  else
  {
    ImageSequenceRecorder image_sequence_recorder(nh);
    ros::spin();
  }

  return 0;
}
