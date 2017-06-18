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
#include <boost/format.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <sequence_recorder/sequence.h>

using namespace std::chrono;

namespace sequence_recorder
{

std::string GetTimeStamp()
{
  static const std::string TIME_FORMAT = "%F_%X";

  time_t rawtime;
  time(&rawtime);
  struct tm *timeinfo = localtime(&rawtime);
  char buffer[50];
  strftime(buffer, 50, TIME_FORMAT.c_str(), timeinfo);

  return std::string(buffer);
}

Sequence::Sequence(const std::string &output_directory, bool grayscale)
    : grayscale_(grayscale), output_path_(output_directory)
{
  output_path_ /= GetTimeStamp();
  fs::create_directories(output_path_);
}

void Sequence::AddFrame(const cv::Mat &frame)
{
  static const std::vector<int> IMAGE_OUTPUT_PARAMETERS({CV_IMWRITE_PXM_BINARY});
  static const std::string filename_format_string = "%04d.bmp";

  high_resolution_clock::time_point frame_time = high_resolution_clock::now();

  if (grayscale_)
  {
//    cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
  }

  size_t frame_count = frames_tree_.size();
  std::string filename = boost::str(boost::format(filename_format_string) % frame_count);

  fs::path image_path = output_path_ / filename;
  imwrite(image_path.string(), frame, IMAGE_OUTPUT_PARAMETERS);

  pt::ptree entry;
  entry.put("", filename);
  frames_tree_.push_back(std::make_pair(std::to_string(frame_time.time_since_epoch().count()), entry));
}

void Sequence::AddWheelVelocity(const r0vert_msgs::WheelVelocity::ConstPtr &velocity)
{
  nanoseconds now = high_resolution_clock::now().time_since_epoch();
  pt::ptree entry;
  entry.put("left", velocity->left);
  entry.put("right", velocity->right);
  entry.put("time", velocity->time);
  wheel_velocity_tree_.push_back(std::make_pair(std::to_string(now.count()), entry));

}

void Sequence::WriteMetadata()
{
  fs::path wheel_velocity_file_path = output_path_ / "wheel_velocity.json";
  pt::write_json(wheel_velocity_file_path.string(), wheel_velocity_tree_);

  fs::path frames_file_path = output_path_ / "frames.json";
  pt::write_json(frames_file_path.string(), frames_tree_);
}

} // namespace sequence_recorder