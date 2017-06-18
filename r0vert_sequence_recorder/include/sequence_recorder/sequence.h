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

#ifndef CAMERA_RECORDER_SEQUENCE_H
#define CAMERA_RECORDER_SEQUENCE_H

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv/cv.hpp>
#include <r0vert_msgs/WheelVelocity.h>

namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

namespace sequence_recorder
{

class Sequence
{
public:
  Sequence(const std::string &output_directory, bool grayscale=true);

  void AddFrame(const cv::Mat &frame);

  void AddWheelVelocity(const r0vert_msgs::WheelVelocity::ConstPtr &velocity);

  void WriteMetadata();

private:
  fs::path output_path_;

  bool grayscale_;

  pt::ptree wheel_velocity_tree_;

  pt::ptree frames_tree_;
};

} // namespace sequence_recorder

#endif //CAMERA_RECORDER_SEQUENCE_H
