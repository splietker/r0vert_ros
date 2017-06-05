/* Copyright (c) 2016, Malte Splietker
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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <turtle_actionlib/Velocity.h>

class TeleopJoy
{
public:
  TeleopJoy();

  enum Mode {
    OneStick = 0,
    TwoStick,
    Tank
  };

private:
  void JoyCallback(const sensor_msgs::Joy::ConstPtr &ptr);

  ros::NodeHandle n_;

  ros::Publisher velocity_pub_;
  ros::Subscriber joystick_sub_;
  int axis_angular_, axis_linear_, axis_throttle_;
  double max_speed_;

  Mode mode_;
};

TeleopJoy::TeleopJoy() :
  mode_(TwoStick)
{
  // Create joystick axis mapping parameters
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("axis_angular", axis_angular_, 0);
  private_node_handle.param("axis_linear", axis_linear_, 1);
  private_node_handle.param("axis_throttle", axis_throttle_, 2);
  private_node_handle.param("max_speed", max_speed_, 1.0);

  velocity_pub_ = n_.advertise<turtle_actionlib::Velocity>("velocity", 1);
  joystick_sub_ = n_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::JoyCallback, this);
}

void TeleopJoy::JoyCallback(const sensor_msgs::Joy::ConstPtr &ptr)
{
  if (ptr->buttons[12])
  {
    mode_ = static_cast<Mode>((mode_ + 1) % 3);
    ROS_INFO("New mode: %d", mode_);
  }

  if (mode_ == OneStick)
  {
    //n_.getParam("max_speed", max_speed_);
    ROS_DEBUG("Received: v=%f, a=%f", (float) ptr->axes[axis_linear_], (float) ptr->axes[axis_angular_]);
    turtle_actionlib::Velocity velocity;
    velocity.linear = (float) ptr->axes[axis_linear_] * (float) max_speed_;
    velocity.angular = (float) ptr->axes[axis_angular_] * (float) max_speed_;
    velocity_pub_.publish(velocity);
  }
  else if (mode_ == TwoStick)
  {
    turtle_actionlib::Velocity velocity;
    velocity.linear = (float) ptr->axes[1] * (float) max_speed_;
    velocity.angular = (float) ptr->axes[2] * (float) max_speed_;
    velocity_pub_.publish(velocity);
  }
  else if (mode_ == Tank)
  {
    turtle_actionlib::Velocity velocity;
    velocity.linear =  ((float) ptr->axes[3] + ptr->axes[1]) * (float) max_speed_ / 2;
    velocity.angular = ((float) ptr->axes[3] - ptr->axes[1]) * (float) max_speed_ / 2;
    velocity_pub_.publish(velocity);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "r0vert_teleop");

  TeleopJoy teleopJoy;

  ros::spin();
}
