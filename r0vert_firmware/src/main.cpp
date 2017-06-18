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

#include <Arduino.h>

#include <ros.h>
#include <turtle_actionlib/Velocity.h>
#include <r0vert_msgs/BatteryVoltage.h>
#include <r0vert_msgs/WheelVelocity.h>

#include "motor.h"
#include "battery.h"
#include "timer.h"
#include "wheel_encoder.h"


ros::NodeHandle nh;

// Subscribers
void velocity_callback(const turtle_actionlib::Velocity &velocity_msg);

ros::Subscriber<turtle_actionlib::Velocity> velocity_subscriber("velocity", &velocity_callback);

// Publishers
r0vert_msgs::BatteryVoltage battery_msg;
ros::Publisher battery_publisher("battery", &battery_msg);
r0vert_msgs::WheelVelocity velocity_msg;
ros::Publisher velocity_publisher("wheel_velocity", &velocity_msg);

// Publishing timers
void publish_status();

Timer publish_status_timer(1000, &publish_status);

// Publishing timers
void publish_velocity();

Timer publish_velocity_timer(50, &publish_velocity);

// Hardware
Motor motor_left(6, 7);
Motor motor_right(4, 5);

PIDController controller_left(&motor_left);
PIDController controller_right(&motor_right);

WheelEncoder encoder_left(50, 52, 53);
WheelEncoder encoder_right(50, 52, 51, true);

Battery battery1(A0, 15.275);
Battery battery2(A1, 15.218);
//14.861825541788024

void velocity_callback(const turtle_actionlib::Velocity &velocity_msg)
{
  float hypotenuse = sqrt(pow(velocity_msg.linear, 2) + pow(velocity_msg.angular, 2));
  float angle;
  if (hypotenuse == 0)
  {
    angle = PI / 2;
  }
  else
  {
    float normalized_x = -velocity_msg.angular / hypotenuse;
    angle = acos(normalized_x);
  }

  float speed = min(hypotenuse, 1);

  // Apply exponential function for better steerability
  speed = pow(2, speed) - 1;

  float heading = (angle - PI / 2) / (PI / 2);

  if (heading > 0 and velocity_msg.linear > 0)
  {
    controller_left.SetSpeed((1 - 2 * heading) * speed);
    controller_right.SetSpeed(speed);
  }
  else if (heading > 0 and velocity_msg.linear <= 0)
  {
    controller_left.SetSpeed(-speed);
    controller_right.SetSpeed((-1 + 2 * heading) * speed);
  }
  else if (heading <= 0 and velocity_msg.linear > 0)
  {
    controller_left.SetSpeed(speed);
    controller_right.SetSpeed((1 + 2 * heading) * speed);
  }
  else if (heading <= 0 and velocity_msg.linear <= 0)
  {
    controller_left.SetSpeed((-1 - 2 * heading) * speed);
    controller_right.SetSpeed(-speed);
  }
}

void publish_status()
{
  battery_msg.battery1 = battery1.Voltage();
  battery_msg.battery2 = battery2.Voltage();
  battery_publisher.publish(&battery_msg);
}

void publish_velocity()
{
  static const double velocity_threshold = 0.02;
  static unsigned long last_publish_time = micros();
  unsigned long current_time = micros();
  double time_diff = (current_time - last_publish_time) * 1e-6;
  velocity_msg.left = encoder_left.IncrementalDiff() / time_diff;
  velocity_msg.right = encoder_right.IncrementalDiff() / time_diff;
  velocity_msg.time = time_diff;

  if (controller_left.set_speed() == 0 and controller_right.set_speed() == 0
      and fabs(velocity_msg.left) < velocity_threshold
      and fabs(velocity_msg.right) < velocity_threshold)
  { // If motors set to stop and measured velocity close to 0
    velocity_msg.left = 0;
    velocity_msg.right = 0;
  }

  velocity_publisher.publish(&velocity_msg);

  last_publish_time = current_time;
}

void motor_control()
{
  double velocity_left = encoder_left.Velocity();
  double velocity_right = encoder_right.Velocity();
  controller_left.EncoderUpdate(velocity_left);
  controller_right.EncoderUpdate(velocity_right);

  velocity_msg.left += velocity_left;
  velocity_msg.right += velocity_right;
}

void setup()
{
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(velocity_subscriber);
  nh.advertise(battery_publisher);
  nh.advertise(velocity_publisher);

  encoder_left.Init();
  encoder_right.Init();
  controller_left.Init();
  controller_right.Init();
}

void loop()
{
  nh.spinOnce();

  encoder_left.Update();
  encoder_right.Update();
  battery1.Update();
  battery2.Update();

  publish_status_timer.Update();
  publish_velocity_timer.Update();
  motor_control();
}
