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

#include "motor.h"
#include "battery.h"
#include "timer.h"


ros::NodeHandle nh;

// Subscribers
void velocity_callback(const turtle_actionlib::Velocity &velocity_msg);
ros::Subscriber<turtle_actionlib::Velocity> velocity_subscriber("velocity", &velocity_callback);

// Publishers
r0vert_msgs::BatteryVoltage battery_msg;
ros::Publisher battery_publisher("battery", &battery_msg);

// Timer set to publish status every 1000ms
void publish_status();
Timer publishing_timer(1000, &publish_status);

// Hardware
Motor motor_left(4, 5);
Motor motor_right(6, 7);

Battery battery1(A0, 15.275);
Battery battery2(A1, 15.218);

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
    float normalized_x = velocity_msg.angular / hypotenuse;
    angle = acos(normalized_x);
  }

  float speed = min(hypotenuse, 1);

  // Apply exponential function for better steerability
  speed = pow(2, speed) - 1;

  float heading = (angle - PI / 2) / (PI / 2);

  if (heading > 0 and velocity_msg.linear > 0)
  {
    motor_left.SetSpeed((1 - 2 * heading) * speed);
    motor_right.SetSpeed(speed);
  }
  else if (heading > 0 and velocity_msg.linear <= 0)
  {
    motor_left.SetSpeed(-speed);
    motor_right.SetSpeed((-1 + 2 * heading) * speed);
  }
  else if (heading <= 0 and velocity_msg.linear > 0)
  {
    motor_left.SetSpeed(speed);
    motor_right.SetSpeed((1 + 2 * heading) * speed);
  }
  else if (heading <= 0 and velocity_msg.linear <= 0)
  {
    motor_left.SetSpeed((-1 - 2 * heading) * speed);
    motor_right.SetSpeed(-speed);
  }
}

void publish_status()
{
  battery_msg.battery1 = battery1.Voltage();
  battery_msg.battery2 = battery2.Voltage();
  battery_publisher.publish(&battery_msg);
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(velocity_subscriber);
  nh.advertise(battery_publisher);
}

void loop()
{
  nh.spinOnce();

  battery1.Update();
  battery2.Update();

  publishing_timer.Update();

  delay(1);
}
