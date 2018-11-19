/**
 *  MIT License
 *
 *  Copyright (c) 2018 Saimouli Katragadda
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file guidance.cpp
 *@author Saimouli Katragadda
 *@copyright MIT License
 *@brief implements guidance class functions
 */

#include <iostream>
#include "turtlebot_walker/guidance.h"

/**
 * @brief      Constructs the object.
 */
Guidance::Guidance() {
  // define lin. and angular speeds
  linearVel = 0.2;
  angularVel = 1.0;
  // Publish the velocity
  pubVelocities = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  // Subscribe to laser scan to detect obstacles
  sub = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
      &Guidance::laserCallback, this);
  // define initial velocity
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // stop the publisher
  pubVelocities.publish(msg);
}

/**
 * @brief      Class Destructor
 */
Guidance::~Guidance() {
  // Stop the turtlebot at the end of the program
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // publish the turtlebot
  pubVelocities.publish(msg);
}

/**
 * @brief      Callback for the laser scan data
 * @param      msg, the message
 */
void Guidance::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 0.8) {
      collision = true;
      return;
    }
  }
  collision = false;
}

/**
 * @brief      Returns the collision flag
 * @return     of type bool return when obstacle is detected
 */
bool Guidance::checkObstacle() {
  return collision;
}

/**
 * @brief      guidance logic to run the robot
 */
void Guidance::moveRobot() {
  // publish at 10 Hz
  ros::Rate loop(10);
  // keep running until ros dies
  while (ros::ok()) {
    // check obstacle
    if (checkObstacle()) {
      // ros out obstacle found
      ROS_INFO("Obstacle present turning");
      // Stop the robot
      msg.linear.x = 0.0;
      // Turn the robot
      msg.angular.z = angularVel;
    } else {
      ROS_INFO("Moving Forward");
      // Stop turning
      msg.angular.z = 0.0;
      // set forward speed
      msg.linear.x = linearVel;
    }

    // Publish the vel message
    pubVelocities.publish(msg);

    ros::spinOnce();
    loop.sleep();
  }
}
