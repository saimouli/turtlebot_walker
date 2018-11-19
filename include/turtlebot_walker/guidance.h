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
 *@file guidance.h
 *@author Saimouli Katragadda
 *@copyright MIT License
 *@brief defines guidance class functions
 */

#ifndef INCLUDE_TURTLEBOT_WALKER_GUIDANCE_H_
#define INCLUDE_TURTLEBOT_WALKER_GUIDANCE_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Class for Guidance.
 */
class Guidance {
 private:
  // Variable to detect collisions
  bool collision;
  // Variable for velocities
  geometry_msgs::Twist msg;
  // node handler
  ros::NodeHandle nh;
  // publish velocities
  ros::Publisher pubVelocities;
  // subscribe laserscan topic
  ros::Subscriber sub;
  // variable to store linear speed
  float linearVel;
  // variable to store angular speed
  float angularVel;

 public:
  /**
   * @brief      constructor for guidance
   */
  Guidance();
  /**
   * @brief      destroys object
   */
  ~Guidance();
  /**
   * @brief      callback function for laserscan
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief      Checks for obstacles nearby
   * @return     of type bool. 1 if obstacle found,
   *             0 otherwise
   */
  bool checkObstacle();
  /**
   * @brief      function which runs robot
   */
  void moveRobot();
};

#endif
