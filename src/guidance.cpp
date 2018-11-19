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
 
}

/**
 * @brief      Class Destructor
 */
Guidance::~Guidance() {
 
}

/**
 * @brief      Callback for the laser scan data
 * @param      msg, the message
 */
void Guidance::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
 
}

/**
 * @brief      Returns the collision flag
 * @return     of type bool return when obstacle is detected
 */
bool Guidance::checkObstacle() {
}

/**
 * @brief      guidance logic to run the robot
 */
void Guidance::moveRobot() {
}
