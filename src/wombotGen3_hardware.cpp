///////////////////////////////////////////////////////////////////////////////
//      Title     : wombot_hardware.cpp
//      Project   : wombot
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2021. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* Portions of code derived from package dynamixel_workbench_controllers.
*
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/**
*  Portions of code derived from package husky_base.
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "wombotGen3_base/wombotGen3_hardware.h"

using std::vector;
using std::list;


#define TWO_PI 6.283

namespace wombotGen3_base
{

    WombotHardware::WombotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh) :
        nh_(nh),
        private_nh_(private_nh),
        Encoder_Map(2)  // Encoder Map Object for 2 wheels
    {
        private_nh_.param<double>("max_speed", max_speed_, 10.0);

        registerControlInterfaces();

        //this topic will have two different messages in its pipeline, one for left and one for right.
        //so there will be two publish to this topic commands in the arduino code
        posRads_topic = "/pos_rad";
        posTicks_topic = "/pos_ticks";
        command_topic = "/pid_commands";
        vel_topic = "/vel_rad";

        //print some info about the node
        std::string incoming1 = nh_.resolveName (posRads_topic);
        ROS_INFO_STREAM("Listening for incoming data on topic " << incoming1 );

        //print some info about the node
        std::string incoming2 = nh_.resolveName (posTicks_topic);
        ROS_INFO_STREAM("Listening for incoming data on topic " << incoming2 );

        std::string outgoing = nh_.resolveName (command_topic);
        ROS_INFO_STREAM("Sending outgoing data on topic " << outgoing );

        std::string outgoing2 = nh_.resolveName (vel_topic);
        ROS_INFO_STREAM("Sending outgoing data on topic " << outgoing2 );

        // TODO: maybe change this to a queue of 1
        posTicks_sub = nh_.subscribe (posTicks_topic, 10, &WombotHardware::updateWheelPositionTicks_cb, this);
        posRads_sub = nh_.subscribe (posRads_topic, 10, &WombotHardware::updateWheelPositionRads_cb, this);
        // command_pub = nh_.advertise<std_msgs::Int32MultiArray>(command_topic, 10, false);
        command_pub = nh_.advertise<motion_control::pid_commands_data>(command_topic, 10, false);
        vel_pub = nh_.advertise<std_msgs::Float32MultiArray>(vel_topic, 10, false);
    }


    WombotHardware::~WombotHardware()
    {
        // std_msgs::Int32MultiArray msg;
        // msg.data.resize(4);
        // Left PWM, Left Dir, Right PWM, Right Dir
        // int32_t ctrl_data[4] = {0, 1, 0, 1};

        motion_control::pid_commands_data msg;
        int32_t ctrl_data[2] = {0, 0};
        
        for (int i = 0; i < NUM_MOTORS; ++i) {
            msg.data[i] = ctrl_data[i];
        }
        command_pub.publish(msg);
    }

    // void WombotHardware::updateWheelPositionTicks_cb (const std_msgs::Int64MultiArrayConstPtr &msg) 
    void WombotHardware::updateWheelPositionTicks_cb (const motion_control::encoder_ticks_data &msg)
    {
        for (int i = 0; i < NUM_MOTORS; i++) {
            wheel_positionTicks_[i] = msg.data[i];
        }
        
    }

    //**************
    //TODO make this a custom message to be ints and floats
    //refer to how Alex did his cutom float array message in Motor controller Scrat Navigation package
    // void WombotHardware::updateWheelPositionRads_cb(const std_msgs::Float32MultiArrayConstPtr &msg)
    void WombotHardware::updateWheelPositionRads_cb (const motion_control::encoder_rad_data &msg)
    {
        vector<float> rad_vector;
        for (int i = 0; i < NUM_MOTORS; i++) {
            rad_vector.push_back(msg.data[i]);
        }
        vector<double> wheel_velocities = Encoder_Map.mapRelativeEncoderRPS(rad_vector);
        for (int i = 0; i < NUM_MOTORS; i++) {
            wheel_positionRads_[i] = msg.data[i];
            wheel_velocitiesRPS_[i] = wheel_velocities[i];
        }
        // if used a pointer for Encoder_Map like this // EncoderMap *Encoder_Map = new EncoderMap(2);
        //, then you would use ->mapRelativeEncoderRPS instead of '.'
    }

    void WombotHardware::registerControlInterfaces()
    {
        // TODO: Parameterize these joint names
        joints_[LEFT].name  = "left_drive_wheel_joint";
        joints_[RIGHT].name = "right_drive_wheel_joint";

        for (unsigned int i = 0; i < NUM_MOTORS; i++) {
            // create the joint handles
            // these contain the latest joint information from the motors
            const hardware_interface::JointStateHandle joint_state_handle(joints_[i].name,
                                                                        &joints_[i].position,
                                                                        &joints_[i].velocity,
                                                                        &joints_[i].effort);

            // use this handle for velocity control
            const hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);

            // associate the joint handles with the interfaces
            joint_state_interface_.registerHandle(joint_state_handle);
            velocity_joint_interface_.registerHandle(joint_handle);
        }

        // register the interfaces with the controller
        // the controller will now look to these interfaces and the associated state variables
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
    }



    void WombotHardware::updateJointsFromHardware()
    {
        std_msgs::Float32MultiArray velMsg;
        velMsg.data.resize(2);

        // output pulled data to the joint handles
        for (int i = 0; i < NUM_MOTORS; ++i) 
        {
            //may need to convert this into a position like how it's done in
            // convertValue2Radian function of dynamixel_workbench.cpp
            joints_[i].position = wheel_positionRads_[i];
            joints_[i].velocity =  wheel_velocitiesRPS_[i];

            // reverse the LEFT motor
            if (i == RIGHT) {
                // angle *= -1;
                joints_[i].velocity *= -1;
            }
            velMsg.data[i] = joints_[i].velocity;

            // joints_[i].effort = 0;
        }
        vel_pub.publish(velMsg);
    }


    bool WombotHardware::publishCommandsToHardware()
    {
        uint8_t ids[NUM_MOTORS] = {(uint8_t)pololu_ids_[LEFT], (uint8_t)pololu_ids_[RIGHT]};

        // pull velocity commands from joint handles
        double control_signals[NUM_MOTORS];
        // TODO
        // maybe add multiplication of -1 back to the Left one. not sure how this will be handled by lower-level arduino
        control_signals[LEFT]  = joints_[LEFT].velocity_command;
        control_signals[RIGHT] = -1*joints_[RIGHT].velocity_command;

        //TODO use std::cout to print to console or can publish to a rostopic as a diagnostic
        // std_msgs::Int32MultiArray vel_command_msg;
        // vel_command_msg.data.resize(2);
        // vel_command_msg.data[0] = control_signals[LEFT];
        // vel_command_msg.data[1] = control_signals[RIGHT];
        // vel_command_pub.publish(vel_command_msg);

        // apply robot speed limits
        // shouldn't the address of these be passed in and not the actual variable?
        // WombotHardware::limitDifferentialSpeed(control_signals[LEFT], control_signals[RIGHT]);

        //TODO convert control signal (after being limited) to a pwm
        //refer to Alex's code on how to do this (line 268)
        // so keep sending a higher and higher control signal until it no longer increases the rpm
        // also send a lower and lower control signal until it no longer rotates the motor
        //also refer to my notes about converting angular velocity to 

        // std_msgs::Int32MultiArray msg;
        // msg.data.resize(4);
        // //Left Speed, Right Speeds
        // int32_t ctrl_data[4] = {(int32_t)control_signals[LEFT]*27, 1, (int32_t)control_signals[RIGHT]*27, 1};

        motion_control::pid_commands_data msg;
        int32_t ctrl_data[2] = {(int32_t)control_signals[LEFT]*27, (int32_t)control_signals[RIGHT]*27};
        //don't use NUM_MOTORS here since there are 4 values in ctrl_data
        for (int i = 0; i < NUM_MOTORS; ++i) {
            msg.data[i] = ctrl_data[i];
        }
        command_pub.publish(msg);

        return true;
    }



    void WombotHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
    {
        const double largest_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

        if (largest_speed > max_speed_) {
            const double reduction_rate = max_speed_ / largest_speed;

            diff_speed_left *= reduction_rate;
            diff_speed_right *= reduction_rate;
        }
    }


}  // namespace wombot_base



//LEFTOVER CODE
/*

uint8_t ids[NUM_MOTORS] = {(uint8_t)pololu_ids_[LEFT], (uint8_t)pololu_ids_[RIGHT]};

        int32_t get_position[NUM_MOTORS];
        float32_t get_velocity[NUM_MOTORS];
        int32_t get_current_or_load[NUM_MOTORS];

        const uint16_t length_of_data = control_items_["Present_Position"]->data_length + 
                                        control_items_["Present_Velocity"]->data_length + 
                                        control_items_["Present_Current"]->data_length;
        uint32_t get_all_data[length_of_data];

        for (int i = 0; i < NUM_MOTORS; ++i) {
            get_position[i] = encoder_ticks_[i];
            get_velocity[i] = encoder_vel_[i];
            get_current_or_load[i] = 0;
            // convertValue2Load outputs percentage [0,100] of max torque
            joints_[i].effort = dxl_wb_->convertValue2Load(get_current_or_load[i]) * 0.01 * MX_64_MAX_TORQUE;
        }

***************
if(msg.header.frame_id == "left_wheel")
{
    encoder_ticks_[0] = msg.ticks;
    encoder_vel_[0] = msg.angVel;
}
else if(msg.header.frame_id == "right_wheel")
{
    encoder_ticks_[1] = msg.ticks;
    encoder_vel_[1] = msg.angVel;
}

***********************
      catch (std::runtime_error &e)
      {
        ROS_ERROR_STREAM("Error in getting encoder info across rosserial: "
                          << e.what());
      }
****************
// list< vector<float> > temp_holder;
// temp_holder.push_back(msg->data);

******************
last_position_[LEFT] = NAN;
last_position_[RIGHT] = NAN;

*****************

//Hangling Rollover
double angle = (double)wheel_positions_[i];
// load initial cache
if (std::isnan(last_position_[i])) {
    last_position_[i] = angle;
}

// detect encoder rollover
// the output range of the driver is [-pi, pi]
const double travel = angle - last_position_[i];

// positive rollover
// TODO may not need this as it should be handled further down
if (travel < 0 && joints_[i].velocity > 0) {
    joints_[i].position += travel + TWO_PI;
}
// negative rollover
else if (travel > 0 && joints_[i].velocity < 0) {
    joints_[i].position += travel - TWO_PI;
}
// no rollover
else {
    joints_[i].position += travel;
}

last_position_[i] = angle;

***********************


*/