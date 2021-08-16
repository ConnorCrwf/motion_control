///////////////////////////////////////////////////////////////////////////////
//      Title     : wombot_hardware.h
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


#ifndef WOMBOT_BASE_WOMBOT_HARDWARE_H
#define WOMBOT_BASE_WOMBOT_HARDWARE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// #include "../src/encoder_mapping.h"
#include "encoder_mapping.h"

#include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Int64MultiArray.h>
// #include <std_msgs/Int32MultiArray.h>

#include <motion_control/encoder_rad_data.h>
#include <motion_control/encoder_ticks_data.h>
#include <motion_control/pid_commands_data.h>

// #include <motion_control/EncoderInfo.h>
// #include <motion_control/Pwm.h>

#include <list>
#include <vector>

using std::vector;
using std::list;

#include <yaml-cpp/yaml.h>

#define NUM_MOTORS 2

// The RobotHardware is class inheritance syntax. So WombotHardware is just an extended version of the RobotHardware class. 
// Whereas for Encoder_Map, your WombotHardware class contains an instance of EncoderMap, and does not inherit from it

namespace wombotGen3_base
{
  /**
  * Class representing Wombot hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class WombotHardware :
    public hardware_interface::RobotHW
  {
  public:
    EncoderMap Encoder_Map;
    // EncoderMap *Encoder_Map = new EncoderMap(2);

    WombotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

    ~WombotHardware();

    // void updateWheelPositionRads_cb (const std_msgs::Float32MultiArrayConstPtr &msg);
    // void updateWheelPositionTicks_cb (const std_msgs::Int64MultiArrayConstPtr &msg);

    void updateWheelPositionRads_cb (const motion_control::encoder_rad_data &msg);
    void updateWheelPositionTicks_cb (const motion_control::encoder_ticks_data &msg);
    /**
     * @brief Connect to the two Dynamixel drive motors and configure them.
     */
    bool connect();

    /**
     * @brief Populates the joints variable by pulling status from the hardware.
     */
    void updateJointsFromHardware();

    /**
     * @brief Write current velocity commands to the motors.
     */
    bool publishCommandsToHardware();

  private:

    enum Motors {LEFT=0, RIGHT=1};

    typedef int32_t MotorID;
    /**
     * Motor ID numbers
     * pololu_ids_[0] is left motor and pololu_ids_[1] is right motor
     */
    MotorID pololu_ids_[NUM_MOTORS];

    typedef int32_t POLOLU_REG_SIZE;

    std::string posRads_topic; //default input
    std::string posTicks_topic; //default input
    std::string command_topic; //default output
    std::string vel_topic; //default output
    //initiliaze publishers and subscribers 
    ros::Publisher command_pub;
    ros::Publisher vel_pub;
    ros::Subscriber posRads_sub;
    ros::Subscriber posTicks_sub;

    /** Various helper functions for connect() **/
    bool initControlItems(void);

    /**
     * @brief Create joint handles and control interfaces, and register them with the robot hardware.
     */
    void registerControlInterfaces();

    /**
     * @brief Ensures that the motor speeds are not saturated by the velocity commands.
     * @param travel_speed_left The desired left wheel speed. Will be reduced if necessary to prevent saturation.
     * @param travel_speed_right The desired left wheel speed. Will be reduced if necessary to prevent saturation.
     */
    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    ros::NodeHandle nh_, private_nh_;


    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    // found in joint_command_interface.h. have to do file search in /opt/ros to find it
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // ROS Parameters
    double max_speed_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller.
    * joints_[0] is left motor and joints_[1] is right motor.
    */
    struct Joint
    {
        std::string name;
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() :
            position(0), velocity(0), effort(0), velocity_command(0)
        { }
    } joints_[2];

    double wheel_velocitiesRPS_[NUM_MOTORS];
    double wheel_positionTicks_[NUM_MOTORS];
    double wheel_positionRads_[NUM_MOTORS];

  };

}  // namespace wombot_base

#endif  // WOMBOT_BASE_WOMBOT_HARDWARE_H



//LEFTOVER CODE

/*
// stores the controls
std::map<std::string, const ControlItem*> control_items_;

********************
// sued to detect encoder rollover
double last_position_[NUM_MOTORS];

*/