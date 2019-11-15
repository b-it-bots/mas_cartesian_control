/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Author: Matthias Fueller (matthias.fueller@h-brs.de)
 *
 *  Copyright (c) 2013, Hochschule Bonn-Rhein-Sieg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redstributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * This is the ROS node class for the Cartesian controller.
 * It provides the connection between the ROS environment
 * and the arm_cartesian_control class.
 */

#ifndef ROS_ARM_CARTESIAN_CONTROL_H_
#define ROS_ARM_CARTESIAN_CONTROL_H_

#include <mcr_arm_cartesian_control/ros_urdf_loader.h>
#include <mcr_arm_cartesian_control/arm_cartesian_control.h>
#include <sensor_msgs/JointState.h>
#include <kdl/kdl.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <tf/transform_listener.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>


class RosArmCartesianControl
{
private:
    ros::NodeHandle node_handle;

    std::string velocity_command_topic = "/right_arm/joint_group_velocity_controller/command";
    std::string sigma_values_topic = "sigma_values";
    std::string weight_ts_topic = "weight_task_space";
    std::string weight_js_topic = "weight_joint_space";
    std::string joint_state_topic = "/joint_states";
    std::string cart_control_topic = "/mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command";
    std::string wrench_topic = "/ft_sensor/ft_compensated";
    std::string control_mode_topic = "control_modes";
    std::string tooltip_name = "DEFAULT_CHAIN_TIP";

public:

    arm_cc::Arm_Cartesian_Control* control;

    KDL::Chain arm_chain;
    KDL::JntArray joint_positions;
    KDL::Twist targetVelocity;
    KDL::ChainIkSolverVel* ik_solver;
    KDL::JntArrayVel cmd_velocities;

    Eigen::VectorXd sigma;
    Eigen::MatrixXd weight_ts;
    Eigen::MatrixXd weight_js;

    ros::Publisher cmd_vel_publisher;
    ros::Publisher sigma_publisher;
    ros::Subscriber sub_joint_state;
    ros::Subscriber sub_wjs;
    ros::Subscriber sub_wts;
    ros::Subscriber sub_cc;
    ros::Subscriber sub_wrench;
    ros::Subscriber sub_control_mode;
    ros::Time t_last_command;
    ros::Rate* loop_rate;
    ROS_URDF_Loader loader;
    tf::TransformListener *tf_listener;
    geometry_msgs::WrenchStamped wrench;
    std_msgs::Float32MultiArray sigma_array;

    std::vector<bool> joint_positions_initialized;
    std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits;
    std::vector<double> upper_limits;
    std::vector<double> lower_limits;

    std::string root_name = "DEFAULT_CHAIN_ROOT";
    bool motion_stopped = true;
    bool active = false;
    bool use_float_array_msg = false;
    int nrOfJoints;
    int axis_control_modes[6] = {0, 0, 0, 0, 0, 0};
    double rate = 50;
    float wrench_threshold_x = 4.0,
        wrench_threshold_y = 4.0,
        wrench_threshold_z = 4.0,
        wrench_threshold_ax = 0.4,
        wrench_threshold_ay = 0.4,
        wrench_threshold_az = 0.4;
    float linear_vel_const = 0.01,
          angular_vel_const = 0.2;
    float max_arm_cartesian_velocity = 0.1; // m/s
    float max_arm_joint_velocity = 0.25; // rad/s


    RosArmCartesianControl(ros::NodeHandle* nh);

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& joints);

    void wtsCallback(std_msgs::Float32MultiArray weights);

    void wjsCallback(std_msgs::Float32MultiArray weights);

    void cmCallback(std_msgs::Int32MultiArray control_modes);

    void wrenchCallback(geometry_msgs::WrenchStampedConstPtr wr);

    void ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity);

    void init_ik_solver();

    void init_joint_msgs();

    void publishJointVelocities(KDL::JntArrayVel& joint_velocities);

    void publishJointVelocities_FA(KDL::JntArrayVel& joint_velocities);

    void stopMotion();

    bool watchdog();

    void force_torque_control();
    
    void control_loop();
};
#endif /* ROS_ARM_CARTESIAN_CONTROL_H_ */
