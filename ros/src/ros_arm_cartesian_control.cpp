/*
 * ros_arm_cartesian_control.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: matthias
 */

#include "mcr_arm_cartesian_control/ros_arm_cartesian_control.h"

#include "ros/ros.h"

RosArmCartesianControl::RosArmCartesianControl(ros::NodeHandle* nh):node_handle(*nh) 
{

    node_handle.getParam("use_float_array_msg", use_float_array_msg);
    node_handle.getParam("joint_state_topic", joint_state_topic);
    node_handle.getParam("max_arm_cartesian_velocity", max_arm_cartesian_velocity);
    node_handle.getParam("max_arm_joint_velocity", max_arm_joint_velocity);

    if (!node_handle.getParam("/mcr_manipulation/mcr_arm_cartesian_control/root_name", root_name))
    {
        ROS_ERROR("No parameter for root_name specified");
    }
    ROS_INFO("Using %s as chain root [param: root_name]", root_name.c_str());

    if (!node_handle.getParam("/mcr_manipulation/mcr_arm_cartesian_control/tip_name", tooltip_name))
    {
        ROS_ERROR("No parameter for tip_name specified");
    }
    ROS_INFO("Using %s as tool tip [param: tip_name]", tooltip_name.c_str());

    tf_listener = new tf::TransformListener();
    loader.loadModel(node_handle, root_name, tooltip_name, arm_chain, joint_limits);
    nrOfJoints = arm_chain.getNrOfJoints();
    std::cout << "Nr of joints " << nrOfJoints << std::endl;
    joint_positions.resize(nrOfJoints);
    joint_positions_initialized.resize(nrOfJoints, false);
    init_ik_solver();

    sigma_publisher = node_handle.advertise<std_msgs::Float32MultiArray>(
                            sigma_values_topic, 1);

    cmd_vel_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(
                            velocity_command_topic, 1);
    
    sub_joint_state = node_handle.subscribe(joint_state_topic,
                                       1, &RosArmCartesianControl::jointstateCallback, this);

    sub_wjs = node_handle.subscribe(weight_js_topic, 1,
                             &RosArmCartesianControl::wjsCallback, this);
    
    sub_wts = node_handle.subscribe(weight_ts_topic, 1,
                             &RosArmCartesianControl::wtsCallback, this);

    sub_cc = node_handle.subscribe(cart_control_topic, 1,
                             &RosArmCartesianControl::ccCallback, this);

    sub_wrench = node_handle.subscribe(wrench_topic, 1,
                             &RosArmCartesianControl::wrenchCallback, this);

    sub_control_mode = node_handle.subscribe(control_mode_topic, 1,
                             &RosArmCartesianControl::cmCallback, this);
    

    control = new arm_cc::Arm_Cartesian_Control(&arm_chain, ik_solver);

    for (unsigned int i = 0; i < joint_limits.size(); i++)
    {
        upper_limits.push_back(joint_limits[i]->upper);
        lower_limits.push_back(joint_limits[i]->lower);
    }
    control->setJointLimits(lower_limits, upper_limits);

    cmd_velocities.resize(nrOfJoints);
    control->setJointVelLimit(max_arm_joint_velocity);
    control->setCartVelLimit(max_arm_cartesian_velocity);
    loop_rate = new ros::Rate(rate);
}

void RosArmCartesianControl::jointstateCallback(const sensor_msgs::JointState::ConstPtr& joints)
{

    for (unsigned i = 0; i < joints->position.size(); i++)
    {

        const char* joint_uri = joints->name[i].c_str();

        for (unsigned int j = 0; j < arm_chain.getNrOfJoints(); j++)
        {
            const char* chainjoint =
                arm_chain.getSegment(j).getJoint().getName().c_str();

            if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0)
            {
                joint_positions.data[j] = joints->position[i];
                joint_positions_initialized[j] = true;
            }
        }
    }
}

void RosArmCartesianControl::wtsCallback(std_msgs::Float32MultiArray weights)
{
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();
    weight_ts(0, 0) = weights.data[0];
    weight_ts(1, 1) = weights.data[1];
    weight_ts(2, 2) = weights.data[2];
    weight_ts(3, 3) = weights.data[3];
    weight_ts(4, 4) = weights.data[4];
    weight_ts(5, 5) = weights.data[5];
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);
}

void RosArmCartesianControl::wjsCallback(std_msgs::Float32MultiArray weights)
{
    weight_js.resize(nrOfJoints, nrOfJoints);
    weight_js.setIdentity();
    for (int i = 0; i < nrOfJoints; i++)
    {
        weight_js(i,i) = weights.data[i];
    }
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);
}

void RosArmCartesianControl::cmCallback(std_msgs::Int32MultiArray control_modes)
{
    for (int i = 0; i < 6; i++)
    {
        axis_control_modes[i] = control_modes.data[i];
    }
}

void RosArmCartesianControl::wrenchCallback(geometry_msgs::WrenchStampedConstPtr wr)
{
    wrench = *wr;
}

void RosArmCartesianControl::ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity)
{

    for (size_t i = 0; i < joint_positions_initialized.size(); i++)
    {
        if (!joint_positions_initialized[i])
        {
            std::cout << "joints not initialized" << std::endl;
            return;
        }
    }

    if (!tf_listener) return;

    geometry_msgs::Vector3Stamped linear_in;
    geometry_msgs::Vector3Stamped linear_out;
    linear_in.header = desiredVelocity->header;
    linear_in.vector = desiredVelocity->twist.linear;

    try
    {
        tf_listener->transformVector(root_name, linear_in, linear_out);
    }
    catch (...)
    {
        ROS_ERROR("Could not transform frames %s -> %s for linear transformation", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
        return;
    }
    geometry_msgs::Vector3Stamped angular_in;
    geometry_msgs::Vector3Stamped angular_out;
    angular_in.header = desiredVelocity->header;
    angular_in.vector = desiredVelocity->twist.angular;

    try
    {
        tf_listener->transformVector(root_name, angular_in, angular_out);
    }
    catch (...)
    {
        ROS_ERROR("Could not transform frames %s -> %s for angular transformation", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
        return;
    }
    targetVelocity.vel.data[0] = linear_out.vector.x;
    targetVelocity.vel.data[1] = linear_out.vector.y;
    targetVelocity.vel.data[2] = linear_out.vector.z;

    targetVelocity.rot.data[0] = angular_out.vector.x;
    targetVelocity.rot.data[1] = angular_out.vector.y;
    targetVelocity.rot.data[2] = angular_out.vector.z;

    t_last_command = ros::Time::now();
    active = true;
}


void RosArmCartesianControl::init_ik_solver()
{

    ik_solver = new KDL::ChainIkSolverVel_wdls(arm_chain);
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();

    weight_ts(0, 0) = 1;
    weight_ts(1, 1) = 1;
    weight_ts(2, 2) = 1;
    weight_ts(3, 3) = 1;
    weight_ts(4, 4) = 1;
    weight_ts(5, 5) = 1;
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);

    weight_js = (Eigen::MatrixXd::Identity(arm_chain.getNrOfJoints(),
                                           arm_chain.getNrOfJoints()));
    weight_js(0, 0) = 1;
    weight_js(1, 1) = 1;
    weight_js(2, 2) = 1;
    weight_js(3, 3) = 1;
    weight_js(4, 4) = 1;
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);


    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setLambda(10000.0);
}


void RosArmCartesianControl::publishJointVelocities_FA(KDL::JntArrayVel& joint_velocities)
{
    std_msgs::Float64MultiArray joint_velocitiy_array;
    joint_velocitiy_array.data.clear();
    for (unsigned int i = 0; i < joint_velocities.qdot.rows(); i++)
    {
        joint_velocitiy_array.data.push_back(joint_velocities.qdot(i));
        if (std::isnan(joint_velocities.qdot(i)))
        {
            ROS_ERROR("invalid joint velocity: nan");
            return;
        }
        if (fabs(joint_velocities.qdot(i)) > 1.0)
        {
            ROS_ERROR("invalid joint velocity: too fast");
            return;
        }
    }
    motion_stopped = false;
    cmd_vel_publisher.publish(joint_velocitiy_array);
}


void RosArmCartesianControl::stopMotion()
{
    if (!motion_stopped)
    {
    	std_msgs::Float64MultiArray joint_velocitiy_array;
    	joint_velocitiy_array.data.clear();
    	for (unsigned int i = 0; i < nrOfJoints; i++)
    	{
            joint_velocitiy_array.data.push_back(0.0);
    	}
    	cmd_vel_publisher.publish(joint_velocitiy_array);
    	motion_stopped = true;
    }
}


bool RosArmCartesianControl::watchdog()
{

    double watchdog_time = 0.3;
    if (active == false)
    {
        return false;
    }

    ros::Time now = ros::Time::now();

    ros::Duration time = (now - t_last_command);

    if (time > ros::Duration(watchdog_time))
    {
        active = false;
        stopMotion();
        return false;

    }

    return true;
}


// Compliant control 


void RosArmCartesianControl::force_torque_control()
{
    if (axis_control_modes[0] == 0)
    {
        if (fabs(wrench.wrench.force.x) > wrench_threshold_x)
        {
            targetVelocity.vel.data[0] = (wrench.wrench.force.x) * linear_vel_const; 
        }
    }
    if (axis_control_modes[1] == 0)
    {
        if (fabs(wrench.wrench.force.y) > wrench_threshold_y)
        {
            targetVelocity.vel.data[1] = (wrench.wrench.force.y) * linear_vel_const; 
        }
    }
    if (axis_control_modes[2] == 0)
    {
        if (fabs(wrench.wrench.force.z) > wrench_threshold_z)
        {
            targetVelocity.vel.data[2] = (wrench.wrench.force.z) * linear_vel_const; 
        }
    }
    if (axis_control_modes[3] == 0)
    {
        if (fabs(wrench.wrench.torque.x) > wrench_threshold_ax)
        {
            targetVelocity.rot.data[0] = (wrench.wrench.torque.x) * angular_vel_const; 
        }
    }
    if (axis_control_modes[4] == 0)
    {
        if (fabs(wrench.wrench.torque.y) > wrench_threshold_ay)
        {
            targetVelocity.rot.data[1] = (wrench.wrench.torque.y) * angular_vel_const; 
        }
    }
    if (axis_control_modes[5] == 0)
    {
        if (fabs(wrench.wrench.torque.z) > wrench_threshold_az)
        {
            targetVelocity.rot.data[2] = (wrench.wrench.torque.z) * angular_vel_const; 
        }
    }
}

void RosArmCartesianControl::control_loop()
{
    
    while (ros::ok())
    {
        ros::spinOnce();
        if (watchdog())
        {
            control->process(1 / rate, joint_positions, targetVelocity, cmd_velocities, sigma);
            
            sigma_array.data.clear();           
            publishJointVelocities_FA(cmd_velocities);
        }
        loop_rate->sleep();
    }

}

