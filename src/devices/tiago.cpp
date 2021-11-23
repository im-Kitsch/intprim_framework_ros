#include "devices/tiago.h"
#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

std::string TiagoState::JOINT_NAMES[TiagoState::NUM_JOINTS];
double TiagoState::JOINT_THRESHOLD = 0.0;    // TODO, check if it is proper here
//double Robot1Interface::CONTROL_TIME_BUFFER = 0.0;  // TODO, seems not used anymore here
//int    Robot1Interface::CONTROL_FREQUENCY = 0;
//double Robot1Interface::MAX_ACCELERATION = 0.0;

TiagoState::TiagoState() :
    m_message(),
    m_valid(true) //FIXME, check it here
{

}

void TiagoState::to_matrix(std::vector<float>& trajectory) const
{
    if(m_message)   // TODO, seems it's for pointer null or not   //TODO check if valid
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(m_message->position[joint_idx]);
        }
    }
    else
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(0.0);
        }
    }
}

const sensor_msgs::JointState::ConstPtr& TiagoState::get_message()
{
    return m_message;
}

void TiagoState::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true; //TODO, seems not used
}

bool TiagoState::valid() const
{
    return m_valid;
    // return true;
}

bool TiagoState::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            if(std::abs(m_message->position[joint_idx] - trajectory[joint_idx + trajectory_idx]) > JOINT_THRESHOLD)
            {
                return false;
            }
        }
        return true;
    }
    return false;
}


TiagoInterface::TiagoInterface(ros::NodeHandle handle, std::string robot_type) :
    m_state_subscriber(),
    m_state_publisher(),
    m_current_state(),
    m_publish_message()
{   
    std::cout << "robot type is" << robot_type.c_str() << std::endl;
    // set joint names, names specified in roslaunch
    for(int joint = 0; joint < TiagoState::NUM_JOINTS; joint++)
    {
        std::string name;
        if(robot_type.compare("control_sim") == 0)
        {            
            name = "tiago_c_joint" + std::to_string(joint+1); 
            TiagoState::JOINT_NAMES[joint] = name;
        }
        else if(robot_type.compare("observe_sim") == 0)
        {
            name = "tiago_l_joint" + std::to_string(joint+1);
            TiagoState::JOINT_NAMES[joint] = name;
        }
        else
        {
            //TODO, seems not used....
            name = "tiago_joint" + std::to_string(joint+1);
            TiagoState::JOINT_NAMES[joint] = name;
        }
        m_publish_message.name.push_back(name);
        m_publish_message.position.push_back(0.0);  // TODO Check it here
    }
    
    // init subscriber/advertiser
    if(robot_type.compare("control_sim") == 0)
    {
        handle.getParam("control/tiago_c/joint_distance_threshold", TiagoState::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/tiago_c/control", 1);
        m_state_subscriber = handle.subscribe("/robot/tiago_c/state", 1, &TiagoInterface::state_callback, this);
    }
    else if(robot_type.compare("observe_sim") == 0)
    {
        handle.getParam("control/tiago_l/joint_distance_threshold", TiagoState::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/tiago_l/control", 1);
        m_state_subscriber = handle.subscribe("/robot/tiago_l/state", 1, &TiagoInterface::state_callback, this);
    }
    else{
        handle.getParam("control/tiago/joint_distance_threshold", TiagoState::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/tiago/control", 1);
        m_state_subscriber = handle.subscribe("/robot/tiago/state", 1, &TiagoInterface::state_callback, this);
    }
}
//{
////    handle.getParam("control/robot1/control_time_buffer", CONTROL_TIME_BUFFER);
////    handle.getParam("control/robot1/control_frequency", CONTROL_FREQUENCY);
////    handle.getParam("control/robot1/max_acceleration", MAX_ACCELERATION);
////    handle.getParam("control/robot1/joint_distance_threshold", TiagoState::JOINT_THRESHOLD);
////
////    std::cout << "control time buffer: " << CONTROL_TIME_BUFFER << ", control frequency: " << CONTROL_FREQUENCY << ", max accel: " << MAX_ACCELERATION << ", joint thresh: " << Robot1State::JOINT_THRESHOLD << std::endl;
//
////    m_messageTime = (1.0 / CONTROL_FREQUENCY) * CONTROL_TIME_BUFFER;
//
////    m_publishMessage.acceleration = MAX_ACCELERATION;
////    m_publishMessage.blend = 0;
////    m_publishMessage.command = "speed";
////    m_publishMessage.gain = 0;
////    m_publishMessage.jointcontrol = true;
////    m_publishMessage.lookahead = 0;
////    m_publishMessage.time = m_messageTime;
////    m_publishMessage.velocity = 0;
//
////    //TODO, maybe from interaction.launch
////    m_statePublisher = handle.advertise<your_robots::robot1Control>("/robot1/control", 1);
////    m_stateSubscriber = handle.subscribe("/robot1/joints", 1, &Robot1Interface::state_callback, this);
//}

const TiagoState& TiagoInterface::get_state()
{
    return m_current_state;
}

void TiagoInterface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < TiagoState::NUM_JOINTS; ++joint_idx)
    {
        m_publish_message.position[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_state_publisher.publish(m_publish_message);
}

void TiagoInterface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_current_state.set_message(message);
}





