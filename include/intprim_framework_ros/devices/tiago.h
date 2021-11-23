#pragma once
#include "device_interface.h"
#include "robot_interface.h"
#include <vector>
#include "ros/ros.h"

#include <sensor_msgs/JointState.h>

class TiagoState : public DeviceState
{
public:
    static constexpr unsigned int           NUM_JOINTS = 7;
    static std::string                      JOINT_NAMES[NUM_JOINTS];
    static double                           JOINT_THRESHOLD;

    TiagoState();

    void to_matrix(std::vector<float>& trajectory) const;

    const sensor_msgs::JointState::ConstPtr& get_message();

    bool valid() const;

    void set_message(const sensor_msgs::JointState::ConstPtr& message);

    bool within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const;

private:
    sensor_msgs::JointState::ConstPtr           m_message;
    bool                                        m_valid;
};

class TiagoInterface : public RobotInterface
{
public:
    TiagoInterface(ros::NodeHandle handle, std::string robot_type);

    // Covariant return type.
    const TiagoState& get_state();

    void publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx);
private:
    static double                               CONTROL_TIME_BUFFER;
    static int                                  CONTROL_FREQUENCY;
    static double                               MAX_ACCELERATION;

    ros::Subscriber                             m_state_subscriber;
    ros::Publisher                              m_state_publisher;

    TiagoState                                  m_current_state;
    sensor_msgs::JointState                     m_publish_message;

    unsigned int                                m_control_frequency;
    float                                       m_message_time;   //TODO seems not used before

    void state_callback(const sensor_msgs::JointState::ConstPtr& message);
};
