#ifndef OROCOS_ROS_JOINT_STATE_PUBLISHER_H
#define OROCOS_ROS_JOINT_STATE_PUBLISHER_H

// RTT header files. Might missing some or some be unused
#include <rtt/RTT.hpp>
#include <string>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>


class orocos_ros_joint_state_publisher: public RTT::TaskContext {
public:
    orocos_ros_joint_state_publisher(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    bool loadURDFAndSRDF(const std::string& URDF_path, const std::string& SRDF_path);
    bool attachToRobot(const std::string& robot_name);
    std::string _urdf_path;
    std::string _srdf_path;
    boost::shared_ptr<urdf::Model> _urdf_model;
    std::vector<std::string> _joint_list;
    std::string _robot_name;
    sensor_msgs::JointState _joint_state_msg;
    std::map<std::string, geometry_msgs::WrenchStamped> _wrench_msgs;

    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;
    std::vector<std::string> _force_torque_sensors_frames;

    RTT::OutputPort<sensor_msgs::JointState> _joint_state_port;
    std::map<std::string, boost::shared_ptr<RTT::OutputPort<geometry_msgs::WrenchStamped> > > _wrench_ports;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
    std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::dynamics::Wrench> > > _frames_ports_map;
    std::map<std::string, rstrt::dynamics::Wrench> _frames_wrenches_map;
};

#endif // OROCOS_ROS_JOINT_STATE_PUBLISHER_H
