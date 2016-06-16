#ifndef OROCOS_ROS_JOINT_STATE_PUBLISHER_H
#define OROCOS_ROS_JOINT_STATE_PUBLISHER_H

// RTT header files. Might missing some or some be unused
#include <rtt/RTT.hpp>
#include <string>
#include <rst-rt/robot/JointState.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <urdf/model.h>

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
    boost::shared_ptr<urdf::Model> _urdf_model; // A URDF Model
    std::vector<std::string> _joint_list;
    std::string _robot_name;
};

#endif // OROCOS_ROS_JOINT_STATE_PUBLISHER_H