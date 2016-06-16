/* Author: Enrico Mingo Hoffman
 * Date:   16/06/2016
 *
 * Description: Based on the simple orocos/rtt component template by Pouya Mohammadi
 */

#include "orocos_ros_joint_state_publisher.h"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>
#include <rtt/Operation.hpp>


orocos_ros_joint_state_publisher::orocos_ros_joint_state_publisher(std::string const & name):
    RTT::TaskContext(name),
    _urdf_path(""),
    _srdf_path("")
{
    this->addOperation("loadURDFAndSRDF", &orocos_ros_joint_state_publisher::loadURDFAndSRDF,
                this, RTT::ClientThread);
    this->addOperation("attachToRobot", &orocos_ros_joint_state_publisher::attachToRobot,
                this, RTT::ClientThread);

    _urdf_model.reset(new urdf::Model());
}

bool orocos_ros_joint_state_publisher::configureHook()
{
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool orocos_ros_joint_state_publisher::startHook()
{
    // this method starts the component
    return true;
}

void orocos_ros_joint_state_publisher::updateHook()
{
    // this is the actual body of a component. it is called on each cycle
}

void orocos_ros_joint_state_publisher::stopHook()
{
    // stops the component (update hook wont be  called anymore)
}

void orocos_ros_joint_state_publisher::cleanupHook()
{
    // cleaning the component data
}

bool orocos_ros_joint_state_publisher::loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path)
{
    _urdf_path = URDF_path;
    _srdf_path = SRDF_path;

    RTT::log(RTT::Info)<<"URDF path: "<<_urdf_path<<RTT::endlog();
    RTT::log(RTT::Info)<<"SRDF path: "<<_srdf_path<<RTT::endlog();

    bool models_loaded = _urdf_model->initFile(_urdf_path);

    if(models_loaded)
    {
        RTT::log(RTT::Info)<<"Model name: "<<_urdf_model->getName()<<RTT::endlog();

        std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map = _urdf_model->joints_;
        std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
        RTT::log(RTT::Info)<<"Joints: "<<RTT::endlog();
        for(it = joint_map.begin(); it != joint_map.end(); it++)
        {
            if(it->second->type != urdf::Joint::FIXED){
                _joint_list.push_back(it->first);
                RTT::log(RTT::Info)<<"  "<<_joint_list.back()<<RTT::endlog();}
        }
        RTT::log(RTT::Info)<<"Total number of joints is "<<_joint_list.size()<<RTT::endlog();
    }

    return models_loaded;
}

bool orocos_ros_joint_state_publisher::attachToRobot(const std::string &robot_name)
{
    _robot_name = robot_name;
    RTT::log(RTT::Info)<<"Robot name: "<<_robot_name<<RTT::endlog();
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(orocos_ros_joint_state_publisher)
