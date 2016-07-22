/* Author: Enrico Mingo Hoffman
 * Date:   16/06/2016
 *
 * Description: Based on the simple orocos/rtt component template by Pouya Mohammadi
 */

#include "orocos_ros_joint_state_publisher.h"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>
#include <rtt/Operation.hpp>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/OperationCaller.hpp>


orocos_ros_joint_state_publisher::orocos_ros_joint_state_publisher(std::string const & name):
    RTT::TaskContext(name),
    _urdf_path(""),
    _srdf_path(""),
    _joint_state_port("joint_states_orocos_port")
{
    this->setActivity(new RTT::Activity(1, 0.02));

    this->addOperation("loadURDFAndSRDF", &orocos_ros_joint_state_publisher::loadURDFAndSRDF,
                this, RTT::ClientThread);
    this->addOperation("attachToRobot", &orocos_ros_joint_state_publisher::attachToRobot,
                this, RTT::ClientThread);

    _urdf_model.reset(new urdf::Model());

    this->addPort(_joint_state_port).doc("Joint State for ROS");
}

bool orocos_ros_joint_state_publisher::configureHook()
{
    _joint_state_msg.name = _joint_list;
    for(unsigned int i = 0; i < _joint_list.size(); ++i)
    {
        _joint_state_msg.position.push_back(0.0);
        _joint_state_msg.effort.push_back(0.0);
        _joint_state_msg.velocity.push_back(0.0);
    }
    RTT::log(RTT::Info)<<"joint_state_msg has been initialized"<<RTT::endlog();

    return true;
}

bool orocos_ros_joint_state_publisher::startHook()
{
    _joint_state_port.createStream(rtt_roscomm::topic("joint_states"));

    return true;
}

void orocos_ros_joint_state_publisher::updateHook()
{
    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        RTT::FlowStatus fs = _kinematic_chains_feedback_ports.at(it->first)->read(
                    _kinematic_chains_joint_state_map.at(it->first));

        for(unsigned int i = 0; i < it->second.size(); ++i){
            int pos = find(_joint_state_msg.name.begin(), _joint_state_msg.name.end(),
                           it->second[i]) - _joint_state_msg.name.begin();
            _joint_state_msg.position[pos] =
                    _kinematic_chains_joint_state_map.at(it->first).angles[i];
            _joint_state_msg.velocity[pos] =
                    _kinematic_chains_joint_state_map.at(it->first).velocities[i];
            _joint_state_msg.effort[pos] =
                    _kinematic_chains_joint_state_map.at(it->first).torques[i];
        }
    }


    _joint_state_msg.header.stamp = rtt_rosclock::host_now();
    _joint_state_port.write(_joint_state_msg);

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

    RTT::TaskContext* task_ptr = this->getPeer(robot_name);
    if(!task_ptr){
        RTT::log(RTT::Error)<<"Can not getPeer("<<robot_name<<")"<<RTT::endlog();
        return false;}

    RTT::log(RTT::Info)<<"Found Peer "<<robot_name<<RTT::endlog();

    RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints
        = task_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;

        _kinematic_chains_feedback_ports[kin_chain_name] =
            boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> >(
                        new RTT::InputPort<rstrt::robot::JointState>(
                            kin_chain_name+"_"+"JointFeedback"));
        this->addPort(*(_kinematic_chains_feedback_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointFeedback port");

        _kinematic_chains_feedback_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointFeedback"));

        rstrt::robot::JointState tmp(joint_names.size());
        _kinematic_chains_joint_state_map[kin_chain_name] = tmp;
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data"<<RTT::endlog();
    }

    return true;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(orocos_ros_joint_state_publisher)
