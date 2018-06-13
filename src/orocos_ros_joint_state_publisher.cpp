/* Author: Enrico Mingo Hoffman
 * Date:   16/06/2016
 *
 * Description: Based on the simple orocos/rtt component template by Pouya Mohammadi
 */

#include "orocos_ros_joint_state_publisher.h"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>
#include <rtt/Operation.hpp>
#include <rtt_roscomm/rostopic.h>
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

    std::map<std::string, boost::shared_ptr<RTT::OutputPort<geometry_msgs::WrenchStamped> > >::iterator it;
    for(it = _wrench_ports.begin(); it != _wrench_ports.end(); it++)
    {
        this->addPort(*(it->second)).doc(it->first + " State for ROS");
        it->second->createStream(rtt_roscomm::topic(it->first + "_force_torque_sensor"));
    }

    std::map<std::string, boost::shared_ptr<RTT::OutputPort<sensor_msgs::Imu> > >::iterator it2;
    for(it2 = _imu_ports.begin(); it2 != _imu_ports.end(); it2++)
    {
        this->addPort(*(it2->second)).doc(it2->first + "State for ROS");
        it2->second->createStream(rtt_roscomm::topic(it2->first + "_imu_sensor"));
    }

    return true;
}

void orocos_ros_joint_state_publisher::updateHook()
{
    ros::Time tick = rtt_rosclock::host_now();

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
    _joint_state_msg.header.stamp = tick;
    _joint_state_port.write(_joint_state_msg);

    std::map<std::string, rstrt::dynamics::Wrench>::iterator it2;
    for(it2 = _frames_wrenches_map.begin(); it2 != _frames_wrenches_map.end(); it2++)
    {
        RTT::FlowStatus fs = _frames_ports_map.at(it2->first)->read(
                    _frames_wrenches_map.at(it2->first));

        _wrench_msgs.at(it2->first).wrench.force.x = _frames_wrenches_map.at(it2->first).forces[0];
        _wrench_msgs.at(it2->first).wrench.force.y = _frames_wrenches_map.at(it2->first).forces[1];
        _wrench_msgs.at(it2->first).wrench.force.z = _frames_wrenches_map.at(it2->first).forces[2];
        _wrench_msgs.at(it2->first).wrench.torque.x = _frames_wrenches_map.at(it2->first).torques[0];
        _wrench_msgs.at(it2->first).wrench.torque.y = _frames_wrenches_map.at(it2->first).torques[1];
        _wrench_msgs.at(it2->first).wrench.torque.z = _frames_wrenches_map.at(it2->first).torques[2];

        _wrench_msgs.at(it2->first).header.frame_id = it2->first;
        _wrench_msgs.at(it2->first).header.stamp = tick;

        _wrench_ports.at(it2->first)->write(_wrench_msgs.at(it2->first));
    }

    std::map<std::string, rstrt::robot::IMU>::iterator it3;
    for(it3 = _frames_imu_map.begin(); it3 != _frames_imu_map.end(); it3++)
    {
        RTT::FlowStatus fs = _imu_frames_ports_map.at(it3->first)->read(
                    _frames_imu_map.at(it3->first));

        _imu_msgs.at(it3->first).angular_velocity.x = _frames_imu_map.at(it3->first).angularVelocity[0];
        _imu_msgs.at(it3->first).angular_velocity.y = _frames_imu_map.at(it3->first).angularVelocity[1];
        _imu_msgs.at(it3->first).angular_velocity.z = _frames_imu_map.at(it3->first).angularVelocity[2];

        _imu_msgs.at(it3->first).linear_acceleration.x = _frames_imu_map.at(it3->first).linearAcceleration[0];
        _imu_msgs.at(it3->first).linear_acceleration.y = _frames_imu_map.at(it3->first).linearAcceleration[1];
        _imu_msgs.at(it3->first).linear_acceleration.z = _frames_imu_map.at(it3->first).linearAcceleration[2];

        _imu_msgs.at(it3->first).orientation.w = _frames_imu_map.at(it3->first).rotation[0];
        _imu_msgs.at(it3->first).orientation.x = _frames_imu_map.at(it3->first).rotation[1];
        _imu_msgs.at(it3->first).orientation.y = _frames_imu_map.at(it3->first).rotation[2];
        _imu_msgs.at(it3->first).orientation.z = _frames_imu_map.at(it3->first).rotation[3];

        _imu_msgs.at(it3->first).header.frame_id = it3->first;
        _imu_msgs.at(it3->first).header.stamp = tick;

        _imu_ports.at(it3->first)->write(_imu_msgs.at(it3->first));
    }

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
            if(it->second->type != urdf::Joint::FIXED && it->second->type != urdf::Joint::FLOATING){
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


    RTT::OperationCaller<std::vector<std::string> (void) > getForceTorqueSensorsFrames
        = task_ptr->getOperation("getForceTorqueSensorsFrames");
    std::vector<std::string> ft_sensors_frames = getForceTorqueSensorsFrames();
    for(unsigned int i = 0; i < ft_sensors_frames.size(); ++i)
    {
        _frames_ports_map[ft_sensors_frames[i]] =
                boost::shared_ptr<RTT::InputPort<rstrt::dynamics::Wrench> >(
                    new RTT::InputPort<rstrt::dynamics::Wrench>(
                        ft_sensors_frames[i]+"_SensorFeedback"));
        this->addPort(*(_frames_ports_map.at(ft_sensors_frames[i]))).
                doc(ft_sensors_frames[i]+"_SensorFeedback port");

        _frames_ports_map.at(ft_sensors_frames[i])->connectTo(
                    task_ptr->ports()->getPort(ft_sensors_frames[i]+"_SensorFeedback"));

        rstrt::dynamics::Wrench tmp;
        _frames_wrenches_map[ft_sensors_frames[i]] = tmp;

        geometry_msgs::WrenchStamped tmp2;
        tmp2.wrench.force.x = 0.; tmp2.wrench.torque.x = 0.;
        tmp2.wrench.force.y = 0.; tmp2.wrench.torque.y = 0.;
        tmp2.wrench.force.z = 0.; tmp2.wrench.torque.z = 0.;
        _wrench_msgs[ft_sensors_frames[i]] = tmp2;

        _wrench_ports[ft_sensors_frames[i]] =
                boost::shared_ptr<RTT::OutputPort<geometry_msgs::WrenchStamped> >(
                    new RTT::OutputPort<geometry_msgs::WrenchStamped>(
                        ft_sensors_frames[i]+"_orocos_port"));
        RTT::log(RTT::Info)<<"Added "<<ft_sensors_frames[i]<<" port and data"<<RTT::endlog();
    }

    RTT::OperationCaller<std::vector<std::string> (void) > getIMUSensorsFrames
        = task_ptr->getOperation("getIMUSensorsFrames");
    std::vector<std::string> imu_sensors_frames = getIMUSensorsFrames();
    for(unsigned int i = 0; i < imu_sensors_frames.size(); ++i)
    {
        _imu_frames_ports_map[imu_sensors_frames[i]] =
                boost::shared_ptr<RTT::InputPort<rstrt::robot::IMU> >(
                    new RTT::InputPort<rstrt::robot::IMU>(
                        imu_sensors_frames[i]+"_SensorFeedback"));
        this->addPort(*(_imu_frames_ports_map.at(imu_sensors_frames[i]))).
                doc(imu_sensors_frames[i]+"_SensorFeedback port");

        _imu_frames_ports_map.at(imu_sensors_frames[i])->connectTo(
                    task_ptr->ports()->getPort(imu_sensors_frames[i]+"_SensorFeedback"));

        rstrt::robot::IMU tmp;
        _frames_imu_map[imu_sensors_frames[i]] = tmp;

        sensor_msgs::Imu tmp2;
        tmp2.angular_velocity.x = 0.; tmp2.angular_velocity.y = 0.; tmp2.angular_velocity.z = 0.;
        tmp2.linear_acceleration.x = 0.; tmp2.linear_acceleration.y = 0.; tmp2.linear_acceleration.z = 0.;
        tmp2.orientation.w = 1.; tmp2.orientation.x = 0.; tmp2.orientation.y = 0.; tmp2.orientation.z = 0.;
        _imu_msgs[imu_sensors_frames[i]] = tmp2;

        _imu_ports[imu_sensors_frames[i]] =
                boost::shared_ptr<RTT::OutputPort<sensor_msgs::Imu> >(
                    new RTT::OutputPort<sensor_msgs::Imu>(
                        imu_sensors_frames[i]+"_orocos_port"));
        RTT::log(RTT::Info)<<"Added "<<imu_sensors_frames[i]<<" port and data"<<RTT::endlog();
    }

    return true;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(orocos_ros_joint_state_publisher)
