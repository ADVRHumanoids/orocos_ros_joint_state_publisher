/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#include "simple-rtt-component.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>

RttComponent::RttComponent(std::string const & name) : RTT::TaskContext(name) {
    // constructor:
    
}

bool RttComponent::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool RttComponent::startHook() {
    // this method starts the component
    return true;
}

void RttComponent::updateHook() {
    // this is the actual body of a component. it is called on each cycle
}

void RttComponent::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void RttComponent::cleanupHook() {
    // cleaning the component data
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RttComponent)
