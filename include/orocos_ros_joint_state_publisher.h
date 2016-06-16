/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#ifndef SIMPLERTTCOMPONENT_HPP
#define SIMPLERTTCOMPONENT_HPP

// RTT header files. Might missing some or some be unused
#include <rtt/RTT.hpp>
#include <string>

#define ROBOT_DOF_SIZE 7

class RttComponent: public RTT::TaskContext {
public:
    RttComponent(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
	// Declare ports and their datatypes

};

#endif // SIMPLERTTCOMPONENT_HPP
