/*
 * robomagellan_interface.hpp
 *
 * Class definition of the hardware interface to the real robot.
 */
#ifndef __ROBOMAGELLAN_INTERFACE_HPP__
#define __ROBOMAGELLAN_INTERFACE_HPP__

#include <hardware_interface/system_interface.hpp>

namespace robomagellan_firmware {

// System interfaces inherit from the lifecycle node interfaces
class RobomagellanInterface : public hardware_interface::SystemInterface {

}; // End class RobomagellanInterface

} // End namespace robomagellan_firmware

#endif
