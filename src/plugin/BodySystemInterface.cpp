#include "BodySystemInterface.h"
#include "ROS2ControlItem.h"
#include "Format.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/MessageOut>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "gettext.h"

using namespace cnoid;
using namespace hardware_interface;


BodySystemInterface::BodySystemInterface()
{
    item = nullptr;
}


BodySystemInterface::BodySystemInterface(ROS2ControlItem* item)
    : item(item)
{

}


hardware_interface::CallbackReturn BodySystemInterface::on_init(const hardware_interface::HardwareInfo& info)
{
    if(!item){
        return CallbackReturn::ERROR;
    }
        
    auto mout = MessageOut::master();

    // copy the HardwareInfo as `info_ = hardware_info;`
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    const int numJoints = info_.joints.size();

    states.resize(numJoints, {0.0, 0.0, 0.0});
    commands.resize(numJoints, 0);
    controlTypes.resize(numJoints, ControlMode::UNDEFINED);
    gains.resize(numJoints, {0.0, 0.0});

    for (int i = 0; i < info_.joints.size(); ++i) {
        const ComponentInfo& joint = info_.joints[i];
        const std::string jointName = joint.name;

        // set control types from robot description
        if (joint.command_interfaces.size() != 1) {
            mout->putErrorln(
                formatR(_("joint {} must have one command interface"), jointName));
            return CallbackReturn::ERROR;
        }

        const std::string controlTypeString = joint.command_interfaces[0].name;
        if (controlTypeString == "position") {
            controlTypes[i] = ControlMode::POSITION;
        } else if (controlTypeString == "velocity") {
            controlTypes[i] = ControlMode::VELOCITY;
        } else if (controlTypeString == "effort") {
            controlTypes[i] = ControlMode::EFFORT;
        } else if (controlTypeString == "position_pd") {
            controlTypes[i] = ControlMode::POSITION_PD;
        } else if (controlTypeString == "velocity_pd") {
            controlTypes[i] = ControlMode::VELOCITY_PD;
        } else {
            mout->putErrorln(
                formatR(_("the command interface of joint {} is invalid"), jointName));
            return CallbackReturn::ERROR;
        }

        mout->putln(formatR(_("{}:"), jointName));
        mout->putln(formatR(_("\tcontrol type: {}"), controlTypeString));

        // if controlTypes[i] is POSITION, VELOCITY, or EFFORT, skip gain settings
        if (controlTypes[i] <= ControlMode::EFFORT) {
            continue;
        }

        // declare gain parameters
        // parameter names are compatible with the PID Controller in ros2_controllers
        const std::string paramNameP = "gains." + jointName + ".p";
        const std::string paramNameD = "gains." + jointName + ".d";

        auto declareParam = [&](const std::string paramName) {
            try {
                item->node()->declare_parameter<double>(paramName, 0.0);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
                mout->putErrorln(formatR(_("{}"), e.what()));
            } catch (rclcpp::exceptions::InvalidParameterValueException& e) {
                mout->putErrorln(formatR(_("{}"), e.what()));
            } catch (rclcpp::exceptions::InvalidParameterTypeException& e) {
                mout->putErrorln(formatR(_("{}"), e.what()));
            }
        };

        declareParam(paramNameP);
        declareParam(paramNameD);

        // get gain parameters
        auto getParam = [&](const std::string& paramName) -> double {
            try {
                return item->node()->get_parameter(paramName).as_double();
            } catch (rclcpp::exceptions::ParameterNotDeclaredException& e) {
                // this error occurs if the above declaration failed
                // mout->putErrorln(formatR(_("{}"), e.what()));
                return 0.0;
            } catch (rclcpp::exceptions::InvalidParameterValueException& e) {
                mout->putErrorln(formatR(_("{}"), e.what()));
                return 0.0;
            } catch (rclcpp::exceptions::InvalidParameterTypeException& e) {
                mout->putErrorln(formatR(_("{}"), e.what()));
                return 0.0;
            }
        };

        gains[i].p = getParam(paramNameP);
        gains[i].d = getParam(paramNameD);

        mout->putln(formatR(_("\tP gain: {}"), gains[i].p));
        if (gains[i].p <= 0.0) {
            mout->putWarningln(formatR(_("\tP gain should be positive")));
        }
        mout->putln(formatR(_("\tD gain: {}"), gains[i].d));
        if (gains[i].d < 0.0) {
            mout->putErrorln(formatR(_("\tP gain must be non-negative")));
        }
    }

    return CallbackReturn::SUCCESS;
}


CallbackReturn BodySystemInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


std::vector<StateInterface> BodySystemInterface::export_state_interfaces()
{
    std::vector<StateInterface> stateInterfaces;
    stateInterfaces.reserve(3 * info_.joints.size());
    for (int i = 0; i < info_.joints.size(); ++i) {
        const std::string& jointName = info_.joints[i].name;
        stateInterfaces.emplace_back(StateInterface(
        jointName, hardware_interface::HW_IF_POSITION, &states[i].position));
        stateInterfaces.emplace_back(StateInterface(
        jointName, hardware_interface::HW_IF_VELOCITY, &states[i].velocity));
        stateInterfaces.emplace_back(StateInterface(
        jointName, hardware_interface::HW_IF_EFFORT, &states[i].effort));
    }

    return stateInterfaces;
}


std::vector<hardware_interface::CommandInterface> BodySystemInterface::export_command_interfaces()
{
    std::vector<CommandInterface> commandInterfaces;
    commandInterfaces.reserve(info_.joints.size());
    for (int i = 0; i < info_.joints.size(); ++i) {
        const std::string jointName = info_.joints[i].name;
        switch (controlTypes[i]) {
            case ControlMode::POSITION:
            case ControlMode::POSITION_PD:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_POSITION, &commands[i]));
                break;
            case ControlMode::VELOCITY:
            case ControlMode::VELOCITY_PD:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_VELOCITY, &commands[i]));
                break;
            case ControlMode::EFFORT:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_EFFORT, &commands[i]));
                break;
            default:
                // TODO: error! set actuation_mode of joint {0} in the body description file
                break;
        }
    }

    return commandInterfaces;
}


hardware_interface::CallbackReturn BodySystemInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    auto body = item->bodyItem()->body();
    
    for (int i = 0; i < info_.joints.size(); ++i) {
        Link* joint = body->joint(info_.joints[i].name);
        if (!joint) {
            MessageOut::master()->putErrorln(
                formatR(_("joint {} is not found in the simulation body"), info_.joints[i].name));
            return CallbackReturn::ERROR;
        }

        states[i].position = joint->q();
        states[i].velocity = joint->dq();
        states[i].effort = joint->u();

        switch (controlTypes[i]) {
            case ControlMode::POSITION:
                commands[i] = joint->q();
                joint->setActuationMode(Link::JointDisplacement);
                break;
            case ControlMode::VELOCITY:
                    commands[i] = joint->dq();
                    joint->setActuationMode(Link::JointVelocity);
                    break;
            case ControlMode::EFFORT:
                commands[i] = joint->u();
                joint->setActuationMode(Link::JointEffort);
                break;
            case ControlMode::POSITION_PD:
                commands[i] = joint->q();
                joint->setActuationMode(Link::JointEffort);
                break;
            case ControlMode::VELOCITY_PD:
                commands[i] = joint->dq();
                joint->setActuationMode(Link::JointEffort);
                break;
            default:
                // TODO: output error message
                return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn BodySystemInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


hardware_interface::return_type BodySystemInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    auto ioBody = item->io()->body();
    
    // copy joint states from the simulation body
    for (int i = 0; i < info_.joints.size(); ++i) {
        const Link* joint = ioBody->joint(info_.joints[i].name);

        states[i].position = joint->q();
        states[i].velocity = joint->dq();
        states[i].effort = joint->u();
    }

    return return_type::OK;
}


hardware_interface::return_type BodySystemInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    auto ioBody = item->io()->body();

    // TODO: enforces joint limits

    // copy control commands to the simulation body
    for (int i = 0; i < info_.joints.size(); ++i) {
        Link* joint = ioBody->joint(info_.joints[i].name);

        switch (controlTypes[i]) {
            case ControlMode::POSITION:
                joint->q_target() = commands[i];
                break;
            case ControlMode::VELOCITY:
                joint->dq_target() = commands[i];
                break;
            case ControlMode::EFFORT:
                joint->u() = commands[i];
                break;
            case ControlMode::POSITION_PD:
                joint->u() = - gains[i].p * (joint->q() - commands[i]) - gains[i].d * joint->dq();
                break;
            case ControlMode::VELOCITY_PD:
                joint->u() = - gains[i].p * (joint->dq() - commands[i]) - gains[i].d * joint->ddq();
                break;
            default:
                // TODO: output error message
                return return_type::ERROR;
        }
    }

    return return_type::OK;
}
