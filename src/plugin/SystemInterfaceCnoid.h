#ifndef SYSTEM_INTERFACE_CNOID_H
#define SYSTEM_INTERFACE_CNOID_H

#include <cnoid/ControllerIO>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace cnoid {

class SystemInterfaceCnoid : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterfaceCnoid);

    SystemInterfaceCnoid();
    SystemInterfaceCnoid(cnoid::ControllerIO* io, std::shared_ptr<rclcpp::Node> node);

    virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    virtual hardware_interface::CallbackReturn on_configure( const rclcpp_lifecycle::State& previous_state) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    virtual hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    template <typename T>
    struct Limits
    {
        T lower;
        T upper;
    };
    struct State
    {
        double position;
        double velocity;
        double effort;
    };
    struct Gain
    {
        double p;
        double d;
    };

    enum ControlMode {
        POSITION,
        VELOCITY,
        EFFORT,
        POSITION_PD,
        VELOCITY_PD,
        UNDEFINED
    };

    cnoid::ControllerIO* controllerIo;
    std::shared_ptr<rclcpp::Node> node;
    std::vector<State> states;
    std::vector<double> commands;
    std::vector<ControlMode> controlTypes;
    std::vector<Gain> gains;
};

}  // namespace cnoid

#endif  // SYSTEM_INTERFACE_CNOID_H
