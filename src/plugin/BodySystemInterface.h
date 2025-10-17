#ifndef CNOID_ROS_PLUGIN_BODY_SYSTEM_INTERFACE_H
#define CNOID_ROS_PLUGIN_BODY_SYSTEM_INTERFACE_H

#include <cnoid/ControllerIO>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace cnoid {

class ROS2ControlItem;

class BodySystemInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BodySystemInterface);

    BodySystemInterface();
    BodySystemInterface(ROS2ControlItem* item);

#ifndef ROS_DISTRO_HUMBLE
    virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
#else
    virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
#endif
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

    ROS2ControlItem* item;
    std::shared_ptr<rclcpp::Node> node;
    cnoid::ControllerIO* io;
    std::vector<State> states;
    std::vector<double> commands;
    std::vector<ControlMode> controlTypes;
    std::vector<Gain> gains;
};

}  // namespace cnoid

#endif
