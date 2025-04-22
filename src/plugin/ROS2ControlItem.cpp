#include "ROS2ControlItem.h"
#include "SystemInterfaceCnoid.h"
#include "Format.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageOut>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void ROS2ControlItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<ROS2ControlItem>(N_("ROS2ControlItem"))
        .addCreationPanel<ROS2ControlItem>();
}


ROS2ControlItem::ROS2ControlItem()
{
    io = nullptr;
}


ROS2ControlItem::ROS2ControlItem(const ROS2ControlItem& org)
    : ControllerItem(org)
{
    io = nullptr;
}


ROS2ControlItem::~ROS2ControlItem()
{
    finalize();
}


Item* ROS2ControlItem::doDuplicate() const
{
    return new ROS2ControlItem(*this);
}


bool ROS2ControlItem::store(Archive& archive)
{
    if(!nodeNamespace.empty()){
        archive.write("namespace", nodeNamespace);
    }
    return true;
}

bool ROS2ControlItem::restore(const Archive& archive)
{
    if(!archive.read({"namespace"}, nodeNamespace)){
        nodeNamespace.clear();
    }
    return true;
}


void ROS2ControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Namespace"), nodeNamespace, changeProperty(nodeNamespace));
}


bool ROS2ControlItem::initialize(ControllerIO* io)
{
    auto mv = MessageView::instance();
    
    // check if Choreonoid is executed as a ROS 2 node
    if (!rclcpp::ok()) {
        mv->putln(
            formatR(_("Choreonoid is not executed as a ROS 2 node")),
            MessageView::Error);
    }

    // check the body
    if (!io->body()){
        mv->putln(
            formatR(_("ROS2ControlItem \"{}\" is invalid because it is not assigned to a body"),
                    displayName()),
            MessageView::Error);
        return false;
    }

    // copy controller interface into the private member
    this->io = io;

    // create an executor and a executor thread
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executorThread = std::thread([this](){ executor->spin(); });

    // create ros2_control node
    const std::string nodeName = "choreonoid_ros2_control";
    node = rclcpp::Node::make_shared(nodeName, nodeNamespace);
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    executor->add_node(node);
    
    // get urdf from the robot state publisher
    const std::string urdfString = getURDF();

    // construct hardwareInfo from the URDF data
    std::vector<hardware_interface::HardwareInfo> hardwareInfos;
    try {
        hardwareInfos = hardware_interface::parse_control_resources_from_urdf(urdfString);
    } catch (const std::runtime_error& error) {
        mv->putln(
            formatR(_("Failed to parse the robot URDF: {}"), error.what()),
            MessageView::Error);
        finalize();
        return false;
    }
    HardwareInfo& hardwareInfo = hardwareInfos.front();
    hardwareInfo.type = "system";

#if !defined(ROS_DISTRO_HUMBLE)
    hardwareInfo.hardware_plugin_name = "choreonoid_ros2_control/SystemInterfaceCnoid";
#endif

    // initialize ResourceManager
    std::unique_ptr<hardware_interface::ResourceManager> resourceManager;

    try {
#if defined(ROS_DISTRO_HUMBLE)
        resourceManager = std::make_unique<hardware_interface::ResourceManager>();
        resourceManager->load_urdf(urdfString, false, false);
#else
        resourceManager = std::make_unique<hardware_interface::ResourceManager>(
            urdfString, node->get_node_clock_interface(), node->get_node_logging_interface());
#endif
    } catch (...) {
        mv->putln(formatR(_("Failed to initialize ResourceManager")), MessageView::Error);
        finalize();
        return false;
    }

    // initialize SystemInterfaceCnoid
    auto interface = make_unique<SystemInterfaceCnoid>(io, node);
    resourceManager->import_component(std::move(interface), hardwareInfo);
    
    // activate the corresponding HardwareComponent
    rclcpp_lifecycle::State state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE);
    resourceManager->set_component_state(hardwareInfo.name, state);

    // create controller manager
    mv->putln(formatR(_("Creating ControllerManager")));

    auto options = controller_manager::get_cm_node_options();
    options.parameter_overrides().emplace_back("use_sim_time", true);
    
    controllerManager = std::make_shared<controller_manager::ControllerManager>(
        std::move(resourceManager),
        executor,
        "controller_manager",
        nodeNamespace,
        options);

    const int updateRate = controllerManager->get_parameter("update_rate").as_int();
    if (updateRate < 0.1) {
        mv->putln(
            formatR(_("ROS2ControlItem {} gets an invalid update rate: {}. It should be >= 0.1"),
                    displayName(), updateRate),
            MessageView::Error);
    }
    controlPeriod.reset(new rclcpp::Duration(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / static_cast<double>(updateRate)))));

    executor->add_node(controllerManager);
    
    mv->putln(formatR(_("{} has successfully been initialized"), displayName()));

    return true;
}


bool ROS2ControlItem::start()
{
    // set time step
    const double timeStep = io->timeStep();
    const int timeStepSec = static_cast<int>(timeStep);
    const int timeStepNsec = static_cast<int>(timeStep * 1e9);

    period.reset(new rclcpp::Duration(timeStepSec, timeStepNsec));

    return true;
}


void ROS2ControlItem::input()
{
    // get current time
    const double currentTime = io->currentTime();
    const int nowSec = static_cast<int>(currentTime);
    const int nowNsec = static_cast<int>((currentTime - nowSec) * 1e9);

    // RCL_ROS_TIME parameter is essential to use simulation time
    now = rclcpp::Time(nowSec, nowNsec, RCL_ROS_TIME);

    // read joint states
    controllerManager->read(now, *period);
}


bool ROS2ControlItem::control()
{
    // update control commands
    // TODO: apply update_rate param of controllerManager
    try {
        controllerManager->update(now, *period);
    }
    catch(const std::runtime_error& error){
        io->mout()->putErrorln(error.what());
        return false;
    }
    return true;
}


void ROS2ControlItem::output()
{
    // write commands into joints
    controllerManager->write(now, *period);
}


void ROS2ControlItem::stop()
{

}


void ROS2ControlItem::finalize()
{
    if (executor) {
        executor->cancel();
        executorThread.join();
        if(node){
            executor->remove_node(node);
        }
        if(controllerManager){
            executor->remove_node(controllerManager);
        }
        executor.reset();
    }
}


std::string ROS2ControlItem::getURDF() const
{
    string robotStatePublisherName;
    if (nodeNamespace.empty()) {
        robotStatePublisherName = "robot_state_publisher";
    } else {
        robotStatePublisherName = formatC("{0}/robot_state_publisher", nodeNamespace);
    }
    string parameterName = "robot_description";
    
    std::string urdfString;
    auto mv = MessageView::instance();

    using namespace std::chrono_literals;
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
        node, robotStatePublisherName);

    mv->putln(formatR(_("Try to connect to {} ..."), robotStatePublisherName));

    while (!client->wait_for_service(0.1s)) {}

    mv->putln(formatR(_("Connected to {}"), robotStatePublisherName));

    // search and wait for robot_description parameter
    mv->putln(
        formatR(_("Try to get URDF parameter {} from {} ..."),
                parameterName, robotStatePublisherName));

    try {
        auto f = client->get_parameters({parameterName});
        f.wait();
        std::vector<rclcpp::Parameter> values = f.get();
        urdfString = values[0].as_string();
    } catch (const std::exception& error) {
        mv->putln(
            formatR(_("Failed to get the URDF parameter: {}"), error.what()),
            MessageView::Error);
    }

    if (urdfString.empty()) {
        mv->putln(
            formatR(_("Parameter {} of {} is empty"),
                    parameterName, robotStatePublisherName),
            MessageView::Error);
    }

    mv->putln(formatR(_("Received URDF parameter from {}"), robotStatePublisherName));

    return urdfString;
}
