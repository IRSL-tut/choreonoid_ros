#include "ROS2ControlItem.h"
#include "BodySystemInterface.h"
#include "Format.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageOut>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

#ifndef ROS_DISTRO_HUMBLE
class ResourceManagerEx : public hardware_interface::ResourceManager
{
public:
    ResourceManagerEx(ROS2ControlItem* item);
    virtual bool load_and_initialize_components(const std::string& urdf, unsigned int update_rate) override;
private:
    ROS2ControlItem* item;
};
#endif

}


void ROS2ControlItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<ROS2ControlItem>(N_("ROS2ControlItem"))
        .addCreationPanel<ROS2ControlItem>();
}


ROS2ControlItem::ROS2ControlItem()
    : currentRosTime(0, 0, RCL_ROS_TIME),
      cmPeriod(0, 0)
{
    bodyItem_ = nullptr;
    isConfiguread = false;
#ifndef ROS_DISTRO_HUMBLE
    cmClockHandle = nullptr;
    isClockOverrideEnabled = false;
#endif
    io_ = nullptr;
}


ROS2ControlItem::ROS2ControlItem(const ROS2ControlItem& org)
    : ControllerItem(org),
      currentRosTime(0, 0, RCL_ROS_TIME),
      cmPeriod(0, 0)
{
    bodyItem_ = nullptr;
    isConfiguread = false;
#ifndef ROS_DISTRO_HUMBLE
    cmClockHandle = nullptr;
    isClockOverrideEnabled = false;
#endif
    io_ = nullptr;
}


ROS2ControlItem::~ROS2ControlItem()
{
    finalize();
}


Item* ROS2ControlItem::doDuplicate() const
{
    return new ROS2ControlItem(*this);
}


void ROS2ControlItem::onTargetBodyItemChanged(BodyItem* bodyItem)
{
    bodyItem_ = bodyItem;
    isConfiguread = false;
    
    auto mout = MessageOut::master();
    
    if(!bodyItem){
        mout->putErrorln(
            formatR(_("{0} must be a child item of a robot item to be configured."),
                    displayName()));
        finalize();
        return;
    }
        
    if (!rclcpp::ok()) {
        mout->putErrorln(
            formatR(_("{0} cannot be configured because Choreonoid is not executed with choreonoid_ros."),
                    displayName()));
        return;
    }
    
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executorThread = std::thread([this](){ executor->spin(); });
    
    const std::string nodeName = "choreonoid_ros2_control";
    node_ = rclcpp::Node::make_shared(nodeName, nodeNamespace);
    node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    executor->add_node(node_);
    
    auto resourceManager = createResourceManager();
    if(!resourceManager){
        mout->putErrorln(formatR(_("{0} cannot be configured."), displayName()));
        finalize();
        return;
    }
            
    auto options = controller_manager::get_cm_node_options();
    options.parameter_overrides().emplace_back("use_sim_time", true);
#ifndef ROS_DISTRO_HUMBLE
    // Detach the ControllerManager's clock from the /clock topic so that
    // the simulation time can be injected directly via rcl_set_ros_time_override.
    // Without this, the /clock topic would asynchronously overwrite the injected time.
    options.arguments({
        "--ros-args",
        "-r", "/clock:=/__choreonoid_unused_clock"
    });
#endif
    controllerManager = std::make_shared<controller_manager::ControllerManager>(
        std::move(resourceManager),
        executor,
        "controller_manager",
        nodeNamespace,
        options);

    const int updateRate = controllerManager->get_parameter("update_rate").as_int();
    if (updateRate < 0.1) {
        mout->putErrorln(
            formatR(_("Invalid update rate {0} is specified for {1}. The rate should be >= 0.1"),
                    updateRate, displayName()));
    }

    executor->add_node(controllerManager);

#ifndef ROS_DISTRO_HUMBLE
    cmClockHandle = controllerManager->get_clock()->get_clock_handle();
    isClockOverrideEnabled = false;
#endif

    isConfiguread = true;
}


std::unique_ptr<hardware_interface::ResourceManager> ROS2ControlItem::createResourceManager()
{
#ifndef ROS_DISTRO_HUMBLE
    return std::make_unique<ResourceManagerEx>(this);
#else
    // get urdf from the robot state publisher
    const std::string urdfString = getURDF();

    // construct hardwareInfo from the URDF data
    std::vector<hardware_interface::HardwareInfo> hardwareInfos;
    try {
        hardwareInfos = hardware_interface::parse_control_resources_from_urdf(urdfString);
    } catch (const std::runtime_error& error) {
        MessageOut::master()->putErrorln(formatR(_("Failed to parse the robot URDF: {}"), error.what()));
        finalize();
        return nullptr;
    }
    if(hardwareInfos.empty()){
        MessageOut::master()->putErrorln(_("ros2_control information is not found in the robot URDF."));
        finalize();
        return nullptr;
    }
    auto& hardwareInfo = hardwareInfos.front();
    
    auto resourceManager = std::make_unique<hardware_interface::ResourceManager>();
    resourceManager->load_urdf(urdfString, false, false);
    
    auto bodySystem = std::make_unique<BodySystemInterface>(this);
    resourceManager->import_component(std::move(bodySystem), hardwareInfo);

    // activate the corresponding HardwareComponent
    rclcpp_lifecycle::State state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE);
    resourceManager->set_component_state(hardwareInfo.name, state);

    return resourceManager;
#endif
}


std::string ROS2ControlItem::getURDF() const
{
    auto mout = MessageOut::master();
    
    string robotStatePublisherName;
    if (nodeNamespace.empty()) {
        robotStatePublisherName = "robot_state_publisher";
    } else {
        robotStatePublisherName = formatC("{0}/robot_state_publisher", nodeNamespace);
    }
    string parameterName = "robot_description";
    
    using namespace std::chrono_literals;
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
        node_, robotStatePublisherName);

    mout->putln(formatR(_("Try to connect to {} ..."), robotStatePublisherName));

    while (!client->wait_for_service(0.1s)) {}

    mout->putln(formatR(_("Connected to {}"), robotStatePublisherName));

    // search and wait for robot_description parameter
    mout->putln(
        formatR(_("Try to get URDF parameter {} from {} ..."),
                parameterName, robotStatePublisherName));

    std::string urdfString;
    try {
        auto f = client->get_parameters({parameterName});
        f.wait();
        std::vector<rclcpp::Parameter> values = f.get();
        urdfString = values[0].as_string();
    } catch (const std::exception& error) {
        mout->putErrorln(
            formatR(_("Failed to get the URDF parameter: {}"), error.what()));
    }

    if (urdfString.empty()) {
        mout->putErrorln(
            formatR(_("Parameter {} of {} is empty"),
                    parameterName, robotStatePublisherName));
    }

    mout->putln(formatR(_("Received URDF parameter from {}"), robotStatePublisherName));

    return urdfString;
}


bool ROS2ControlItem::initialize(ControllerIO* io)
{
    if(!isConfiguread){
        io->mout()->putErrorln(formatR(_("{0} is not correctly configured."), displayName()));
        return false;
    }
    
    io_ = io;
    return true;
}


bool ROS2ControlItem::start()
{
    cmPeriod = rclcpp::Duration::from_seconds(io_->timeStep());
    return true;
}


void ROS2ControlItem::input()
{
    // Update the cached ROS time at the beginning of the simulation step.
    // The same value is reused by control() and output() in the same step.
    double time = io_->currentTime();

#ifndef ROS_DISTRO_HUMBLE
    // Avoid zero time to prevent controller_manager error in Jazzy
    // when clock check fails at the first control cycle
    if (time < 1e-9) {
        time = 1e-9;  // 1 nanosecond
    }
#endif

    int64_t nsec_total = static_cast<int64_t>(std::llround(time * 1e9));
    int32_t sec = static_cast<int32_t>(nsec_total / 1000000000LL);
    uint32_t nsec = static_cast<uint32_t>(nsec_total % 1000000000LL);
    currentRosTime = rclcpp::Time(sec, nsec, RCL_ROS_TIME);

#ifndef ROS_DISTRO_HUMBLE
    // In Jazzy and later, ControllerManager::update ignores its time argument
    // and instead uses its own node clock. Inject the simulation time into
    // that clock via the rcl ROS time override mechanism so that controllers
    // receive the simulation time in lockstep with read/update/write calls.
    if(cmClockHandle){
        auto cmClock = controllerManager->get_clock();
        std::lock_guard<std::mutex> clockLock(cmClock->get_clock_mutex());
        if(!isClockOverrideEnabled){
            if(rcl_enable_ros_time_override(cmClockHandle) == RCL_RET_OK){
                isClockOverrideEnabled = true;
            }
        }
        rcl_ret_t ret = rcl_set_ros_time_override(cmClockHandle, nsec_total);
        (void)ret;
    }
#endif

    controllerManager->read(currentRosTime, cmPeriod);
}


bool ROS2ControlItem::control()
{
    // TODO: apply update_rate param of controllerManager
    try {
        controllerManager->update(currentRosTime, cmPeriod);
    }
    catch(const std::runtime_error& error){
        io_->mout()->putErrorln(error.what());
        return false;
    }
    return true;
}


void ROS2ControlItem::output()
{
    controllerManager->write(currentRosTime, cmPeriod);
}


void ROS2ControlItem::stop()
{
    io_ = nullptr;
}


void ROS2ControlItem::finalize()
{
    if (executor) {
        executor->cancel();
        executorThread.join();
        if(controllerManager){
            executor->remove_node(controllerManager);
            controllerManager.reset();
        }
        if(node_){
            executor->remove_node(node_);
            node_.reset();
        }
        executor.reset();
    }
#ifndef ROS_DISTRO_HUMBLE
    cmClockHandle = nullptr;
    isClockOverrideEnabled = false;
#endif
    isConfiguread = false;
    io_ = nullptr;
}


void ROS2ControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Namespace"), nodeNamespace, changeProperty(nodeNamespace));
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


#ifndef ROS_DISTRO_HUMBLE
ResourceManagerEx::ResourceManagerEx(ROS2ControlItem* item)
    : hardware_interface::ResourceManager(
        item->node()->get_node_clock_interface(), item->node()->get_node_logging_interface()),
      item(item)
{

}


bool ResourceManagerEx::load_and_initialize_components(const std::string& urdf, unsigned int /* update_rate */)
{
    components_are_loaded_and_initialized_ = false;
    auto hardwareInfos = hardware_interface::parse_control_resources_from_urdf(urdf);
    for(auto& info : hardwareInfos){
        if(info.hardware_plugin_name == "choreonoid_ros2_control/BodySystemInterface" &&
           info.type == "system"){
            auto bodySystem = std::make_unique<BodySystemInterface>(item);
            hardware_interface::HardwareComponentParams params;
            params.hardware_info = info;
            params.logger = item->node()->get_logger();
            params.clock = item->node()->get_clock();
            import_component(std::move(bodySystem), params);
            components_are_loaded_and_initialized_ = true;
            break;
        }
    }
    return components_are_loaded_and_initialized_;
}
#endif
