#ifndef CNOID_ROS_PLUGIN_ROS2_CONTROL_ITEM_H
#define CNOID_ROS_PLUGIN_ROS2_CONTROL_ITEM_H

#include <cnoid/ControllerItem>
#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cnoid {

class ROS2ControlItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    ROS2ControlItem();
    ROS2ControlItem(const ROS2ControlItem& org);
    virtual ~ROS2ControlItem();

    BodyItem* bodyItem() { return bodyItem_; }
    std::shared_ptr<rclcpp::Node> node() { return node_; }
    ControllerIO* io() { return io_; }

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTargetBodyItemChanged(BodyItem* bodyItem) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    BodyItem* bodyItem_;
    std::shared_ptr<rclcpp::Node> node_;
    std::string nodeNamespace;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    std::thread executorThread;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;
    bool isConfiguread;

    ControllerIO* io_;
    rclcpp::Time now;
    std::shared_ptr<rclcpp::Duration> period;
    std::shared_ptr<rclcpp::Duration> controlPeriod;

    std::unique_ptr<hardware_interface::ResourceManager> createResourceManager();
    void finalize();
    std::string getURDF() const;
};

typedef ref_ptr<ROS2ControlItem> ROS2ControlItemPtr;

}  // namespace cnoid

#endif  // CNOID_ROS_PLUGIN_ROS2_CONTROL_ITEM_H
