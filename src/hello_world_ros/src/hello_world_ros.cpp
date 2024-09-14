#include "std_msgs/msg/string.hpp"
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include "rclcpp/rclcpp.hpp"

namespace hello_world {
   class HelloWorldROS:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate {
    
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    public: void Configure(const gz::sim::Entity &_id,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr) final;
    private: rclcpp::Node::SharedPtr _ros_node; 
    private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  };
}


IGNITION_ADD_PLUGIN(
    hello_world::HelloWorldROS,
    gz::sim::System,
    hello_world::HelloWorldROS::ISystemConfigure,
    hello_world::HelloWorldROS::ISystemPostUpdate)

using namespace hello_world;

void HelloWorldROS::Configure(const gz::sim::Entity &_entity,
    
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) {

    rclcpp::init(0, nullptr);
    _ros_node = rclcpp::Node::make_shared("hello_world_ros_plugin");
    _publisher = _ros_node->create_publisher<std_msgs::msg::String>("topic", 10);


}
void HelloWorldROS::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &/*_ecm*/) {
    std::string msg = "Hello, world! Simulation is ";
    if (!_info.paused)
        msg += "not ";
    msg += "paused.";

 
    auto message = std_msgs::msg::String();
    message.data = msg;
    _publisher->publish( message );
}

