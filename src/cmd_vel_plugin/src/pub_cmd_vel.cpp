#include "std_msgs/msg/string.hpp"
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include "rclcpp/rclcpp.hpp"
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <geometry_msgs/msg/twist.hpp>

namespace cmd_vel_plugin {
   class PubCmdVel:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate {
    
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    public: void Configure(const gz::sim::Entity &_id,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr) final;
    public: void cmd_vel_cb( const geometry_msgs::msg::Twist );

    private: rclcpp::Node::SharedPtr _ros_node; 
    private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    private: rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscriber;
    private: gz::transport::Node::Publisher _gz_cmdVelPub;
    private: gz::transport::Node _gz_node;
    private: gz::msgs::Twist _cmdVelMsg;

  };
}


IGNITION_ADD_PLUGIN(
    cmd_vel_plugin::PubCmdVel,
    gz::sim::System,
    cmd_vel_plugin::PubCmdVel::ISystemConfigure,
    cmd_vel_plugin::PubCmdVel::ISystemPostUpdate)

using namespace cmd_vel_plugin;

void PubCmdVel::cmd_vel_cb( const geometry_msgs::msg::Twist t) {

    double vx = (t.linear.x < 0.2) ? t.linear.x : 0.2; 
    _cmdVelMsg.mutable_linear()->set_x (vx);
    double vz = (t.angular.z < 0.5) ? t.angular.z : 0.5;
    _cmdVelMsg.mutable_angular()->set_z(vz);
 
}


void PubCmdVel::Configure(const gz::sim::Entity &_entity, 
    const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) {

    rclcpp::init(0, nullptr);
    _ros_node = rclcpp::Node::make_shared("cmd_vel_plugin");

    _subscriber = _ros_node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_from_ros", 10, std::bind(&PubCmdVel::cmd_vel_cb, this, std::placeholders::_1));
    _publisher = _ros_node->create_publisher<std_msgs::msg::String>("topic", 10);

    _gz_cmdVelPub = _gz_node.Advertise<gz::msgs::Twist>("/cmd_vel");
}
void PubCmdVel::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &/*_ecm*/) {    
    rclcpp::spin_some( _ros_node );
    _gz_cmdVelPub.Publish(_cmdVelMsg);
}

