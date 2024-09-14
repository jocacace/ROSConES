#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

namespace sine_controller {
  class SineController : public controller_interface::ControllerInterface
  {
  private:  
    rclcpp::Duration _dt;
    template<typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
    std::vector<std::vector<std::string>> state_interface_names_;
    
    int _j_size = 1;
    float _initial_joint_position;
    double _desired_joint_positions;
    float _amplitude;
    float _frequency;
    double t_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _sine_param_sub;

  public:
    SineController() : controller_interface::ControllerInterface(), _dt(0, 0) {}

    void sine_param_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)  {
    
      if( msg->data.size() != 2 ) {
        cout << "Wrong number of sine parameters" << endl;
        return;
      }
      
      _amplitude = msg->data[0];
      _frequency = msg->data[1];
      
    }

    
    // setta dove andare a prendere lo stato (virtuale) - hw interface
    controller_interface::InterfaceConfiguration state_interface_configuration() const {
      std::vector<std::string> state_interfaces_config_names;
      state_interfaces_config_names.push_back("revolute_joint/position");
      //state_interfaces_config_names.push_back("revolute_joint/velocity");
   
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
    }

    
    controller_interface::InterfaceConfiguration command_interface_configuration() const {
      std::vector<std::string> command_interfaces_config_names;
      command_interfaces_config_names.push_back("revolute_joint/position");
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces_config_names};
    }

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      
      _sine_param_sub =   get_node()->create_subscription<std_msgs::msg::Float32MultiArray>("/sine_param", 10, 
            std::bind(&SineController::sine_param_cb, this, std::placeholders::_1));

      _dt = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / update_rate_));

      _amplitude = 0.0;
      _frequency = 0.0;

      command_interfaces_.reserve     (_j_size); // metterlo in un float2
      state_interfaces_.reserve       (_j_size);
      joint_state_interfaces_.resize  (_j_size);
      state_interface_names_.resize   (_j_size);
      
    
      std::vector<std::string> joint_name = {"revolute_joint"};
      for(int i = 0; i < _j_size; i++) {
        state_interface_names_[i].resize(1);
        state_interface_names_[i][0] = joint_name[i] + "/position";
      }

  
      RCLCPP_INFO(get_node()->get_logger(), "configure successful");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    // quando il controllore passa in attivo
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
      for(int i = 0; i < 1; i++) 
        controller_interface::get_ordered_interfaces( state_interfaces_, state_interface_names_[i], std::string(""),joint_state_interfaces_[i]);
      
      _initial_joint_position = joint_state_interfaces_[0][0].get().get_value();
      RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
      t_ = 0.0;

      return controller_interface::CallbackReturn::SUCCESS;
    }

    // on init viene chiamato quando c' il load del controllore
    controller_interface::CallbackReturn on_init() {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
      return controller_interface::CallbackReturn::SUCCESS;
    }
    
    controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        t_ += _dt.seconds();
        _desired_joint_positions = _initial_joint_position + (_amplitude * std::sin(2 * M_PI * _frequency * t_));
        command_interfaces_[0].set_value(_desired_joint_positions);
        return controller_interface::return_type::OK;
    }
  };
} // namespace sine_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sine_controller::SineController, controller_interface::ControllerInterface)
