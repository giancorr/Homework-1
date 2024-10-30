#include <functional>
#include <memory>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PublisherSubscriber : public rclcpp::Node
{
public:
  PublisherSubscriber()
  : Node("arm_controller_node")
  {
    this->declare_parameter("desired_param", rclcpp::PARAMETER_DOUBLE_ARRAY);    
    for(int i=0; i<4; i++){
      desiredpositions[i] = this->get_parameter("desired_param").as_double_array()[i];
    }
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1000, std::bind(&PublisherSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&PublisherSubscriber::timer_callback, this));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PublisherSubscriber::parametersCallback, this, std::placeholders::_1));
  }

private:

  void topic_callback(const sensor_msgs::msg::JointState & msg) const{ 
    int min_size = std::min({msg.name.size(), msg.position.size()});

    for (int i = 0; i < min_size; i++) {
        RCLCPP_INFO(this->get_logger(), "Joint State: '%s' \t Position: %f \n", 
                    msg.name[i].c_str(), msg.position[i]);
    }
  }

  void timer_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.resize(4);
         
        for (int i = 0; i < 4; i++) {
            currentpositions[i] = desiredpositions[i]; 
            message.data[i] = currentpositions[i];
        }
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing positions: [%f, %f, %f, %f]", 
                     message.data[0], message.data[1], message.data[2], message.data[3]);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";
      for (const auto &param: parameters){
        if (param.get_name() == "desired_param"){
          for(int i = 0; i<4; i++){
            desiredpositions[i] = param.as_double_array()[i];
          }
        }
      }
      
      return result;
    }

  std::array<double, 4> currentpositions; 
  std::array<double, 4> desiredpositions; 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherSubscriber>());
  rclcpp::shutdown();
  return 0;
}
