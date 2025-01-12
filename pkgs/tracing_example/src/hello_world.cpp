#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

//#include "tracing_example/hello-tp.h"

using namespace std::chrono_literals;

class TracingExample : public rclcpp::Node
{
public:
  explicit TracingExample(const std::string &node_name)
      : Node(node_name)
  {
    pub_ = create_publisher<std_msgs::msg::String>("pub_topic", rclcpp::QoS(1));
  }

  void publish()
  {
    RCLCPP_INFO(get_logger(), "iteration start");
    std_msgs::msg::String msg;
    msg.data = "iteration start";
    // lttng_ust_tracepoint(hello_world, my_first_tracepoint, 23,
    //                      "iteration start");
    pub_->publish(msg);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TracingExample>("hello_world");
  rclcpp::Rate r(2);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->publish();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
