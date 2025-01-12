#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "tracing_example/primitives-tp.h"

using namespace std::chrono_literals;

class TracingExample : public rclcpp::Node
{
public:
  explicit TracingExample(const std::string &node_name)
      : Node(node_name)
  {
  }

  void showcase()
  {
    RCLCPP_INFO(get_logger(), "primitives example showcase");
    // integer showcase
    lttng_ust_tracepoint(tracing_examples, tracepoint_int,
                          23, 24, 25);
    // float showcase
    lttng_ust_tracepoint(tracing_examples, tracepoint_float,
                          1.5f, 1.7);
    // string showcase
    lttng_ust_tracepoint(tracing_examples, tracepoint_string,
                          (char*)"null_terminated_string");
    // int array showcase
    std::array<int32_t, TRACEPOINT_ARRAY_COUNT> arr1;
    arr1.fill(100);
    lttng_ust_tracepoint(tracing_examples, tracepoint_array,
                          arr1.data());
    // char array showcase
    std::array<char, TRACEPOINT_CHAR_ARRAY_COUNT> arr2;
    strcpy(arr2.data(), "static");
    lttng_ust_tracepoint(tracing_examples, tracepoint_char_array,
                          arr2.data());
    // int sequence showcase
  std::vector<int32_t> vec1(10, 77);
    std::vector<char> cvec1;
    cvec1.resize(strlen("dynamic"));
    strcpy(cvec1.data(), "dynamic");
    lttng_ust_tracepoint(tracing_examples, tracepoint_sequence,
                          vec1.data(), vec1.size(), cvec1.data(), cvec1.size());
    // custom struct showcase
    lttng_ust_tracepoint(tracing_examples, tracepoint_custom_struct, {10.0, 20});
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
    node->showcase();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
