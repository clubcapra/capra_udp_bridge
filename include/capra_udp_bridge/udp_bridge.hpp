#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialized_message.hpp>

#include <arpa/inet.h>
#include <unistd.h>

#include <thread>
#include <vector>
#include <string>
#include <unordered_map>
#include <set>
#include <sstream>
#include <stdexcept>

class ParamsHelperException : public std::runtime_error
{
public:
  explicit ParamsHelperException(const std::string & what)
  : std::runtime_error(what) {}
};

template<class T>
void fetch_param(std::shared_ptr<rclcpp::Node> nh,
                 const std::string & param_name,
                 T & output)
{
  rclcpp::Parameter param;
  if (!nh->get_parameter(param_name, param)) {
    std::ostringstream err;
    err << "Missing param '" << param_name
        << "' (ns: " << nh->get_namespace() << ")";
    throw ParamsHelperException(err.str());
  }
  output = param.get_value<T>();
}

enum class Direction { IN, OUT, BIDIR };

struct TopicConfig
{
  std::string topic;
  std::string type;
  int port = 0;
  Direction direction = Direction::BIDIR;
};

class UdpBridgeNode : public rclcpp::Node
{
public:
  UdpBridgeNode();

private:
  void parse_params();
  void setup_udp();
  void setup_topics();
  void start_rx_thread();

  std::string proxy_ip_ = "0.0.0.0";
  std::vector<TopicConfig> topics_;

  int tx_sock_;

  // RX: one socket per port (IMPORTANT FIX)
  std::unordered_map<int, int> rx_socks_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subs_;
  std::vector<rclcpp::GenericPublisher::SharedPtr> pubs_;
};