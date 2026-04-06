#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialized_message.hpp>

#include <arpa/inet.h>
#include <unistd.h>

#include <thread>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

// ================= PARAM HELPER (twist_mux style) =================

class ParamsHelperException : public std::runtime_error
{
public:
  explicit ParamsHelperException(const std::string & what)
  : std::runtime_error(what) {}
};

template<class T>
void fetch_param(std::shared_ptr<rclcpp::Node> nh, const std::string & param_name, T & output)
{
  rclcpp::Parameter param;
  if (!nh->get_parameter(param_name, param)) {
    std::ostringstream err_msg;
    err_msg << "could not load parameter '" << param_name << "'. (namespace: " <<
      nh->get_namespace() << ")";
    throw ParamsHelperException(err_msg.str());
  }

  output = param.get_value<T>();
}

// ================= DATA =================

enum class Direction { IN, OUT, BIDIR };

struct TopicConfig
{
  std::string topic;
  std::string type;
  int port = 0;
  Direction direction = Direction::BIDIR;
};

// ================= NODE =================

class UdpBridgeNode : public rclcpp::Node
{
public:
  UdpBridgeNode();
private:
  // ---------- PARAMS ----------
  void parse_params();

  // ---------- UDP ----------
  void setup_udp();

  // ---------- TOPICS ----------
  void setup_topics();

  // ---------- RX ----------
  void start_rx_thread();

  // ---------- MEMBERS ----------

  std::string proxy_ip_ = "0.0.0.0";
  std::vector<TopicConfig> topics_;

  int send_sock_;
  int recv_sock_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subs_;
  std::vector<rclcpp::GenericPublisher::SharedPtr> pubs_;
};
