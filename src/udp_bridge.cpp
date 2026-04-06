#include "capra_udp_bridge/udp_bridge.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <cstring>

static std::string dir_to_str(Direction d)
{
  switch (d)
  {
    case Direction::IN: return "IN";
    case Direction::OUT: return "OUT";
    case Direction::BIDIR: return "BIDIR";
  }
  return "UNKNOWN";
}

UdpBridgeNode::UdpBridgeNode()
: Node("capra_udp_bridge", "",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
{
  parse_params();
  setup_udp();
  setup_topics();
  start_rx_thread();
}

void UdpBridgeNode::parse_params()
{
  // Required param
  this->declare_parameter("proxy_ip", proxy_ip_);

  auto nh = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  fetch_param(nh, "proxy_ip", proxy_ip_);
  RCLCPP_INFO(get_logger(), "[CONFIG] proxy_ip=%s", proxy_ip_.c_str());

  // Discover groups (like twist_mux)
  auto result = this->list_parameters({}, 2);

  std::set<std::string> groups;

  for (const auto & name : result.names)
  {
    if (name.rfind("qos_overrides", 0) == 0)
      continue;

    auto pos = name.find('.');
    if (pos == std::string::npos)
      continue;

    groups.insert(name.substr(0, pos));
  }

  // Parse each group
  for (const auto & g : groups)
  {
    try
    {
      TopicConfig cfg;

      fetch_param(nh, g + ".topic", cfg.topic);
      fetch_param(nh, g + ".type", cfg.type);
      fetch_param(nh, g + ".port", cfg.port);

      std::string dir = "bidirectional";
      this->get_parameter(g + ".direction", dir);

      if (dir == "in") cfg.direction = Direction::IN;
      else if (dir == "out") cfg.direction = Direction::OUT;
      else cfg.direction = Direction::BIDIR;

      topics_.push_back(cfg);

      RCLCPP_INFO(get_logger(),
        "[CONFIG] group='%s' topic='%s' type='%s' port=%d dir=%s",
        g.c_str(), cfg.topic.c_str(), cfg.type.c_str(),
        cfg.port, dir_to_str(cfg.direction).c_str());
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(get_logger(),
        "Skipping group '%s': %s", g.c_str(), e.what());
    }
  }

  if (topics_.empty())
    RCLCPP_WARN(get_logger(), "No valid topics configured!");
}

void UdpBridgeNode::setup_udp()
{
  send_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  recv_sock_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (send_sock_ < 0 || recv_sock_ < 0)
    throw std::runtime_error("socket failed");

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(0);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(recv_sock_, (sockaddr*)&addr, sizeof(addr)) < 0)
    throw std::runtime_error("bind failed");

  RCLCPP_INFO(get_logger(), "[UDP] sockets ready");
}

void UdpBridgeNode::setup_topics()
{
  for (auto & cfg : topics_)
  {
    RCLCPP_INFO(get_logger(),
      "[SETUP] %s | %s | port=%d | dir=%s",
      cfg.type.c_str(), cfg.topic.c_str(),
      cfg.port, dir_to_str(cfg.direction).c_str());

    // PUB (UDP -> ROS)
    if (cfg.direction != Direction::OUT)
    {
      pubs_.push_back(create_generic_publisher(
        cfg.topic, cfg.type, rclcpp::QoS(10)));

      RCLCPP_INFO(get_logger(),
        "  ↳ PUB UDP:%d → ROS:%s",
        cfg.port, cfg.topic.c_str());
    }
    else pubs_.push_back(nullptr);

    // SUB (ROS -> UDP)
    if (cfg.direction != Direction::IN)
    {
      subs_.push_back(create_generic_subscription(
        cfg.topic, cfg.type, rclcpp::QoS(10),
        [this, cfg](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          sockaddr_in dest{};
          dest.sin_family = AF_INET;
          dest.sin_port = htons(cfg.port);
          inet_pton(AF_INET, proxy_ip_.c_str(), &dest.sin_addr);

          sendto(send_sock_,
            msg->get_rcl_serialized_message().buffer,
            msg->size(), 0,
            (sockaddr*)&dest, sizeof(dest));
        }));

      RCLCPP_INFO(get_logger(),
        "  ↳ SUB ROS:%s → UDP:%s:%d",
        cfg.topic.c_str(), proxy_ip_.c_str(), cfg.port);
    }
    else subs_.push_back(nullptr);
  }
}

void UdpBridgeNode::start_rx_thread()
{
  RCLCPP_INFO(get_logger(), "[UDP RX] thread started");

  std::thread([this]()
  {
    uint8_t buffer[65536];

    while (rclcpp::ok())
    {
      sockaddr_in src{};
      socklen_t len = sizeof(src);

      ssize_t size = recvfrom(
        recv_sock_, buffer, sizeof(buffer), 0,
        (sockaddr*)&src, &len);

      if (size <= 0) continue;

      int port = ntohs(src.sin_port);

      for (size_t i = 0; i < topics_.size(); ++i)
      {
        if (topics_[i].port != port) continue;
        if (!pubs_[i]) continue;

        rclcpp::SerializedMessage msg(size);
        memcpy(msg.get_rcl_serialized_message().buffer, buffer, size);
        msg.get_rcl_serialized_message().buffer_length = size;

        pubs_[i]->publish(msg);
      }
    }
  }).detach();
}