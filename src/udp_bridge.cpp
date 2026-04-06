#include "capra_udp_bridge/udp_bridge.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <cstring>

#include "capra_udp_bridge/udp_bridge.hpp"

#include <cstring>
#include <fcntl.h>
#include <sys/select.h>

static std::string dir_to_str(Direction d)
{
  switch (d) {
    case Direction::IN: return "IN";
    case Direction::OUT: return "OUT";
    case Direction::BIDIR: return "BIDIR";
  }
  return "UNKNOWN";
}

static std::string addr_to_str(const sockaddr_in & addr)
{
  char ip[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
  return std::string(ip) + ":" + std::to_string(ntohs(addr.sin_port));
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
  int yes = 1;

  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (tx_sock_ < 0)
    throw std::runtime_error("TX socket failed");

  for (const auto & cfg : topics_)
  {
    if (cfg.direction == Direction::OUT)
      continue;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
      throw std::runtime_error("RX socket failed");

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg.port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR(get_logger(),
        "bind failed on port %d: %s",
        cfg.port, strerror(errno));
      throw std::runtime_error("bind failed");
    }

    rx_socks_[cfg.port] = sock;

    RCLCPP_INFO(get_logger(),
      "[UDP] RX bound port %d", cfg.port);
  }
}

void UdpBridgeNode::setup_topics()
{
  for (const auto & cfg : topics_)
  {
    RCLCPP_INFO(get_logger(),
      "[SETUP] %s | %s | port=%d | %s",
      cfg.type.c_str(),
      cfg.topic.c_str(),
      cfg.port,
      dir_to_str(cfg.direction).c_str());

    // UDP -> ROS
    if (cfg.direction != Direction::OUT)
    {
      pubs_.push_back(create_generic_publisher(
        cfg.topic, cfg.type, rclcpp::QoS(10)));

      RCLCPP_INFO(get_logger(),
        "  ↳ PUB UDP:%d → ROS:%s",
        cfg.port, cfg.topic.c_str());
    }
    else {
      pubs_.push_back(nullptr);
    }

    // ROS -> UDP
    if (cfg.direction != Direction::IN)
    {
      subs_.push_back(create_generic_subscription(
        cfg.topic,
        cfg.type,
        rclcpp::QoS(10),
        [this, cfg](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          sockaddr_in dest{};
          dest.sin_family = AF_INET;
          dest.sin_port = htons(cfg.port);
          inet_pton(AF_INET, proxy_ip_.c_str(), &dest.sin_addr);

          auto size = msg->size();

          ssize_t sent = sendto(
            tx_sock_,
            msg->get_rcl_serialized_message().buffer,
            size,
            0,
            (sockaddr*)&dest,
            sizeof(dest));

          if (sent < 0)
          {
            RCLCPP_ERROR(get_logger(),
              "[UDP TX FAIL] %s → %s (%s)",
              cfg.topic.c_str(),
              proxy_ip_.c_str(),
              strerror(errno));
          }
          else
          {
            RCLCPP_INFO(get_logger(),
              "[UDP TX] %s → %s:%d (%ld bytes)",
              cfg.topic.c_str(),
              proxy_ip_.c_str(),
              cfg.port,
              sent);
          }
        }));

      RCLCPP_INFO(get_logger(),
        "  ↳ SUB ROS:%s → UDP:%s:%d",
        cfg.topic.c_str(),
        proxy_ip_.c_str(),
        cfg.port);
    }
    else {
      subs_.push_back(nullptr);
    }
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
      fd_set set;
      FD_ZERO(&set);

      int maxfd = 0;

      for (const auto & [port, sock] : rx_socks_)
      {
        FD_SET(sock, &set);
        maxfd = std::max(maxfd, sock);
      }

      int ret = select(maxfd + 1, &set, nullptr, nullptr, nullptr);

      if (ret <= 0)
        continue;

      for (const auto & [port, sock] : rx_socks_)
      {
        if (!FD_ISSET(sock, &set))
          continue;

        sockaddr_in src{};
        socklen_t len = sizeof(src);

        ssize_t size = recvfrom(
          sock,
          buffer,
          sizeof(buffer),
          0,
          (sockaddr*)&src,
          &len);

        if (size <= 0)
          continue;

        RCLCPP_INFO(get_logger(),
          "[UDP RX] port=%d from %s (%ld bytes)",
          port,
          addr_to_str(src).c_str(),
          size);

        auto it = std::find_if(
          topics_.begin(),
          topics_.end(),
          [&](const TopicConfig & c){ return c.port == port; });

        if (it == topics_.end())
          continue;

        size_t idx = std::distance(topics_.begin(), it);

        if (!pubs_[idx])
          continue;

        rclcpp::SerializedMessage msg(size);
        memcpy(msg.get_rcl_serialized_message().buffer, buffer, size);
        msg.get_rcl_serialized_message().buffer_length = size;

        pubs_[idx]->publish(msg);

        RCLCPP_INFO(get_logger(),
          "[PUB] UDP:%d → ROS:%s",
          port,
          it->topic.c_str());
      }
    }
  }).detach();
}