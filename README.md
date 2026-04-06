# Capre UDP Bridge
This noe creates a bridge between ROS and UDP.

## Configuration
For each node, you need to specify the topic, the direction and the message type.

For example, to send data from ROS to UDP, you can have:
```yaml
/udp_bridge_node_tx:
  ros__parameters:
    proxy_ip: "0.0.0.0"

    cmd_vel:
      topic: /rove/cmd_vel
      type: geometry_msgs/msg/Twist
      port: 5000
      direction: out
```
This will send the twist messages from `/rove/cmd_vel` to `0.0.0.0:5000`.


Similarly, to receive messages from UDP to ROS, you can have:
```yaml
/udp_bridge_node_rx:
  ros__parameters:
    proxy_ip: "0.0.0.0"

    cmd_vel:
      topic: /remote/rove/cmd_vel
      type: geometry_msgs/msg/Twist
      port: 5000
      direction: in
```
This will publish twist messages coming from `0.0.0.0:5000` to `/rove/remote/cmd_vel`
