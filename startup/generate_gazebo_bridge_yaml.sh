#!/bin/bash

# Number of robots
num_robots=$1

# Output YAML file
output_file="../config/gz_bridge.yaml"

# Start writing to the YAML file
cat <<EOF > $output_file
# Auto-generated YAML file for robot topics
EOF

for ((i=1; i<=num_robots; i++)); do
  gz_index=$((i-1))

  cat <<EOF >> $output_file
- ros_topic_name: "cf_${i}/camera_info"
  gz_topic_name: "cf_${gz_index}/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "ignition.msgs.CameraInfo"
  subscriber_queue: 10
  publisher_queue: 10
  lazy: false
  direction: GZ_TO_ROS

- ros_topic_name: "cf_${i}/image"
  gz_topic_name: "cf_${gz_index}/camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "ignition.msgs.Image"
  subscriber_queue: 10
  publisher_queue: 10
  lazy: false
  direction: GZ_TO_ROS

- ros_topic_name: "cf_${i}/battery_status"
  gz_topic_name: "model/crazyflie_${gz_index}/battery/linear_battery/state"
  ros_type_name: "sensor_msgs/msg/BatteryState"
  gz_type_name: "ignition.msgs.BatteryState"
  subscriber_queue: 10
  publisher_queue: 10
  lazy: false
  direction: GZ_TO_ROS

- ros_topic_name: "cf_${i}/battery_charge/start"
  gz_topic_name: "model/crazyflie_${gz_index}/battery/linear_battery/recharge/start"
  ros_type_name: "std_msgs/msg/Bool"
  gz_type_name: "ignition.msgs.Boolean"
  subscriber_queue: 10
  publisher_queue: 10
  lazy: false
  direction: ROS_TO_GZ

- ros_topic_name: "cf_${i}/battery_charge/stop"
  gz_topic_name: "model/crazyflie_${gz_index}/battery/linear_battery/recharge/stop"
  ros_type_name: "std_msgs/msg/Bool"
  gz_type_name: "ignition.msgs.Boolean"
  subscriber_queue: 10
  publisher_queue: 10
  lazy: false
  direction: ROS_TO_GZ

EOF
done

echo "YAML file created: $output_file"
