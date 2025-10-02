#!/usr/bin/env python3
import rclpy
import yaml
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

class GzServiceNode(Node):
    def __init__(self):
        super().__init__('gz_service_charging_node')
        self.declare_parameter('num_cf', 4)
        self.declare_parameter('charging_area_yaml', '')
        self.declare_parameter('min_height', 0.07)
        
        self.num_cf  = self.get_parameter('num_cf').value
        charging_yaml_file = self.get_parameter('charging_area_yaml').get_parameter_value().string_value
        self.min_height = self.get_parameter('min_height').value
        
        try:
            with open(charging_yaml_file, 'r') as file:
                data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f'Failed to read YAML file: {e}')
            return

        # Parse the charging area
        charging_area = data.get('charging_area', {})
        self.charging_area = [charging_area.get('upper_left', None), charging_area.get('down_right', None)]

        
        self.odom_subscribers = {}
        self.battery_subscribers = {}
        self.odom_data = {}
        self.status_flags = {}
        self.publish_start = {}
        self.publish_stop = {}
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        for i  in range (1,self.num_cf+1):
            namespace = f'cf_{i}'
            topic = f'/{namespace}/odom'
            self.odom_data[namespace] = Odometry()
            self.status_flags[namespace] = BatteryState().POWER_SUPPLY_STATUS_DISCHARGING
            self.odom_subscribers[namespace] = self.create_subscription(
                Odometry,
                topic,
                lambda msg, ns=namespace: self.odom_callback(msg, ns),
                10
            )
            self.battery_subscribers[namespace] = self.create_subscription(
                BatteryState,
                f'/{namespace}/battery_status',
                lambda msg, ns=namespace: self.battery_callback(msg, ns),
                10
            )
            self.publish_start[namespace] = self.create_publisher(Bool, namespace + '/battery_charge/start', 10)
            self.publish_stop[namespace] = self.create_publisher(Bool, namespace + '/battery_charge/stop', 10)
        
    def odom_callback(self, msg, namespace):
        # Save the odometry data in the dictionary
        self.odom_data[namespace] = msg
        
    def battery_callback(self, msg, namespace):
        # Save the odometry data in the dictionary
        self.status_flags[namespace] = msg.power_supply_status
        

    def timer_callback(self):
        
        for key, msg in self.odom_data.items():
            if (msg.pose.pose.position.z < self.min_height) and (msg.pose.pose.position.x<self.charging_area[1][0])  and (msg.pose.pose.position.x>self.charging_area[0][0]) and (msg.pose.pose.position.y<self.charging_area[0][1])  and (msg.pose.pose.position.y>self.charging_area[1][1]):
                if self.status_flags[key] == BatteryState().POWER_SUPPLY_STATUS_DISCHARGING:
                    msg = Bool()
                    msg.data = True
                    self.publish_start[key].publish(msg)
            else:
                if self.status_flags[key] == BatteryState().POWER_SUPPLY_STATUS_CHARGING:
                    msg = Bool()
                    msg.data = True
                    self.publish_stop[key].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GzServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
