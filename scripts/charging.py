#!/usr/bin/env python3
import rclpy
import yaml
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

class GzServiceNode(Node):
    def __init__(self):
        super().__init__('gz_service_charging_node')
        self.declare_parameter('num_cf', 4)
        self.declare_parameter('charging_area_yaml', '')
        self.declare_parameter('min_height', 0.2)
        
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
        self.get_logger().info(f"Loaded charging points{self.charging_area} from {charging_yaml_file}")
        self.landing_sites = data.get('landing_areas', {})
        self.get_logger().info(f"Loaded {len(self.landing_sites)}  landing areas points from {charging_yaml_file}")

        
        self.pose_subscribers = {}
        self.battery_subscribers = {}
        self.pose_data = {}
        self.status_flags = {}
        self.publish_start = {}
        self.publish_stop = {}
        self.publish_stop_drain = {}
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        for i  in range (1,self.num_cf+1):
            namespace = f'cf_{i}'
            topic = f'/{namespace}/pose'
            self.pose_data[namespace] = PoseStamped()
            self.status_flags[namespace] = BatteryState().POWER_SUPPLY_STATUS_DISCHARGING
            self.pose_subscribers[namespace] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, ns=namespace: self.pose_callback(msg, ns),
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
            self.publish_stop_drain[namespace] = self.create_publisher(Empty, namespace + '/stop_power_drain', 10)
        
        
    def odom_callback(self, msg, namespace):
        # Save the odometry data in the dictionary
        self.odom_data[namespace] = msg
    
    def pose_callback(self, msg, namespace):
        # Save the pose data in the dictionary
        self.pose_data[namespace] = msg
        
    def battery_callback(self, msg, namespace):
        # Save the battery data in the dictionary
        self.status_flags[namespace] = msg.power_supply_status
    
    def is_inside_landing_site(self, x, y, z):
        for i, site in enumerate(self.landing_sites, start=1):
            coords = site.get('coords', [])
            if len(coords) < 4:
                continue

            xs = [p[0] for p in coords]
            ys = [p[1] for p in coords]
            zs = [p[2] for p in coords]
            z_ref = sum(zs) / len(zs)

            if (min(xs) <= x <= max(xs)) and (min(ys) <= y <= max(ys)):
                if abs(z - z_ref) <= self.min_height:
                    return True, z_ref, i  
        return False, None, None
        

    def timer_callback(self):
        
        for key, msg in self.pose_data.items():
            x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

            if (z < self.min_height) and (x<self.charging_area[1][0])  and (x>self.charging_area[0][0]) and (y<self.charging_area[0][1])  and (y>self.charging_area[1][1]):
                if self.status_flags[key] == BatteryState().POWER_SUPPLY_STATUS_DISCHARGING:
                    msg = Bool()
                    msg.data = True
                    self.publish_start[key].publish(msg)
            else:
                if self.status_flags[key] == BatteryState().POWER_SUPPLY_STATUS_CHARGING:
                    msg = Bool()
                    msg.data = True
                    self.publish_stop[key].publish(msg)
                    
            inside, z_ref, idx = self.is_inside_landing_site(x, y, z)
            if inside and self.status_flags[key] == BatteryState().POWER_SUPPLY_STATUS_DISCHARGING:
                self.get_logger().info(f"{key} inside landing site {idx} (z={z_ref:.2f}) publish to /stop_power_drain")
                self.publish_stop_drain[key].publish(Empty())


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
