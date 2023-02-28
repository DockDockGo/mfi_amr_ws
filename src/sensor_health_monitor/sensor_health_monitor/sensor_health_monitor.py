import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory
import yaml
import importlib
from functools import partial
from mfi_amr_msgs.msg import SensorHealth
from std_msgs.msg import Header


class SensorHealthMonitor(Node):

    def __init__(self):
        super().__init__('sensor_health_monitor')

        self.config_file = os.path.join(
            get_package_share_directory('sensor_health_monitor'),
            'config',
            'config.yaml'
        )

        with open(self.config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        self.get_logger().info(f"Using sensor config: {self.config}")

        self.window_period = self.config["window_period"]

        self.subscriptions_ = []
        self.msg_cntr = {}
        self.exp_pub_rates = {}
        self.sensor_topic_map = {}

        for sensor in self.config["sensors"].keys():
            self.sensor_topic_map[sensor] = []
            for topic_config in self.config["sensors"][sensor]["topics"]:
                type_module, type_class = topic_config["type"].rsplit('.', 1)
                try:
                    msg_mod = importlib.import_module(type_module)
                    msg_cls = getattr(msg_mod, type_class)
                    self.msg_cntr[topic_config["name"]] = 0
                    self.exp_pub_rates[topic_config["name"]] = topic_config["pub_rate"]
                    self.sensor_topic_map[sensor].append(topic_config["name"])

                    self.subscriptions_.append(
                        self.create_subscription(
                            msg_cls,
                            topic_config["name"],
                            # TODO: Replace this callback with something else. If for some reason topic_callback is
                            #  blocked, all callbacks will be blocked
                            partial(self.topic_callback, topic_config["name"]),
                            10)
                    )
                except ModuleNotFoundError as e:
                    self.get_logger().error(f"ModuleNotFoundError: {e}")
                except AttributeError as e:
                    self.get_logger().error(f"AttributeError: {e}")

        self.publisher_ = self.create_publisher(SensorHealth, 'sensor_health', 10)
        self.timer = self.create_timer(self.window_period, self.timer_callback)

    def timer_callback(self):
        msg = SensorHealth()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        for sensor, topics in self.sensor_topic_map.items():
            for topic in topics:
                msg.sensor = sensor
                if self.msg_cntr[topic] / self.window_period < 0.9*self.exp_pub_rates[topic]:
                    msg.sensor_ok = False
                else:
                    msg.sensor_ok = True
                self.publisher_.publish(msg)
                self.msg_cntr[topic] = 0

    def topic_callback(self, topic, msg):
        self.msg_cntr[topic] += 1


def main(args=None):
    rclpy.init(args=args)

    sensor_health_monitor = SensorHealthMonitor()

    rclpy.spin(sensor_health_monitor)

    sensor_health_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
