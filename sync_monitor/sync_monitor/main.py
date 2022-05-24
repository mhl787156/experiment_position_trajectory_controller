import random
import time
import numpy as np
from functools import partial

import rclpy
from rclpy.node import Node

from synchronous_msgs.msg import NotifyDelay, NotifyPause


class Monitor(Node):

    def __init__(self):
        super().__init__('sync_monitor')
        self.notify_delay_sub = self.create_subscription(NotifyDelay, '/monitor/notify_delay', self.notify_delay_cb, 10)
        self.get_logger().info("Initialised")

    def notify_delay_cb(self, msg):
        self.get_logger().info(f'Delay received from {msg.vehicle_id}, with delay {msg.delay} expected at {msg.expected_arrival_time}, arrived at {msg.actual_arrival_time}')

        p_msg = NotifyPause()
        p_msg.delayed_vehicle_id = msg.vehicle_id
        p_msg.delay = msg.delay

        for vname in self.__get_current_vehicle_namespaces():
            if vname == msg.vehicle_id:
                continue

            topic = f'/{vname}/notify_pause'
            delay_pub = self.create_publisher(NotifyPause, topic, 10)
            delay_pub.publish(p_msg)

            self.get_logger().info(f'Forwarded pause message to {topic}')

    def __get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        # self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            # self.get_logger().info(topic_name)
            if 'mavros' in topic_name:
                name = topic_name.split('/')[1]
                if name == 'mavros':
                    name = ''
                namespaces.add(name)
        self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        return namespaces

def main(args=None):
    rclpy.init(args=args)
    mon = Monitor()
    rclpy.spin(mon)
    mon.destroy_node()
    rclpy.shutdown()