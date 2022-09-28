import random
import time
import numpy as np
from functools import partial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Point
from synchronous_msgs.msg import NotifyDelay, NotifyPause, NotifyTaskComplete, MissionStatus
from starling_allocator_msgs.msg import Allocations

class Monitor(Node):

    def __init__(self):
        super().__init__('sync_monitor')
        self.notify_delay_sub = self.create_subscription(NotifyDelay, '/monitor/notify_delay', self.notify_delay_cb, 10)

        # Mission Monitoring
        self.mission_start_sub = self.create_subscription(Empty, '/mission_start', self.mission_start_cb, 10)
        self.mission_abort_sub = self.create_subscription(Empty, '/mission_abort', self.mission_abort_cb, 10)
        self.emergency_stop_sub = self.create_subscription(Empty, '/emergency_stop', self.mission_abort_cb, 10)
        self.notify_task_complete_sub = self.create_subscription(NotifyTaskComplete, '/monitor/notify_task_complete', self.notify_task_complete_cb, 10)
        self.allocation_sub = self.create_subscription(Allocations, '/current_allocated_trajectory', self.current_allocated_trajectory_cb, 10)

        self.mission_complete_pub = self.create_publisher(MissionStatus, '/monitor/mission_status', 10)
        self.mission_abort_pub = self.create_publisher(Empty, '/mission_abort', 10)


        self.reset()
        self.get_logger().info("Initialised")

    def reset(self):
        self.mission_in_progress = False
        self.mission_start_time = None
        self.mission_status_msg = MissionStatus()
        self.mission_status_msg.in_progress = False

        # Should be a mapping between the vehicle/mavros name
        # and the list of JointTrajectoryPoints which make up the trajectory
        self.trajectory_allocations = None
        self.task_list = []
        self.task_complete = []

        self.get_logger().info("Reset Complete")

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

    def mission_start_cb(self, _):
        self.mission_in_progress = True
        self.mission_start_time = self.get_clock().now()
        self.mission_status_msg.in_progress = True
        self.get_logger().info(f"Mission Monitor Starting with task list:\n{self.task_list}")

    def mission_abort_cb(self, _):
        
        if self.mission_in_progress:
            self.get_logger().info(f"Mission Monitor Aborting")
            self.mission_complete_pub.publish(self.mission_status_msg)

        self.reset()

    def normalise_coordinate(self, point_array):
        # Hard code partitioning of centroids
        if point_array[1] > 1.5:
            point_array[1] -= 3.0
        elif point_array[1] < -1.5:
            point_array[1] += 3.0
        return point_array

    def current_allocated_trajectory_cb(self, msg):
        if self.mission_in_progress:
            # Dont set anything if in progress
            return

        # Merge all allocations to find list of tasks
        task_set = {}
        task_id = 0
        allocs = {alloc.vehicle: [] for alloc in msg.allocation}
        for alloc in msg.allocation:
            for points in alloc.trajectory.points:
                point = points.positions[:3]
                point = self.normalise_coordinate(point)
                point = tuple([round(p, 2) for p in point])

                if point not in task_set:
                    task_set[point] = task_id
                    task_id += 1
                id = task_set[point]
                allocs[alloc.vehicle].append(id)

        self.task_list = [np.array(p) for p in sorted(task_set, key=task_set.get)]
        self.task_complete = [False for _ in self.task_list]
        self.trajectory_allocations = allocs # map vehicle_name to task id

        self.mission_status_msg.task_complete = self.task_complete
        plocs = []
        for p in self.task_list:
            a = Point()
            a.x = p[0]
            a.y = p[1]
            a.z = p[2]
            plocs.append(a)
        self.mission_status_msg.task_locations = plocs


    def notify_task_complete_cb(self, msg):
        current_time = self.get_clock().now()

        tn = msg.task_number
        tx = round(msg.task_location.position.x, 2)
        ty = round(msg.task_location.position.y, 2)
        tz = round(msg.task_location.position.z, 2)
        task_location = np.array([tx, ty, tz])

        # Normalise to center point
        task_location = self.normalise_coordinate(task_location)

        # self.get_logger().info(f"Vehicle {msg.vehicle_id} task {msg.task_number} at {task_location} received complete")
        dist_diff = np.linalg.norm(self.task_list - task_location, axis=1)
        # self.get_logger().info(f"Dist diff: {dist_diff}")
        task_idx = dist_diff.argmin()
        task_loc = self.task_list[task_idx]

        if self.task_complete[task_idx]:
            self.get_logger().info(f"Task {task_idx} at {task_loc} has been revisited")
        else:
            self.task_complete[task_idx] = True
            self.get_logger().info(f"Task {task_idx} at {task_loc} has been notified as complete")

        self.get_logger().info(f"Tasks Complete: {self.task_complete.count(True)}/{len(self.task_complete)}")
        
        # Set status message
        self.mission_status_msg.task_completed = int(task_idx)
        self.mission_status_msg.vehicle_id = msg.vehicle_id
        self.mission_status_msg.vehicle_location = msg.vehicle_location
        self.mission_status_msg.time_elapsed = (current_time - self.mission_start_time).to_msg()
        
        if all(self.task_complete):
            self.mission_status_msg.completed = True

        self.mission_complete_pub.publish(self.mission_status_msg)
        
        if self.mission_status_msg.completed:
            self.get_logger().info(f"All tasks have been complete, monitor resetting.")
            self.mission_abort_cb(None)
            self.mission_abort_pub.publish(Empty()) # Sending abort sigal to all.
            

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