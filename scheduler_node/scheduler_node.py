import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType
import schedule
import time
import threading

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler')
        self.publisher = self.create_publisher(String, '/scheduler/events', 10)

        # Load schedules from parameters
        self.declare_parameter('schedules', [])
        self.schedules = self.load_schedules_from_params()

        # Register all loaded schedules
        for event in self.schedules:
            self.schedule_event(event["receiver_id"], event["action"], event["schedule"])

        # Start scheduling thread
        self.scheduler_thread = threading.Thread(target=self.run_schedule, daemon=True)
        self.scheduler_thread.start()

    def load_schedules_from_params(self):
        """Loads schedules from ROS parameters (list format)."""
        param_value = self.get_parameter('schedules').value
        if isinstance(param_value, list):
            return param_value  # Correct format
        else:
            self.get_logger().warn("Expected a list format for schedules, found something else.")
            return []

    def save_schedules_to_params(self):
        """Saves schedules as a list in ROS parameters."""
        self.set_parameters([rclpy.parameter.Parameter(
            'schedules', rclpy.Parameter.Type.PARAMETER_LIST, self.schedules)])

    def add_event(self, receiver_id, action, schedule_str):
        """Adds a new scheduled event."""
        new_event = {"receiver_id": receiver_id, "action": action, "schedule": schedule_str}
        self.schedules.append(new_event)
        self.save_schedules_to_params()
        self.schedule_event(receiver_id, action, schedule_str)
        self.get_logger().info(f"Scheduled {action} for {receiver_id} at {schedule_str}")

    def schedule_event(self, receiver_id, action, schedule_str):
        """Schedules an event based on the given schedule format."""
        if "every" in schedule_str:
            interval = int(schedule_str.split()[1])
            schedule.every(interval).seconds.do(self.trigger_event, receiver_id, action)
        else:
            schedule.every().day.at(schedule_str).do(self.trigger_event, receiver_id, action)

    def trigger_event(self, receiver_id, action):
        """Publishes an event when triggered by the schedule."""
        msg = String()
        msg.data = f"{receiver_id},{action}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Triggered {action} for {receiver_id}")

    def run_schedule(self):
        """Runs the scheduling loop."""
        while rclpy.ok():
            schedule.run_pending()
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
