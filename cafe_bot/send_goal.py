#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import time
import json

class CafeBotNavigator(Node):
    def __init__(self):
        super().__init__('cafe_bot_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Waypoints
        self.positions = {
            "home":    [3.858, -0.120, 0.5911373481728225, 0.8065709116966736],
            "kitchen": [6.313, -0.405, 0.6167041396168981, 0.7871950229640565],
            "table1":  [6.580,  4.783, -0.6204545835750636, 0.7842423794470014],
            "table2":  [3.790,  4.746, -0.7861670990996075, 0.6180139903702083],
            "table3":  [0.610,  4.608, -0.8042113183456834, 0.5943434658887888],
        }

        self.orders = []  # dynamic queue

        self.current_index = 0
        self.timeout_sec = 10
        self.result_future = None
        self.state = "idle"

        # Subscribe to orders using String messages with JSON
        self.subscription = self.create_subscription(
            String,
            '/orders',
            self.order_callback,
            10
        )

        # Timer to check navigation and timeouts
        self.timer = self.create_timer(1.0, self.process_orders)

    def order_callback(self, msg):
        try:
            # Parse JSON data from String message
            order_data = json.loads(msg.data)
            table = order_data.get('table', '')
            confirmed = order_data.get('confirmed', False)
            status = order_data.get('status', '')
            
            # Check if table already in queue
            for order in self.orders:
                if order["table"] == table:
                    order["confirmed"] = confirmed
                    order["status"] = status
                    self.get_logger().info(f"Updated order: {table}, confirmed={confirmed}, status={status}")
                    return
            
            # New order
            self.orders.append({
                "table": table,
                "confirmed": confirmed,
                "status": status
            })
            self.get_logger().info(f"Received new order: {table}, confirmed={confirmed}, status={status}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in order message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing order: {e}")

    def process_orders(self):
        if not self.orders or self.result_future and not self.result_future.done():
            return

        if self.current_index >= len(self.orders):
            # Return Home after finishing
            if self.state != "done":
                self.get_logger().info("✅ All deliveries done. Returning Home.")
                self.send_goal("home")
                self.state = "done"
            return

        order = self.orders[self.current_index]

        # Skip canceled orders
        if order["status"] != "active":
            self.get_logger().info(f"Skipping canceled order {order['table']}")
            self.current_index += 1
            return

        # Determine next action
        if self.state in ["idle", "done"]:
            self.get_logger().info(f"Going to Kitchen for {order['table']}")
            self.send_goal("kitchen")
            self.state = "to_kitchen"
            self.kitchen_start_time = time.time()
            return

        if self.state == "to_kitchen":
            if order["confirmed"]:
                self.get_logger().info(f"Picked up food for {order['table']}")
                self.send_goal(order["table"])
                self.state = "to_table"
            elif time.time() - self.kitchen_start_time > self.timeout_sec:
                self.get_logger().info(f"No confirmation at Kitchen for {order['table']}, returning Home")
                self.send_goal("home")
                self.state = "idle"
                self.current_index += 1
            return

        if self.state == "to_table":
            if order["confirmed"]:
                self.get_logger().info(f"Delivered order to {order['table']}")
                self.state = "idle"
                self.current_index += 1
            else:
                self.get_logger().info(f"No confirmation at Table {order['table']}, returning to Kitchen")
                self.send_goal("kitchen")
                self.state = "to_kitchen"
            return

    def send_goal(self, location):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.positions[location][0]
        pose.pose.position.y = self.positions[location][1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = self.positions[location][2]
        pose.pose.orientation.w = self.positions[location][3]

        goal_msg.pose = pose

        self.client.wait_for_server()
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"Sent goal to {location}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return
        self.get_logger().info("✅ Goal accepted")
        self.result_future = goal_handle.get_result_async()

def main(args=None):
    rclpy.init(args=args)
    node = CafeBotNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
