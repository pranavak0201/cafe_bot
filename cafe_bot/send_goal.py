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

        # State variables
        self.orders = []
        self.current_order_index = 0
        self.current_table_index = 0
        self.state = "idle"
        self.timeout_sec = 10
        self.result_future = None
        
        # Timing variables
        self.kitchen_start_time = None
        self.table_start_time = None
        
        # Confirmation tracking
        self.kitchen_confirmed = False
        self.table_confirmations = {}
        
        # Task execution flags
        self.task_active = False
        self.needs_kitchen_return = False

        # Subscribe to orders
        self.subscription = self.create_subscription(
            String,
            '/orders',
            self.order_callback,
            10
        )

        # Timer to process orders
        self.timer = self.create_timer(0.2, self.process_orders)

    def order_callback(self, msg):
        """Handle incoming order messages"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"🔍 RECEIVED MESSAGE: {data}")
            
            # Handle different message types
            if 'action' in data:
                if data['action'] == 'new_order':
                    self.handle_new_order(data)
                elif data['action'] == 'confirm':
                    self.handle_confirmation(data)
                elif data['action'] == 'cancel':
                    self.handle_cancellation(data)
            else:
                # Legacy format - ALWAYS treat as case 2 (requires confirmation/timeout)
                self.handle_simple_order(data)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def handle_simple_order(self, data):
        """Handle simple order format - ALWAYS case 2"""
        table = data.get('table', '')
        status = data.get('status', 'active')
        confirmed = data.get('confirmed', None)
        
        self.get_logger().info(f"🔍 SIMPLE ORDER: table={table}, status={status}, confirmed={confirmed}")
        
        if table:
            if status == 'canceled':
                # Handle cancellation
                self.handle_order_cancellation(table)
                return
            elif confirmed is not None:
                # This is a confirmation message
                self.handle_order_confirmation(table, confirmed)
                return
            else:
                # New order - FORCE case 2 behavior
                order = {
                    'tables': [table],
                    'status': 'active',
                    'case_type': 2,  # HARDCODED - requires confirmation and timeout
                    'requires_confirmation': True
                }
                self.orders.append(order)
                self.get_logger().info(f"✅ NEW ORDER: {table} added as CASE 2 (requires confirmation)")
                
                # Start processing if idle
                if self.state == "idle" and not self.task_active:
                    self.start_order_processing()

    def handle_new_order(self, data):
        """Handle structured new order"""
        tables = data.get('tables', [])
        if not tables and 'table' in data:
            tables = [data['table']]
            
        if tables:
            order = {
                'tables': tables,
                'status': 'active',
                'case_type': data.get('case_type', 2),
                'requires_confirmation': True
            }
            self.orders.append(order)
            self.get_logger().info(f"✅ STRUCTURED ORDER: {tables}, case: {order['case_type']}")
            
            if self.state == "idle" and not self.task_active:
                self.start_order_processing()

    def handle_order_confirmation(self, table, confirmed):
        """Handle confirmation for a specific table"""
        if confirmed:
            if not self.kitchen_confirmed:
                self.kitchen_confirmed = True
                self.get_logger().info(f"✅ KITCHEN confirmation received")
            else:
                self.table_confirmations[table] = True
                self.get_logger().info(f"✅ TABLE {table} confirmation received")

    def handle_order_cancellation(self, table):
        """Handle order cancellation"""
        if self.orders and self.current_order_index < len(self.orders):
            current_order = self.orders[self.current_order_index]
            if table in current_order['tables']:
                if 'canceled_tables' not in current_order:
                    current_order['canceled_tables'] = []
                current_order['canceled_tables'].append(table)
                self.get_logger().info(f"❌ ORDER CANCELED: {table}")
                
                # Handle transit cancellation
                if self.state in ['to_kitchen', 'to_table'] and self.task_active:
                    current_order['case_type'] = 4
                    self.handle_transit_cancellation(table)

    def handle_transit_cancellation(self, table):
        """Handle cancellation during transit"""
        if self.state == 'to_kitchen':
            self.get_logger().info(f"🚫 Canceled while going to kitchen - returning home")
            self.send_goal("home")
            self.state = "returning_home"
        elif self.state == 'to_table':
            self.get_logger().info(f"🚫 Canceled while going to table - returning to kitchen")
            self.send_goal("kitchen")
            self.state = "returning_to_kitchen"

    def start_order_processing(self):
        """Start processing the current order"""
        if self.current_order_index >= len(self.orders):
            return
            
        self.task_active = True
        self.current_table_index = 0
        self.kitchen_confirmed = False
        self.table_confirmations.clear()
        
        order = self.orders[self.current_order_index]
        self.get_logger().info(f"🚀 STARTING ORDER {self.current_order_index + 1}: {order}")
        
        # Always go to kitchen first
        self.send_goal("kitchen")
        self.state = "to_kitchen"
        self.kitchen_start_time = time.time()

    def process_orders(self):
        """Main processing loop"""
        if not self.orders:
            return
            
        # Wait for navigation to complete
        if self.result_future and not self.result_future.done():
            return
            
        # Handle different states
        if self.state == "idle" and not self.task_active:
            if self.current_order_index < len(self.orders):
                self.start_order_processing()
                
        elif self.state == "to_kitchen":
            self.handle_kitchen_phase()
            
        elif self.state == "to_table":
            self.handle_table_phase()
            
        elif self.state == "returning_to_kitchen":
            self.get_logger().info("📍 Back at kitchen - now going home")
            self.send_goal("home")
            self.state = "returning_home"
            
        elif self.state == "returning_home":
            self.handle_completion()

    def handle_kitchen_phase(self):
        """Handle robot at kitchen"""
        if not self.task_active:
            return
            
        current_order = self.orders[self.current_order_index]
        case_type = current_order.get('case_type', 2)
        elapsed = time.time() - self.kitchen_start_time
        
        self.get_logger().info(f"🍽️ KITCHEN: case={case_type}, elapsed={elapsed:.1f}s, confirmed={self.kitchen_confirmed}")
        
        # Check for confirmation first
        if self.kitchen_confirmed:
            self.get_logger().info("✅ Food collected - going to tables")
            self.go_to_next_table()
            return
        
        # Check for timeout
        if elapsed > self.timeout_sec:
            if case_type in [2, 3]:  # Cases that require confirmation
                self.get_logger().info("⏰ KITCHEN TIMEOUT - returning home (no confirmation)")
                self.send_goal("home")
                self.state = "returning_home"
                self.needs_kitchen_return = False
                return
            else:
                # Case 1 - no confirmation needed
                self.get_logger().info("📦 Food ready (no confirmation needed) - going to tables")
                self.go_to_next_table()

    def go_to_next_table(self):
        """Go to next table in order"""
        current_order = self.orders[self.current_order_index]
        tables = current_order['tables']
        canceled = current_order.get('canceled_tables', [])
        
        # Find next valid table
        while self.current_table_index < len(tables):
            table = tables[self.current_table_index]
            if table not in canceled:
                self.get_logger().info(f"🚚 Going to {table}")
                self.send_goal(table)
                self.state = "to_table"
                self.table_start_time = time.time()
                return
            else:
                self.get_logger().info(f"⏭️ Skipping canceled {table}")
                self.current_table_index += 1
        
        # No more tables
        self.finish_order()

    def handle_table_phase(self):
        """Handle robot at table"""
        if not self.task_active:
            return
            
        current_order = self.orders[self.current_order_index]
        tables = current_order['tables']
        current_table = tables[self.current_table_index]
        case_type = current_order.get('case_type', 2)
        elapsed = time.time() - self.table_start_time
        
        self.get_logger().info(f"🏠 TABLE {current_table}: case={case_type}, elapsed={elapsed:.1f}s")
        
        # Check for confirmation
        if current_table in self.table_confirmations:
            self.get_logger().info(f"✅ Delivered to {current_table}")
            self.current_table_index += 1
            
            if self.current_table_index < len(tables):
                self.go_to_next_table()
            else:
                self.finish_order()
            return
        
        # Check for timeout
        if elapsed > self.timeout_sec:
            if case_type == 3:
                # Case 3b - return to kitchen first
                self.get_logger().info(f"⏰ Table timeout - returning to kitchen")
                self.send_goal("kitchen")
                self.state = "returning_to_kitchen"
                self.needs_kitchen_return = True
                return
            elif case_type == 6:
                # Case 6 - skip and continue
                self.get_logger().info(f"⏰ Table timeout - moving to next table")
                self.needs_kitchen_return = True
                self.current_table_index += 1
                
                if self.current_table_index < len(tables):
                    self.go_to_next_table()
                else:
                    self.finish_order()
                return
            else:
                # Default - treat as delivered for case 1
                self.get_logger().info(f"✅ Delivered to {current_table} (timeout, no confirmation needed)")
                self.current_table_index += 1
                
                if self.current_table_index < len(tables):
                    self.go_to_next_table()
                else:
                    self.finish_order()

    def finish_order(self):
        """Finish current order"""
        current_order = self.orders[self.current_order_index]
        case_type = current_order.get('case_type', 2)
        
        self.get_logger().info(f"✅ ORDER {self.current_order_index + 1} COMPLETED")
        
        # Determine return path
        if self.needs_kitchen_return or case_type in [6, 7]:
            self.get_logger().info("🔄 Returning to kitchen first")
            self.send_goal("kitchen")
            self.state = "returning_to_kitchen"
        else:
            self.get_logger().info("🏠 Returning home")
            self.send_goal("home")
            self.state = "returning_home"

    def handle_completion(self):
        """Handle order completion"""
        self.get_logger().info("🏠 Returned home - order complete")
        self.task_active = False
        self.current_order_index += 1
        self.current_table_index = 0
        self.state = "idle"
        self.needs_kitchen_return = False
        
        # Process next order if available
        if self.current_order_index < len(self.orders):
            self.get_logger().info("📋 More orders in queue...")

    def send_goal(self, location):
        """Send navigation goal"""
        if location not in self.positions:
            self.get_logger().error(f"❌ Unknown location: {location}")
            return
            
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
        self.get_logger().info(f"📍 Navigating to {location}")

    def goal_response_callback(self, future):
        """Handle navigation response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Navigation goal rejected")
            return
        
        self.get_logger().info("✅ Navigation goal accepted")
        self.result_future = goal_handle.get_result_async()

def main(args=None):
    rclpy.init(args=args)
    node = CafeBotNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()