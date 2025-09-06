# Cafe Bot Navigation System # 

## Overview
The Cafe Bot Navigation System is a ROS 2-based autonomous delivery robot that:
Navigates between home, kitchen, and customer tables
Handles order confirmations and timeouts
Processes multiple orders sequentially
Manages order cancellations and failures
---

## Package Structure
```bash
cafe_bot/
├── cafe_bot/
│   ├── cafe_bot/
│   │   ├── __init__.py
│   │   ├── send_goal.py          # Main navigation node
│   │   └── msg/
│   │       └── Order.msg         # (Future) Custom message definition
│   ├── launch/
│   │   ├── bringup_world.launch.py
│   │   └── nav2_bringup.launch.py
│   ├── worlds/
│   │   └── map1.world            # Gazebo world file
│   ├── maps/
│   │   ├── map1.pgm              # Occupancy grid map
│   │   └── map1.yaml             # Map metadata
│   ├── config/
│   │   └── nav2_params.yaml      # Navigation parameters
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
```
# Step 1: World Design
Created a cafe environment in Gazebo with:
Home position (robot charging station)
Kitchen area (food preparation)
Three customer tables (table1, table2, table3)
Clear navigation pathways

# Step 2: Mapping Process
Launched the simulation world, used Cartographer for SLAM to build the map, and then saved the generated map for navigation.

# Step 3: Coordinate Collection
```bash
self.positions = {
    "home":    [3.858, -0.120, 0.5911373481728225, 0.8065709116966736],
    "kitchen": [6.313, -0.405, 0.6167041396168981, 0.7871950229640565],
    "table1":  [6.580,  4.783, -0.6204545835750636, 0.7842423794470014],
    "table2":  [3.790,  4.746, -0.7861670990996075, 0.6180139903702083],
    "table3":  [0.610,  4.608, -0.8042113183456834, 0.5943434658887888],
}
```
# Core Logic Overview
The navigation node implements a state machine that manages orders, confirmations, and navigation tasks.
Main Components

State Variables

state: Current robot state (idle, to_kitchen, to_table, returning_to_kitchen, returning_home)

orders[]: Queue of active orders
current_order_index: Currently processed order
current_table_index: Current table within the order
task_active: Whether the robot is busy
Confirmation Tracking
kitchen_confirmed: Kitchen confirmation flag
table_confirmations{}: Tracks confirmations for each table
Timeout tracking via kitchen_start_time and table_start_time
Order Processing Flow
Wait for orders on /orders topic.
Move to kitchen → wait for kitchen confirmation (or timeout).
Deliver to each table in the order → wait for confirmation (or timeout).
Return to kitchen if more orders remain.
Return to home when all orders are done.

## How to Run
Follow these steps to run the simulation and test the navigation:
1.Launch the world
``` Bash
# In your first terminal
source ~/multi_map_ws/install/setup.bash
ros2 launch cafe_bot bringup_world.launch.py
```
2.Launch the Nav2 stack
``` Bash
# In your second terminal
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:=/home/ubuntu/cafe_bot/src/cafe_bot/maps/map2.yaml
```

3.Run the navigator node
``` Bash
# In your third terminal
ros2 run cafe_bot cafe_bot_navigator
```
# Order Publication Examples
Orders are published to the /orders topic as std_msgs/String messages containing JSON data.
Case 1 – No confirmation needed
``` Bash
ros2 topic pub /orders std_msgs/msg/String \
  '{"data": "{\"action\": \"new_order\", \"table\": \"table1\", \"case_type\": 1}"}' --once

```
Case 2 – Requires confirmation (default)
``` Bash
# Send new order (defaults to case_type=2)
ros2 topic pub /orders std_msgs/msg/String \
  '{"data": "{\"table\": \"table1\"}"}' --once

# Don't send confirmation → order will timeout automatically
```
Case 3 – Order cancellation
``` Bash
# Send new order
ros2 topic pub /orders std_msgs/msg/String \
  '{"data": "{\"table\": \"table1\"}"}' --once

# Cancel order within 2–3 seconds (while robot is moving to kitchen)
ros2 topic pub /orders std_msgs/msg/String \
  '{"data": "{\"table\": \"table1\", \"status\": \"canceled\"}"}' --once

```
# Notes
The system supports multiple sequential orders.
Most cases work as expected.
In some situations with multiple orders and confirmations, the robot may mis-handle the sequence — but re-publishing the command from the terminal corrects the behavior.
