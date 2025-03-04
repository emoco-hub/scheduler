# ROS 2 Scheduler Node

This ROS 2 package provides a **Scheduler Node** that allows scheduling events for different receivers. 
Schedules are stored as **ROS 2 parameters** and persisted to disk, enabling scheduled events to survive node restarts.

## Features

- **Schedule one-time or recurring events** using a cron-like syntax or human-readable intervals.
- **Stores schedules in ROS parameters (`/scheduler/schedules`)** for persistence.
- **Publishes events on a shared topic (`/scheduler/events`)**, which nodes can subscribe to.
- **Schedules persist across restarts** by saving them to disk (`~/ros2_ws/scheduler_schedules.json`).
- **Supports dynamic schedule modification** via `ros2 param set`.

---

## 📦 **Installation Instructions**

1. **Install dependencies:**

   ```bash
   pip install schedule
   ```

2. **Extract the package into your ROS 2 workspace:**

   ```bash
   cd ~/ros2_ws/src/
   unzip /path/to/scheduler_node.zip
   ```

3. **Build the package:**

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select scheduler_node --symlink-install
   ```

4. **Source your workspace:**

   ```bash
   source install/setup.bash
   ```

---

## 🚀 **Usage**

### **1️⃣ Run the Scheduler Node with default schedule config**
```bash
ros2 run scheduler_node scheduler_node --ros-args --params-file ./src/scheduler_node/config/default_params.yaml
```

### **2️⃣ Schedule an Event**
Use **`ros2 param set`** to define a scheduled action:
```bash
ros2 param set /scheduler schedules '{"light_1": {"action": "ON", "schedule": "every 10 seconds"}}'
```

### **3️⃣ Check Scheduled Events**
```bash
ros2 param get /scheduler schedules
```

Expected output:
```yaml
value: '{"light_1": {"action": "ON", "schedule": "every 10 seconds"}}'
```

### **4️⃣ Modify a Schedule**
Change the schedule dynamically:
```bash
ros2 param set /scheduler schedules '{"light_1": {"action": "OFF", "schedule": "0 9 * * *"}}'
```

### **5️⃣ Restart the Node (Schedules Persist)**
```bash
ros2 run scheduler_node scheduler_node
ros2 param get /scheduler schedules  # Should still contain saved schedules
```

---

## 📡 **Subscribing to Scheduled Events**
Nodes can subscribe to `/scheduler/events` and filter messages based on `receiver_id`.

### Example: **Subscriber Node (Python)**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EventListener(Node):
    def __init__(self):
        super().__init__('event_listener')
        self.subscription = self.create_subscription(
            String, '/scheduler/events', self.event_callback, 10)

    def event_callback(self, msg):
        receiver_id, action = msg.data.split(',')
        if receiver_id == 'light_1':
            self.get_logger().info(f"Received event: {action}")

def main(args=None):
    rclpy.init(args=args)
    node = EventListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Run the subscriber:
```bash
ros2 run my_package event_listener
```

---

## ⚙ **Internals & How It Works**
### 🔹 **Storing & Persisting Schedules**
- **Schedules are stored in the ROS parameter server (`/scheduler/schedules`).**
- **They are also saved to a JSON file (`~/ros2_ws/scheduler_schedules.json`)**.
- On startup, the node **loads stored schedules** and **re-registers active events**.

### 🔹 **Event Execution**
1. The scheduler **monitors the current time**.
2. When a scheduled event occurs, it **publishes a message** to `/scheduler/events`.
3. Nodes listening to `/scheduler/events` **receive and process the event**.

---

## 🛠 **Development & Debugging**
### **🔹 Check Running Schedules**
```bash
ros2 param get /scheduler schedules
```

### **🔹 Manually Publish an Event**
```bash
ros2 topic pub /scheduler/events std_msgs/String "{data: 'light_1,ON'}"
```

### **🔹 Monitor Events**
```bash
ros2 topic echo /scheduler/events
```

---

## 📜 **License**
This package is released under the **Apache-2.0 License**.
