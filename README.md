# 🚀 CaféBot: Cafe Robot Using ROS2 + Nav2
CaféBot is an intelligent butler robot designed to streamline food delivery operations in busy cafés. It automates order pickup and delivery, reducing employee workload and improving efficiency. The robot follows a structured workflow—starting from the home position, picking up food from the kitchen, and delivering it to customer tables. It handles multiple orders, cancellations, and confirmation-based interactions, ensuring a smooth dining experience. An autonomous café robot powered by ROS 2 and Nav2. It delivers orders by navigating to waypoints like kitchen and tables, waits for user confirmation at each stop, and handles dynamic task cancellations. A Tkinter GUI provides real-time control, status updates, and mission customization.

---

## 🚀 Features  
✅ **Autonomous Navigation: Uses Nav2 to move between predefined waypoints (home, kitchen, tables) 📍**  
✅ **Intelligent Task Queue: Executes tasks in sequence with dynamic handling of new orders and cancellations 🧠**  
✅ **Tkinter GUI Control: Real-time graphical interface to start missions, view status, and cancel specific table orders 🖥️**  
✅ **Stop & Confirm: Waits for user confirmation at each waypoint before proceeding 🛑**  
✅ **Timeout Mechanism: Automatically proceeds if confirmation is not received within a time window ⏳**  
✅ **Handles Multiple Orders Efficiently: Manages multiple simultaneous table deliveries with smart sequencing 📦**  
✅ **Smart Return Logic: Returns to the kitchen automatically if any table task is canceled 🔁**  
✅ **Mission End Logic: Returns home after completing all delivery tasks or upon cancellation 🏠**  
✅ **Adaptive to Dynamic Environments: Designed to operate in busy café spaces with flexible navigation logic 🌐**  
✅ **Real-Time Feedback: Monitors position and cancels goal when within threshold distance for better interaction 🔄**  

---

## 🛠️ Installation  

### 1️⃣ **Clone the Repository**  
```bash
cd ~/ros_ws/src
git clone https://github.com/yashbhaskar/cafe-robot-using-nav2.git
cd ~/ros_ws
```

### 2️⃣ **Build the Package** 
```bash
colcon build
source install/setup.bash
```

---

### 1️⃣ Launch Gazebo and State_Publisher:
```bash
ros2 launch robot_description gazebo.launch.py
ros2 launch robot_description state_publisher.launch.py
```
![Screenshot from 2025-03-02 02-38-27](https://github.com/user-attachments/assets/84597889-09bd-4ac2-ad3b-f2dba351bae5)
![Screenshot from 2025-03-02 02-39-27](https://github.com/user-attachments/assets/8ef47259-9354-41f7-88e6-0e2f0b051700)

### 2️⃣ Run the Robot Controller Command Node:
```bash
ros2 run robot_description robot_controller.py
```

### 3️⃣ Give Table inputs on gui and Start:
```bash
table1,table2,table3
```
![Screenshot from 2025-03-02 02-42-27](https://github.com/user-attachments/assets/62d263ae-bd8f-44b9-8d6f-c698fe49207e)

### 4️⃣ Give input Confirm/Cancel:
Once the node starts, it will listen for voice commands such as:
``"If Confirm"`` – Move towards tables for delivery
``"If Cancel"`` – Move towards Home
![Screenshot from 2025-03-02 02-43-51](https://github.com/user-attachments/assets/b6c0c0b1-d167-4890-bde1-5d54251bb14e)

---

## 📹 Working Videos

https://drive.google.com/drive/u/0/folders/1HVsILKfBWDLQr8Mfs-OQ0qOrstMorvIb

---
