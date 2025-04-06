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

## 🎮 Usage
### 1️⃣ Launch Robot:
```bash
ros2 launch robot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```
![Screenshot from 2025-03-02 02-38-27](https://github.com/user-attachments/assets/84597889-09bd-4ac2-ad3b-f2dba351bae5)
![Screenshot from 2025-03-02 02-39-27](https://github.com/user-attachments/assets/8ef47259-9354-41f7-88e6-0e2f0b051700)
![Screenshot from 2025-04-07 00-45-10](https://github.com/user-attachments/assets/c3383dea-c929-4fba-9eae-25f1c5428f9a)
![Screenshot from 2025-04-07 00-44-36](https://github.com/user-attachments/assets/5152c2b9-fd25-411d-a23b-eed98569a302)

### 2️⃣ Run the Robot Controller Command Node:
```bash
ros2 run robot_description robot_controller.py
```

### 3️⃣ Give Table inputs on gui and Start:
```bash
table1,table2,table3
```
![Screenshot from 2025-04-07 00-48-23](https://github.com/user-attachments/assets/60848c0b-fa8f-4066-9566-5ba5f90c24ca)

### 4️⃣ Give input Confirm/Cancel:
Once the node starts, it will listen for voice commands such as:
``"If Confirm"`` – Move towards tables for delivery
``"If Cancel"`` – Move towards Home
![Screenshot from 2025-04-07 00-49-39](https://github.com/user-attachments/assets/af31bab0-0261-4eab-882f-a0a147b03aca)

---

## 📹 Working Videos

https://drive.google.com/drive/u/0/folders/1HVsILKfBWDLQr8Mfs-OQ0qOrstMorvIb

---

## 📌 Handling Different Scenarios
| Scenario | Robot Behavior |
|----------|---------------|
| Order received | Moves from home → kitchen → table → home |
| No confirmation at kitchen | Waits for 10 second timeout, then returns home |
| No confirmation at table | Waits for 10 second timeout, then move kitchen before returning home |
| Task canceled | Returns to kitchen → home |
| Multiple orders (any order cancel) | Delivers to all tables and move kitchen before returning home |
| Multiple orders (any table order cancel before reaching table) | Delivers to remaining tables and move kitchen before returning home |
| Multiple orders (all confirm order) | Delivers to all tables and returning home |

---

## 🔄 Step-by-Step Navigation Logic  

### 1️⃣ Initial Movement  
- The robot takes inputs table1,table2,table3 and **starts** and moves to the **Kitchen** to pick up an item.  

### 2️⃣ Order Processing  
- If the order is **confirmed**, the robot proceeds to **Table 1** to deliver the item.
- If **Cancel table1** , it moves to **Table 2** without reaching table1.    
- If the order is **canceled**, the robot **immediately returns Home** without making any further stops.  

### 3️⃣ Delivery Check at Table 1  
- After reaching at **Table 1**, the robot takes order confirmation:  
  - If **Confirm** , it moves to **Table 2**.
  - If **Cancel table2** , it moves to **Table 3** without reaching table2.  
  - If **Cancel** , it moves to **Table 2**.after delivered all orders robot move to kitchen before home.
  - If **Not Give input** , it moves to **Table 2**.after delivered all orders robot move to kitchen before home.
  
## 4️⃣ Delivery Check at Table 2
- After reaching at **Table 2**, the robot takes order confirmation:  
  - If **Confirm** , it moves to **Table 3**.
  - If **Cancel table3** , it moves to **Kitchen** without reaching table3.   
  - If **Cancel** , it moves to **Table 3**.after delivered all orders robot move to kitchen before home.
  - If **Not Give input** , it moves to **Table 3**.after delivered all orders robot move to kitchen before home.

## 5️⃣ Delivery Check at Table 3
- After reaching at **Table 3**, the robot takes order confirmation:  
  - If **Confirm** , if all tables order is confirm then move to home. if any table order is cancel then move to kitchen before home.
  - If **Cancel** , robot move to kitchen before home.
  - If **Not Give input** , robot move to kitchen before home.

---

## 📂 Project Structure
```
│── robot_bringup/
|   │── launch/
|   │   ├── autobringup.launch.py
|   │── maps/
|   │   ├── my_map.pgm
|   │   ├── my_map.yaml
|   │── CMakeLists.txt
|   │── package.xml
│── robot_description/
|   │── launch/
|   │   ├── gazebo.launch.py
|   │   ├── rviz.launch.py
|   │   ├── state_publisher.launch.py
|   │── models/
|   │   ├── meshes
|   │   ├── urdf
|   │── rviz/
|   │   ├── robot.rviz
|   │   ├── urdf
|   │── scripts/
|   │   ├── robot_controller.py
|   │── worlds/
|   │   ├── cafe.sdf
|   │── CMakeLists.txt
|   │── package.xml
│── robot_navigation/
|   │── config/
|   │   ├── nav2_params_robot.yaml
|   │   ├── nav2_params_simulation.yaml
|   │── launch/
|   │   ├── navigation.launch.py
|   │── CMakeLists.txt
|   │── package.xml
│── robot_slam/
|   │── config/
|   │   ├── ekf.yaml
|   │   ├── slam.lua
|   │── launch/
|   │   ├── cartographer.launch.py
|   │── CMakeLists.txt
|   │── package.xml
```

---

## 📡 RQT Graph Visualization
Below is an RQT graph of the ROS 2 nodes and topics used in this package:

![Screenshot from 2025-04-07 01-06-20](https://github.com/user-attachments/assets/03d2de10-b0d8-4331-b956-605beee185f0)

---

## 🚀 Future Improvements

### 1️⃣ PID Controller for Smooth and Fast Navigation

- Implement a PID (Proportional-Integral-Derivative) controller to enhance motion control.
- Ensures precise speed adjustments, reducing jerky movements.
- Optimizes navigation by smoother turns and faster goal-reaching.

### 2️⃣ Multi-Robot Coordination

- Implement a centralized task management system for multiple robots.
- Prevents collisions and optimizes delivery paths for efficiency.

### 3️⃣ Mobile App for Remote Monitoring & Control

- Develop a mobile or web-based dashboard to track robot status.
- Users can override paths, cancel orders, or assign new tasks on the go.

---

## 🤝 Contributing

Feel free to fork this repository, create a pull request, or open an issue if you have suggestions or find bugs.

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com
📌 GitHub: https://github.com/yashbhaskar
