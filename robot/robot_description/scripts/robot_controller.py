#!/usr/bin/env python3

import rclpy
import math
import time
import threading
import tkinter as tk
from tkinter import messagebox
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

WAYPOINTS = {
    "home": (0.0, 0.0),
    "kitchen": (-0.105, 6.085),
    "table1": (0.7907, -5.8238),
    "table2": (4.503, -8.776),
    "table3": (5.205, -3.553)
}
GOAL_THRESHOLD = 0.30         # Set lower than 0.30 m for accuracy


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.task_queue = []
        self.goal_handle = None
        self.awaiting_confirmation = False
        self.current_task = None
        self.goal_pose = (0.0, 0.0)
        self.kitchen_after_cancel = False


    def cancel_specific_table(self, table_name):
        if table_name in self.task_queue:
            self.task_queue.remove(table_name)
            self.get_logger().info(f"❌ {table_name} canceled from queue.")

        # If the robot is currently going to the canceled table
        if self.current_task == table_name:
            self.get_logger().info(f"🛑 Canceling current task: {table_name}")
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            # Flag that a table was canceled so we revisit kitchen later
            self.kitchen_after_cancel = True
            self.awaiting_confirmation = False
            # ✅ Check if no other table tasks remain
            remaining_tables = [t for t in self.task_queue if "table" in t]
            if not remaining_tables:
                # Clear any pending "home" to reinsert properly
                if "home" in self.task_queue:
                    self.task_queue.remove("home")
                # ✅ Insert kitchen before going home
                self.task_queue.insert(0, "kitchen")
                self.task_queue.append("home")
            self.go_to_next_task()



    def process_orders(self, orders):
        self.task_queue = ["kitchen"] + orders + ["home"]
        self.go_to_next_task()

    def go_to_next_task(self):
        if not self.task_queue:
            gui.update_status("✅ All tasks done.")
            return

        self.current_task = self.task_queue.pop(0)
        x, y = WAYPOINTS[self.current_task]
        self.goal_pose = (x, y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        gui.update_status(f"🚀 Navigating to {self.current_task}...")
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

    def feedback_callback(self, feedback_msg):
        if self.awaiting_confirmation:
            return

        current_pos = feedback_msg.feedback.current_pose.pose.position
        goal_x, goal_y = self.goal_pose
        dist = math.hypot(goal_x - current_pos.x, goal_y - current_pos.y)

        if dist <= GOAL_THRESHOLD:
            self.get_logger().info(f"📍Reached {self.current_task}")
            self.awaiting_confirmation = True
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            gui.root.after(100, self.process_arrival)

    def process_arrival(self):
        # No confirmation at home
        if self.current_task == "home":
            gui.update_status("🏠 Reached home. Mission complete.✔️")
            self.awaiting_confirmation = False
            return

        # Skip confirmation if coming to kitchen after table cancel
        if self.current_task == "kitchen" and self.kitchen_after_cancel:
            gui.update_status("🍽️ Returned to kitchen after cancellation.")
            self.kitchen_after_cancel = False
            self.awaiting_confirmation = False
            self.go_to_next_task()
            return

        confirm = gui.ask_confirm(f"Reached {self.current_task}. Confirm?")

        if not confirm:
            if self.current_task == "kitchen":
                gui.update_status("❌ Kitchen task canceled. Returning home.")
                self.task_queue = ["home"]
                self.awaiting_confirmation = False
                self.go_to_next_task()
                return
            elif "table" in self.current_task:
                gui.update_status(f"❌ {self.current_task} canceled. Continuing remaining tasks.")
                self.kitchen_after_cancel = True  # flag for later kitchen revisit

        self.awaiting_confirmation = False

        # If this was the last table, and at least one was canceled, insert kitchen before home
        if not self.task_queue:
            self.go_to_next_task()
            return

        # Check if this is the last table
        if "table" in self.current_task:
            next_tasks = [t for t in self.task_queue if "table" in t]
            if not next_tasks and self.kitchen_after_cancel:
                # All tables done, insert kitchen before home
                if "home" in self.task_queue:
                    self.task_queue.remove("home")
                self.task_queue.insert(0, "kitchen")
                self.task_queue.append("home")

        self.go_to_next_task()





class RobotGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.root.title("Robot Navigation GUI")
        self.cancel_buttons_frame = tk.Frame(root)
        self.cancel_buttons_frame.pack()
        self.cancel_buttons = {}
        tk.Label(root, text="Enter table orders (comma-separated):").pack()
        self.entry = tk.Entry(root)
        self.entry.pack()

        tk.Button(root, text="Start", command=self.start_orders).pack()
        tk.Button(root, text="Cancel Mission", command=self.cancel_mission).pack()

        self.status_label = tk.Label(root, text="Status: Idle")
        self.status_label.pack()

    def start_orders(self):
        entries = [x.strip() for x in self.entry.get().split(',')]
        valid_orders = [e for e in entries if e in WAYPOINTS and "table" in e]
        if valid_orders:
            self.generate_cancel_buttons(valid_orders)
            self.controller.process_orders(valid_orders)
        else:
            messagebox.showerror("Invalid input", "Please enter valid tables (e.g. table1,table2).")

    def generate_cancel_buttons(self, orders):
        # Clear old buttons
        for btn in self.cancel_buttons.values():
            btn.destroy()
        self.cancel_buttons.clear()

        for table in orders:
            btn = tk.Button(self.cancel_buttons_frame, text=f"Cancel {table}",
                            command=lambda t=table: self.cancel_order(t))
            btn.pack(side=tk.LEFT)
            self.cancel_buttons[table] = btn

    def cancel_order(self, table_name):
        self.controller.cancel_specific_table(table_name)
        self.cancel_buttons[table_name].config(state=tk.DISABLED)


    def cancel_mission(self):
        confirm = self.ask_confirm("Cancel mission and return home?")
        if confirm:
            self.controller.task_queue = ["home"]
            self.controller.go_to_next_task()

    def ask_confirm(self, message):
        confirm_window = tk.Toplevel(self.root)
        confirm_window.title("Confirm")
        tk.Label(confirm_window, text=message).pack()

        response = tk.BooleanVar(value=False)

        def on_yes():
            response.set(True)
            confirm_window.destroy()

        def on_no():
            response.set(False)
            confirm_window.destroy()

        tk.Button(confirm_window, text="Confirm", command=on_yes).pack()
        tk.Button(confirm_window, text="Cancel", command=on_no).pack()

        confirm_window.after(10000, confirm_window.destroy)
        confirm_window.grab_set()
        self.root.wait_window(confirm_window)
        return response.get()

    def update_status(self, msg):
        self.status_label.config(text=f"Status: {msg}")


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    global gui
    root = tk.Tk()
    gui = RobotGUI(root, controller)

    ros_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    ros_thread.start()

    root.mainloop()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

