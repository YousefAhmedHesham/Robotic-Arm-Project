#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
import tkinter as tk
import threading

class RobotGUI(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.robot = MoveItPy(node_name="moveit_py_gui")
        self.arm = self.robot.get_planning_component("arm_group")
        self.hand = self.robot.get_planning_component("hand_group")
        self.running = False
        self.status_label = None  # Will be set from main

    def set_status(self, text):
        if self.status_label:
            self.status_label.config(text=text, fg="#00FF00")  # Bright green for better visibility

    def move_to_position(self, position_name, repetitions=1):
        if self.running:
            return

        self.running = True

        def movement_thread():
            for _ in range(repetitions):
                try:
                    self.arm.set_start_state_to_current_state()
                    self.arm.set_goal_state(configuration_name=position_name)
                    plan_result = self.arm.plan()
                    if plan_result:
                        self.robot.execute(plan_result.trajectory, controllers=[])
                except Exception as e:
                    self.get_logger().error(f"Movement failed: {str(e)}")
            self.running = False
            # Update status in GUI thread
            if self.status_label:
                self.status_label.after(0, self.set_status, f"State: {position_name} reached")

        threading.Thread(target=movement_thread).start()

    def open_hand(self):
        if self.running:
            return

        self.running = True

        def open_thread():
            try:
                self.hand.set_start_state_to_current_state()
                self.hand.set_goal_state(configuration_name="open_hand")
                hand_plan = self.hand.plan()
                if hand_plan:
                    self.robot.execute(hand_plan.trajectory, controllers=[])
            except Exception as e:
                self.get_logger().error(f"Open hand failed: {str(e)}")
            self.running = False
            if self.status_label:
                self.status_label.after(0, self.set_status, "State: Open hand done")

        threading.Thread(target=open_thread).start()

    def close_hand(self):
        if self.running:
            return

        self.running = True

        def close_thread():
            try:
                self.hand.set_start_state_to_current_state()
                self.hand.set_goal_state(configuration_name="close_hand")
                hand_plan = self.hand.plan()
                if hand_plan:
                    self.robot.execute(hand_plan.trajectory, controllers=[])
            except Exception as e:
                self.get_logger().error(f"Close hand failed: {str(e)}")
            self.running = False
            if self.status_label:
                self.status_label.after(0, self.set_status, "State: Close hand done")

        threading.Thread(target=close_thread).start()

    def run_full_sequence(self):
        if self.running:
            return

        self.running = True

        def sequence_thread():
            # Move to position_2 first
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(configuration_name="position_2")
            plan_result = self.arm.plan()
            if plan_result:
                self.robot.execute(plan_result.trajectory, controllers=[])

            # Move to each position 15 times
            for pos in ['position_0', 'position_1', 'position_2', 'position_3', 'position_4']:
                for _ in range(15):
                    try:
                        self.arm.set_start_state_to_current_state()
                        self.arm.set_goal_state(configuration_name=pos)
                        plan_result = self.arm.plan()
                        if plan_result:
                            self.robot.execute(plan_result.trajectory, controllers=[])
                    except Exception as e:
                        self.get_logger().error(f"Movement failed: {str(e)}")
                # Update status after each position
                if self.status_label:
                    self.status_label.after(0, self.set_status, f"State: {pos} reached")

            # Hand open/close sequence
            for _ in range(7):
                self.hand.set_start_state_to_current_state()
                self.hand.set_goal_state(configuration_name="open_hand")
                hand_plan = self.hand.plan()
                if hand_plan:
                    self.robot.execute(hand_plan.trajectory, controllers=[])
            if self.status_label:
                self.status_label.after(0, self.set_status, "State: Open hand done")

            for _ in range(7):
                self.hand.set_start_state_to_current_state()
                self.hand.set_goal_state(configuration_name="close_hand")
                hand_plan = self.hand.plan()
                if hand_plan:
                    self.robot.execute(hand_plan.trajectory, controllers=[])
            if self.status_label:
                self.status_label.after(0, self.set_status, "State: Close hand done")

            
            if self.status_label:
                self.status_label.after(0, self.set_status, "Full sequence done")

            self.running = False

        threading.Thread(target=sequence_thread).start()

def main():
    rclpy.init()
    gui_node = RobotGUI()

    root = tk.Tk()
    root.title("Robot Arm Controller")
    root.configure(bg='black')  # Black background
    root.option_add('*Label*Background', 'black')  # Set default label background
    root.option_add('*Label*Foreground', 'white')  # Set default label text color

    # Team info:
    team_title = tk.Label(
        root,
        text="Team 4",
        fg="#00FFFF",  # Cyan for title
        font=("cambria", 14, "bold"),
        justify="center"
    )
    team_title.pack(pady=(5,0))

    team_label = tk.Label(
        root,
        text="Members: Yousef Ahmed Hesham 21P0088\n               Mahmoud Eldwakhly    21P0017",
        fg="#00FFFF",  # Cyan for member names
        font=("cambria", 12, "bold"),
        justify="left",
        anchor="w"
    )
    team_label.pack(pady=5)

    # Status label 
    status_label = tk.Label(root, text="", fg="#00FF00", font=("cambria", 12, "bold"))  # Bright green
    status_label.pack(pady=10)
    gui_node.status_label = status_label

    # --- Button state management ---
    button_refs = {}

    def set_button_active(btn):
        if btn is not None:
            btn.config(bg="#4CAF50", activebackground="#4CAF50", fg="white")
            btn.update_idletasks()

    def set_button_normal(btn):
        if btn is not None:
            btn.config(bg="#333333", activebackground="#4CAF50", fg="white")  # Dark gray buttons with white text

    # --- Button command wrappers ---
    def make_motion_command(btn, func):
        def wrapped(*args, **kwargs):
            set_button_active(btn)
            def reset_btn():
                set_button_normal(btn)
            # Run the motion in a thread, reset button color after done
            def run_and_reset():
                func(*args, **kwargs)
                if btn is not None:
                    btn.after(100, reset_btn)
            threading.Thread(target=run_and_reset).start()
        return wrapped

    # --- Hideable controls ---
    controls = []

    # Full sequence button (hidden initially)
    btn_full_seq = tk.Button(
        root, text="Run Full Sequence",
        font=("cambria", 11, "bold"),
        relief="groove",
        bd=2,
        highlightthickness=0,
        bg="#333333",  # Dark gray
        fg="white",    # White text
        activebackground="#4CAF50",
        cursor="hand2",
        command=make_motion_command(None, gui_node.run_full_sequence)
    )
    controls.append(btn_full_seq)
    button_refs["full_seq"] = btn_full_seq

    # Individual position buttons (hidden initially)
    for pos in range(5):
        btn = tk.Button(
            root,
            text=f"Move to Position {pos}",
            font=("cambria", 11, "bold"),
            relief="groove",
            bd=2,
            highlightthickness=0,
            bg="#333333",  # Dark gray
            fg="white",    # White text
            activebackground="#4CAF50",
            cursor="hand2",
            command=None
        )
        # Attach command with button reference for color change
        btn.config(command=make_motion_command(btn, lambda p=pos: gui_node.move_to_position(f"position_{p}", 15)))
        controls.append(btn)
        button_refs[f"pos_{pos}"] = btn

    # Open hand button (hidden initially)
    btn_open = tk.Button(
        root, text="Open Hand",
        font=("cambria", 11, "bold"),
        relief="groove",
        bd=2,
        highlightthickness=0,
        bg="#333333",  # Dark gray
        fg="white",    # White text
        activebackground="#4CAF50",
        cursor="hand2",
        command=None
    )
    btn_open.config(command=make_motion_command(btn_open, gui_node.open_hand))
    controls.append(btn_open)
    button_refs["open"] = btn_open

    # Close hand button (hidden initially)
    btn_close = tk.Button(
        root, text="Close Hand",
        font=("cambria", 11, "bold"),
        relief="groove",
        bd=2,
        highlightthickness=0,
        bg="#333333",  # Dark gray
        fg="white",    # White text
        activebackground="#4CAF50",
        cursor="hand2",
        command=None
    )
    btn_close.config(command=make_motion_command(btn_close, gui_node.close_hand))
    controls.append(btn_close)
    button_refs["close"] = btn_close

    # --- Back button ---
    def back_to_welcome():
        for c in controls:
            c.pack_forget()
        btn_back.pack_forget()
        btn_welcome.pack(pady=15)

    btn_back = tk.Button(
        root,
        text="Back",
        font=("cambria", 11, "bold"),
        relief="raised",
        bd=2,
        bg="#660000",  # Dark red
        fg="white",    # White text
        activebackground="#ff0000",
        cursor="hand2",
        command=back_to_welcome
    )

    # --- Welcome button ---
    def show_controls():
        for c in controls:
            c.pack(pady=5)
        btn_back.pack(pady=10)
        btn_welcome.pack_forget()  # Hide the welcome button after pressed

    btn_welcome = tk.Button(
        root,
        text="Welcome to Y-M Robotic Arm",
        font=("cambria", 13, "bold"),
        relief="raised",
        bd=3,
        bg="#000066",  # Dark blue
        fg="white",    # White text
        activebackground="#0000FF",
        cursor="hand2",
        command=show_controls
    )
    btn_welcome.pack(pady=15)

    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()