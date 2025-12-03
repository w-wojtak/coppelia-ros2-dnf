#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from datetime import datetime
import threading
import os

class DNFLearningNode(Node):
    def __init__(self):
        super().__init__('dnf_learning_node')

        # Simulation parameters - FIXED DURATION
        self.x_lim = 30
        self.dx = 0.2
        self.dt = 0.1           # Time step (seconds)
        self.t_lim = 120.0      # Total experiment duration (seconds) - increased!
        
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.center_index = len(self.x) // 2

        # Current simulation time
        self.sim_time = 0.0
        self.is_finished = False

        # Active cube inputs: list of (pos, remaining_timesteps)
        self.active_inputs = []

        # Sequence memory field
        self.h_0_sm, self.tau_h_sm, self.theta_sm = 0, 20, 1.5
        self.kernel_pars_sm = (1, 0.7, 0.9)
        self.w_hat_sm = np.fft.fft(self.kernel_osc(*self.kernel_pars_sm))
        self.u_sm = self.h_0_sm * np.ones_like(self.x)
        self.h_u_sm = self.h_0_sm * np.ones_like(self.x)

        # Detection field
        self.h_0_d, self.tau_h_d, self.theta_d = 0, 20, 1.5
        self.kernel_pars_d = (1, 0.7, 0.9)
        self.w_hat_d = np.fft.fft(self.kernel_osc(*self.kernel_pars_d))
        self.u_d = self.h_0_d * np.ones_like(self.x)
        self.h_u_d = self.h_0_d * np.ones_like(self.x)

        # Histories
        self.u_sm_history = []
        self.u_d_history = []
        self.input_events = []  # Track when inputs arrived

        # ============ START INPUT PARAMETERS ============
        self.start_input_duration = 20      # timesteps (2 seconds with dt=0.1)
        self.start_input_amplitude = 3.0    # Gaussian amplitude
        self.start_input_width = 2.0        # Gaussian width
        self.start_input_position = 0.0     # Center (x=0)
        self.start_input_remaining = self.start_input_duration  # Countdown
        # ================================================

        # ============ CUBE INPUT PARAMETERS (Sequence Memory) ============
        self.input_duration_steps = 20      # 2 seconds
        self.input_amplitude = 3.0
        self.input_width = 2.0
        # =================================================================

        # Thread lock for safety
        self.data_lock = threading.Lock()
        self.plot_needs_update = False

        # Initialize plot
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
        self.line1, = self.ax1.plot(self.x, self.u_sm, 'b-', label='u_sm', linewidth=2)
        self.line2, = self.ax2.plot(self.x, self.u_d, 'r-', label='u_d', linewidth=2)
        self.setup_axes()
        self.fig.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)

        # ROS2 subscriber
        self.create_subscription(Float32, '/dnf_inputs', self.input_callback, 10)

        # Timer at 1/dt Hz = 10 Hz for dt=0.1
        # This makes simulation time match real time!
        timer_period = self.dt  # 0.1 seconds
        self.update_timer = self.create_timer(timer_period, self.update_callback)

        self.get_logger().info(f"DNF Learning Node initialized")
        self.get_logger().info(f"  - dt = {self.dt}s, t_lim = {self.t_lim}s")
        self.get_logger().info(f"  - Timer rate = {1/timer_period:.1f} Hz")
        self.get_logger().info(f"  - Experiment will run for {self.t_lim} real seconds")

    # ----------------------------
    # Receive cube input
    # ----------------------------
    def input_callback(self, msg: Float32):
        with self.data_lock:
            pos = msg.data
            n_timesteps = 20  # Gaussian persists for 10 steps
            self.active_inputs.append((pos, n_timesteps))
            self.input_events.append((self.sim_time, pos))
            self.get_logger().info(
                f"[t={self.sim_time:.1f}s] Received input at x={pos:.1f}"
            )

    # ----------------------------
    # Timer callback - runs at 1/dt Hz
    # ----------------------------
    def update_callback(self):
        if self.is_finished:
            return

        self.update_fields()
        self.update_plot()
        
        # Keep matplotlib responsive
        try:
            plt.pause(0.001)
        except:
            pass

    # ----------------------------
    # Update sequence memory & detection
    # ----------------------------
    def update_fields(self):
        with self.data_lock:
            # ========== DETECTION FIELD ==========
            
            # Start input: Gaussian at x=0 for first N timesteps
            start_input = np.zeros_like(self.x)
            if self.start_input_remaining > 0:
                start_input = self.gaussian(
                    center=self.start_input_position,
                    amplitude=self.start_input_amplitude,
                    width=self.start_input_width
                )
                self.start_input_remaining -= 1
                
                # Log when start input begins and ends
                if self.start_input_remaining == self.start_input_duration - 1:
                    self.get_logger().info(f"[t={self.sim_time:.1f}s] 🚀 Start input activated at x=0")
                elif self.start_input_remaining == 0:
                    self.get_logger().info(f"[t={self.sim_time:.1f}s] ✓ Start input finished")

            f_d = np.heaviside(self.u_d - self.theta_d, 1)
            conv_d = self.dx * np.fft.ifftshift(
                np.real(np.fft.ifft(np.fft.fft(f_d) * self.w_hat_d))
            )
            self.h_u_d += self.dt / self.tau_h_d * f_d
            self.u_d += self.dt * (-self.u_d + conv_d + start_input + self.h_u_d)  # Added start_input!

            # ========== SEQUENCE MEMORY FIELD ==========
            
            input_field = np.zeros_like(self.x)
            for i in range(len(self.active_inputs) - 1, -1, -1):
                pos, remaining = self.active_inputs[i]
                input_field += self.gaussian(pos, amplitude=self.input_amplitude, width=self.input_width)
                remaining -= 1
                if remaining <= 0:
                    self.active_inputs.pop(i)
                else:
                    self.active_inputs[i] = (pos, remaining)

            f_sm = np.heaviside(self.u_sm - self.theta_sm, 1)
            conv_sm = self.dx * np.fft.ifftshift(
                np.real(np.fft.ifft(np.fft.fft(f_sm) * self.w_hat_sm))
            )
            self.h_u_sm += self.dt / self.tau_h_sm * f_sm
            self.u_sm += self.dt * (-self.u_sm + conv_sm + input_field + self.h_u_sm)

            # ========== HISTORY & LOGGING ==========
            
            self.u_sm_history.append(self.u_sm.copy())
            self.u_d_history.append(self.u_d[self.center_index])
            self.plot_needs_update = True

            self.sim_time += self.dt

            if abs(self.sim_time % 10.0) < self.dt:
                self.get_logger().info(
                    f"[t={self.sim_time:.1f}s] u_sm: [{self.u_sm.min():.2f}, {self.u_sm.max():.2f}] | "
                    f"u_d: [{self.u_d.min():.2f}, {self.u_d.max():.2f}] | "
                    f"Active inputs: {len(self.active_inputs)}"
                )

            if self.sim_time >= self.t_lim:
                self.is_finished = True
                self.get_logger().info(f"Experiment finished at t={self.sim_time:.1f}s")

    # ----------------------------
    # Gaussian input
    # ----------------------------
    def gaussian(self, center=0, amplitude=1.0, width=1.0):
        return amplitude * np.exp(-((self.x - center) ** 2) / (2 * width ** 2))

    # ----------------------------
    # Kernel function
    # ----------------------------
    def kernel_osc(self, a, b, alpha):
        return a * (
            np.exp(-b * np.abs(self.x)) * 
            (b * np.sin(np.abs(alpha * self.x)) + np.cos(alpha * self.x))
        )

    # ----------------------------
    # Plot
    # ----------------------------
    def update_plot(self):
        if not self.plot_needs_update:
            return
        try:
            with self.data_lock:
                u_sm_copy = self.u_sm.copy()
                u_d_copy = self.u_d.copy()
                sim_time_copy = self.sim_time
                self.plot_needs_update = False

            self.line1.set_ydata(u_sm_copy)
            self.line2.set_ydata(u_d_copy)
            self.ax1.set_title(f"Sequence Memory Field (t={sim_time_copy:.1f}s)")
            self.ax1.relim()
            self.ax1.autoscale_view(scalex=False, scaley=True)
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=False, scaley=True)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception as e:
            self.get_logger().warn(f"Plot update failed: {e}")

    def setup_axes(self):
        object_all = [-20, 0, 20]
        object_labels = ['cube1', 'cube2', 'cube3']

        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(-self.x_lim, self.x_lim)
            ax.set_ylim(-2, 6)
            ax.set_xlabel("Objects")
            ax.set_ylabel("u(x)")
            ax.grid(True, alpha=0.3)
            ax.set_xticks(object_all)
            ax.set_xticklabels(object_labels, rotation=45, ha='right')
            for pos in object_all:
                ax.axvline(x=pos, color='gray', linestyle='--', alpha=0.3, linewidth=0.5)

        self.ax1.set_title("Sequence Memory Field")
        self.ax2.set_title("Task Duration Field")

    # ----------------------------
    # Data saving
    # ----------------------------
    def save_data(self):
        self.get_logger().info("Saving data...")
        try:
            data_dir = "data_basic"
            os.makedirs(data_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            # Save field states
            np.save(f"{data_dir}/u_sm_{timestamp}.npy", self.u_sm)
            np.save(f"{data_dir}/u_d_{timestamp}.npy", self.u_d)

            # Save input events
            if self.input_events:
                np.save(f"{data_dir}/input_events_{timestamp}.npy", np.array(self.input_events))
                self.get_logger().info(f"Recorded {len(self.input_events)} input events")

            # Histories
            u_sm_hist = np.array(self.u_sm_history)
            u_d_hist = np.array(self.u_d_history)
            time_axis = np.arange(len(u_sm_hist)) * self.dt

            fig, axs = plt.subplots(2, 1, figsize=(12, 8))

            # Plot u_sm at specific positions
            positions = [-20, 0, 20]
            for pos in positions:
                idx = np.argmin(np.abs(self.x - pos))
                axs[0].plot(time_axis, u_sm_hist[:, idx], label=f'x={pos}')

            # Mark input events
            for t, pos in self.input_events:
                axs[0].axvline(x=t, color='green', linestyle=':', alpha=0.5)

            axs[0].axhline(self.theta_sm, color='r', linestyle='--', label='theta')
            axs[0].set_ylabel('u_sm')
            axs[0].set_ylim(-1, 5)
            axs[0].grid(True)
            axs[0].legend()
            axs[0].set_title('Sequence Memory Field over Time')

            axs[1].plot(time_axis, u_d_hist, label='x=0')
            axs[1].axhline(self.theta_d, color='r', linestyle='--', label='theta')
            axs[1].set_ylabel('u_d')
            axs[1].set_xlabel('Time [s]')
            axs[1].set_ylim(0, 5)
            axs[1].grid(True)
            axs[1].legend()

            fig.savefig(f"{data_dir}/timecourse_{timestamp}.png", dpi=150, bbox_inches='tight')
            plt.close(fig)
            self.fig.savefig(f"{data_dir}/final_plot_{timestamp}.png", dpi=150, bbox_inches='tight')
            self.get_logger().info(f"Data and plots saved in {data_dir}")
        except Exception as e:
            self.get_logger().error(f"Error saving data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DNFLearningNode()

    try:
        # Spin until finished or interrupted
        while rclpy.ok() and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.01)

        # Save data when done
        if node.is_finished:
            node.save_data()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.save_data()
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()