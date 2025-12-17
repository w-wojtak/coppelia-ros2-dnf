#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import os
import json
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from datetime import datetime
import threading


class DNFLearningNode(Node):
    def __init__(self):
        super().__init__('dnf_learning_node')

        # ============ SIMULATION PARAMETERS ============
        self.x_lim = 30
        self.dx = 0.2
        self.dt = 0.1
        self.t_lim = 60.0  # 1 minute experiment
        
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.center_index = len(self.x) // 2

        # Cube positions in DNF space
        self.input_positions = [-20.0, 0.0, 20.0]
        self.input_indices_sm = [np.argmin(np.abs(self.x - pos)) for pos in self.input_positions]

        # ============ TIMING ============
        self.sim_time = 0.0
        self.is_finished = False
        self.last_log_time = -1  # For logging every second

        # ============ START INPUT (Detection Field) ============
        self.start_input_duration = 20
        self.start_input_amplitude = 3.0
        self.start_input_width = 2.0
        self.start_input_position = 0.0
        self.start_input_remaining = self.start_input_duration

        # ============ CUBE INPUT PARAMETERS ============
        self.input_duration_steps = 20
        self.input_amplitude = 3.0
        self.input_width = 2.0
        self.active_inputs = []

        # ============ SEQUENCE MEMORY FIELD ============
        self.h_0_sm, self.tau_h_sm, self.theta_sm = 0, 20, 1.5
        self.kernel_pars_sm = (1, 0.7, 0.9)
        self.w_hat_sm = np.fft.fft(self.kernel_osc(*self.kernel_pars_sm))
        self.u_sm = self.h_0_sm * np.ones_like(self.x)
        self.h_u_sm = self.h_0_sm * np.ones_like(self.x)

        # ============ DETECTION FIELD ============
        self.h_0_d, self.tau_h_d, self.theta_d = 0, 20, 1.5
        self.kernel_pars_d = (1, 0.7, 0.9)
        self.w_hat_d = np.fft.fft(self.kernel_osc(*self.kernel_pars_d))
        self.u_d = self.h_0_d * np.ones_like(self.x)
        self.h_u_d = self.h_0_d * np.ones_like(self.x)

        # ============ HISTORY TRACKING ============
        self.u_sm_history = []
        self.u_d_history = []
        self.u_sm_full_history = []
        self.u_d_full_history = []
        self.input_events = []

        # ============ THREAD SAFETY ============
        self.data_lock = threading.Lock()
        self.plot_needs_update = False

        # ============ PLOTTING ============
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(8, 4))
        self.line1, = self.ax1.plot(self.x, self.u_sm, 'b-', label='u_sm', linewidth=2)
        self.line2, = self.ax2.plot(self.x, self.u_d, 'r-', label='u_d', linewidth=2)
        self.setup_axes()
        self.fig.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)

        # ============ ROS2 SUBSCRIBER ============
        self.create_subscription(Float32, '/dnf_inputs', self.input_callback, 10)

        # ============ TIMER ============
        self.update_timer = self.create_timer(self.dt, self.update_callback)

        self.get_logger().info("=" * 50)
        self.get_logger().info("DNF Learning Node initialized")
        self.get_logger().info(f"  Experiment duration: {self.t_lim:.0f} seconds")
        self.get_logger().info(f"  dt = {self.dt}s")
        self.get_logger().info(f"  Input positions: {self.input_positions}")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Waiting for cube inputs...")

    # ========================================
    # KERNEL AND GAUSSIAN
    # ========================================
    def kernel_osc(self, a, b, alpha):
        return a * (np.exp(-b * np.abs(self.x)) * 
                   (b * np.sin(np.abs(alpha * self.x)) + np.cos(alpha * self.x)))

    def gaussian(self, center=0, amplitude=1.0, width=1.0):
        return amplitude * np.exp(-((self.x - center) ** 2) / (2 * width ** 2))

    # ========================================
    # INPUT CALLBACK
    # ========================================
    def input_callback(self, msg: Float32):
        with self.data_lock:
            pos = msg.data
            self.active_inputs.append((pos, self.input_duration_steps))
            self.input_events.append((self.sim_time, pos))
            
            # Find cube name
            cube_name = "Unknown"
            if pos == -20.0:
                cube_name = "Cuboid1"
            elif pos == 0.0:
                cube_name = "Cuboid2"
            elif pos == 20.0:
                cube_name = "Cuboid3"
            
            self.get_logger().info(
                f"[{self.sim_time:5.1f}s / {self.t_lim:.0f}s] 📦 INPUT: {cube_name} (x={pos:.1f})"
            )

    # ========================================
    # UPDATE CALLBACK (TIMER)
    # ========================================
    def update_callback(self):
        if self.is_finished:
            return

        self.update_fields()
        self.update_plot()

        try:
            plt.pause(0.001)
        except:
            pass

    # ========================================
    # FIELD UPDATE
    # ========================================
    def update_fields(self):
        with self.data_lock:
            # ===== START INPUT (Detection Field) =====
            start_input = np.zeros_like(self.x)
            if self.start_input_remaining > 0:
                start_input = self.gaussian(
                    center=self.start_input_position,
                    amplitude=self.start_input_amplitude,
                    width=self.start_input_width
                )
                self.start_input_remaining -= 1
                
                if self.start_input_remaining == self.start_input_duration - 1:
                    self.get_logger().info(
                        f"[{self.sim_time:5.1f}s / {self.t_lim:.0f}s] 🚀 Start input activated"
                    )
                elif self.start_input_remaining == 0:
                    self.get_logger().info(
                        f"[{self.sim_time:5.1f}s / {self.t_lim:.0f}s] ✓ Start input finished"
                    )

            # ===== DETECTION FIELD =====
            f_d = np.heaviside(self.u_d - self.theta_d, 1)
            conv_d = self.dx * np.fft.ifftshift(
                np.real(np.fft.ifft(np.fft.fft(f_d) * self.w_hat_d))
            )
            self.h_u_d += self.dt / self.tau_h_d * f_d
            self.u_d += self.dt * (-self.u_d + conv_d + start_input + self.h_u_d)

            # ===== CUBE INPUTS (Sequence Memory Field) =====
            input_field = np.zeros_like(self.x)
            new_active = []
            for pos, remaining in self.active_inputs:
                input_field += self.gaussian(pos, amplitude=self.input_amplitude, width=self.input_width)
                if remaining > 1:
                    new_active.append((pos, remaining - 1))
            self.active_inputs = new_active

            # ===== SEQUENCE MEMORY FIELD =====
            f_sm = np.heaviside(self.u_sm - self.theta_sm, 1)
            conv_sm = self.dx * np.fft.ifftshift(
                np.real(np.fft.ifft(np.fft.fft(f_sm) * self.w_hat_sm))
            )
            self.h_u_sm += self.dt / self.tau_h_sm * f_sm
            self.u_sm += self.dt * (-self.u_sm + conv_sm + input_field + self.h_u_sm)

            # ===== HISTORY TRACKING =====
            u_sm_values = [self.u_sm[idx] for idx in self.input_indices_sm]
            u_d_value = self.u_d[self.center_index]
            self.u_sm_history.append(u_sm_values)
            self.u_d_history.append(u_d_value)
            self.u_sm_full_history.append(self.u_sm.copy())
            self.u_d_full_history.append(self.u_d.copy())

            self.plot_needs_update = True

            # ===== TIME UPDATE =====
            self.sim_time += self.dt

            # ===== LOG EVERY SECOND =====
            current_second = int(self.sim_time)
            if current_second > self.last_log_time:
                self.last_log_time = current_second
                remaining = self.t_lim - self.sim_time
                progress = (self.sim_time / self.t_lim) * 100
                
                # Progress bar
                bar_length = 20
                filled = int(bar_length * self.sim_time / self.t_lim)
                bar = '█' * filled + '░' * (bar_length - filled)
                
                self.get_logger().info(
                    f"[{self.sim_time:5.1f}s / {self.t_lim:.0f}s] [{bar}] {progress:5.1f}% | "
                    f"Events: {len(self.input_events)} | "
                    f"u_sm_max: {self.u_sm.max():.2f}"
                )

            # ===== CHECK IF FINISHED =====
            if self.sim_time >= self.t_lim:
                self.is_finished = True
                self.get_logger().info("=" * 50)
                self.get_logger().info(f"✅ EXPERIMENT COMPLETE at t={self.sim_time:.1f}s")
                self.get_logger().info(f"   Total input events: {len(self.input_events)}")
                self.get_logger().info("=" * 50)

    # ========================================
    # PLOTTING
    # ========================================
    def setup_axes(self):
        object_positions = [-20, 0, 20]
        object_labels = ['Cube1', 'Cube2', 'Cube3']
        start_position = [0]  # for duration field x-axis label

        for ax in [self.ax1, self.ax2]:
            ax.grid(True, alpha=0.3)
            ax.tick_params(axis='both', which='major', labelsize=12)  # increase font size

        # Sequence Memory Field (ax1)
        self.ax1.set_xlim(-self.x_lim, self.x_lim)
        self.ax1.set_ylim(-2, 6)
        self.ax1.set_xlabel("Objects", fontsize=14)
        self.ax1.set_ylabel("u_sm", fontsize=14)
        self.ax1.axhline(y=self.theta_sm, color='r', linestyle='--', 
                label=f'θ={self.theta_sm}', linewidth=2)
        self.ax1.set_xticks(object_positions)
        self.ax1.set_xticklabels(object_labels, rotation=45, ha='right', fontsize=12)
        self.ax1.set_title("Sequence Memory Field", fontsize=16)
        # for pos in object_positions:
        #     self.ax1.axvline(x=pos, color='gray', linestyle='--', alpha=0.3, linewidth=0.5)
        self.ax1.legend(loc='upper right')

        # Task Duration Field (ax2)
        self.ax2.set_xlim(-self.x_lim, self.x_lim)  
        self.ax2.set_ylim(-2, 6)
        self.ax2.set_xlabel("", fontsize=14)
        self.ax2.set_ylabel("u_d", fontsize=14)
        self.ax2.axhline(y=self.theta_d, color='r', linestyle='--', 
                label=f'θ={self.theta_d}', linewidth=2)
        self.ax2.set_xticks(start_position)
        self.ax2.set_xticklabels(['start'], fontsize=12)
        self.ax2.set_title("Task Duration Field", fontsize=16)
        self.ax2.legend(loc='upper right')


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
            self.ax1.set_title(f"Sequence Memory Field (t={sim_time_copy:.1f}s / {self.t_lim:.0f}s)")
            self.ax1.relim()
            self.ax1.autoscale_view(scalex=False, scaley=True)
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=False, scaley=True)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception as e:
            self.get_logger().warn(f"Plot update failed: {e}")

    # ========================================
    # DATA SAVING
    # ========================================
    def save_data(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("SAVING DATA...")
        
        try:
            data_dir = "learned_data"
            os.makedirs(data_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            with self.data_lock:
                # ===== SAVE FINAL FIELD STATES =====
                np.save(os.path.join(data_dir, f"u_sm_{timestamp}.npy"), self.u_sm)
                np.save(os.path.join(data_dir, f"u_d_{timestamp}.npy"), self.u_d)
                np.save(os.path.join(data_dir, f"h_u_sm_{timestamp}.npy"), self.h_u_sm)
                np.save(os.path.join(data_dir, f"h_u_d_{timestamp}.npy"), self.h_u_d)

                # ===== SAVE LEARNED SEQUENCE (for recall) =====
                sequence_data = {
                    'events': self.input_events,
                    'dt': self.dt,
                    't_lim': self.t_lim,
                    'input_positions': self.input_positions,
                    'cube_mapping': {
                        'Cuboid1': -20.0,
                        'Cuboid2': 0.0,
                        'Cuboid3': 20.0
                    }
                }
                
                sequence_path = os.path.join(data_dir, "learned_sequence.json")
                with open(sequence_path, 'w') as f:
                    json.dump(sequence_data, f, indent=2)
                
                self.get_logger().info(f"Learned sequence saved:")
                for i, (t, pos) in enumerate(self.input_events):
                    cube = "Cuboid1" if pos == -20 else ("Cuboid2" if pos == 0 else "Cuboid3")
                    self.get_logger().info(f"  {i+1}. t={t:.1f}s → {cube} (x={pos})")

                # ===== COMPUTE CROSSING TIMES =====
                u_sm_history = np.array(self.u_sm_history)
                u_d_history = np.array(self.u_d_history)
                
                self.get_logger().info("-" * 50)
                self.get_logger().info("Threshold Crossing Times:")
                
                for i, pos in enumerate(self.input_positions):
                    above = u_sm_history[:, i] >= self.theta_sm
                    if np.any(above):
                        crossing_idx = np.argmax(above)
                        crossing_time = crossing_idx * self.dt
                        self.get_logger().info(
                            f"  u_sm at x={pos:+.0f} crosses theta at t={crossing_time:.2f}s"
                        )

                above = u_d_history >= self.theta_d
                if np.any(above):
                    crossing_idx = np.argmax(above)
                    crossing_time = crossing_idx * self.dt
                    self.get_logger().info(
                        f"  u_d at x=0 crosses theta at t={crossing_time:.2f}s"
                    )

                # ===== PLOT TIME COURSES =====
                timesteps = np.arange(len(u_sm_history)) * self.dt

                fig, ax = plt.subplots(figsize=(8, 3))
                cube_labels = ['Cube 1', 'Cube 2', 'Cube 3']

                for i, pos in enumerate(self.input_positions):
                    ax.plot(timesteps, u_sm_history[:, i], label=cube_labels[i], linewidth=2)

                ax.axhline(self.theta_sm, color='r', linestyle='--', label=f'θ = {self.theta_sm}')
                ax.set_ylabel('u_sm')
                ax.set_xlabel('Time [s]')
                ax.set_ylim(-1, 5)
                ax.legend(loc='upper left')
                ax.grid(True, alpha=0.3)
                ax.set_title('Sequence Memory Field Over Time')

                timecourse_path = os.path.join(data_dir, f"timecourse_{timestamp}.png")
                fig.savefig(timecourse_path, dpi=300, bbox_inches='tight')
                plt.close(fig)
                # timesteps = np.arange(len(u_sm_history)) * self.dt
                
                # fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
                # cube_labels = ['Cube 1', 'Cube 2', 'Cube 3']

                # for i, pos in enumerate(self.input_positions):
                #     axs[0].plot(timesteps, u_sm_history[:, i], label=cube_labels[i], linewidth=2)
                # axs[0].axhline(self.theta_sm, color='r', linestyle='--', 
                #               label=f'θ = {self.theta_sm}')
                # axs[0].set_ylabel('u_sm')
                # axs[0].set_ylim(-1, 5)
                # axs[0].legend()
                # axs[0].grid(True, alpha=0.3)
                # axs[0].set_title('Sequence Memory Field Over Time')

                # # for t, pos in self.input_events:
                # #     axs[0].axvline(x=t, color='green', linestyle=':', alpha=0.5)

                # axs[1].plot(timesteps, u_d_history, label='x = 0', linewidth=2, color='red')
                # axs[1].axhline(self.theta_d, color='r', linestyle='--', 
                #               label=f'θ = {self.theta_d}')
                # axs[1].set_ylabel('u_d')
                # axs[1].set_xlabel('Time [s]')
                # axs[1].set_ylim(0, 5)
                # axs[1].legend()
                # axs[1].grid(True, alpha=0.3)
                # axs[1].set_title('Duration Field Over Time')

                # timecourse_path = os.path.join(data_dir, f"timecourse_{timestamp}.png")
                # fig.savefig(timecourse_path, dpi=150, bbox_inches='tight')
                # plt.close(fig)

                final_plot_path = os.path.join(data_dir, f"final_plot_{timestamp}.png")
                self.fig.savefig(final_plot_path, dpi=150, bbox_inches='tight')

                np.save(os.path.join(data_dir, f"u_sm_history_{timestamp}.npy"), 
                       np.array(self.u_sm_full_history))
                np.save(os.path.join(data_dir, f"u_d_history_{timestamp}.npy"), 
                       np.array(self.u_d_full_history))

            self.get_logger().info("-" * 50)
            self.get_logger().info(f"All data saved in: {os.path.abspath(data_dir)}/")
            self.get_logger().info("=" * 50)
            self.get_logger().info("🎉 LEARNING COMPLETE - Ready for recall phase!")
            self.get_logger().info("=" * 50)

        except Exception as e:
            self.get_logger().error(f"Error saving data: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = DNFLearningNode()

    try:
        while rclpy.ok() and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.01)

        if node.is_finished:
            node.save_data()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user - saving data...")
        node.save_data()
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()