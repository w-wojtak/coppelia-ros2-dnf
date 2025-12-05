#!/usr/bin/env python3
"""
DNF Recall Node - ROS2
Loads saved sequence memory from learning phase and performs recall
to predict the learned sequence of cube manipulations.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import os
import json
import glob
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from std_msgs.msg import String
from datetime import datetime
import threading


class DNFRecallNode(Node):
    def __init__(self):
        super().__init__('dnf_recall_node')

        # ============ PARAMETERS ============
        self.declare_parameter('data_path', 'learned_data')
        self.data_path = self.get_parameter('data_path').value

        # ============ SIMULATION PARAMETERS (same as learning node) ============
        self.x_lim = 30
        self.dx = 0.2
        self.dt = 0.1
        self.t_lim = 60.0  # Recall duration
        
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.field_size = len(self.x)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("DNF Recall Node initialized")
        self.get_logger().info(f"  Data path: {self.data_path}")
        self.get_logger().info(f"  Field size: {self.field_size}")
        self.get_logger().info("=" * 50)

        # ============ OBJECT MAPPING (3 cubes) ============
        self.input_positions = [-20.0, 0.0, 20.0]
        self.object_labels = ['Cuboid1', 'Cuboid2', 'Cuboid3']
        self.object_map = dict(zip(self.input_positions, self.object_labels))
        self.input_indices = [np.argmin(np.abs(self.x - pos)) for pos in self.input_positions]

        # ============ TIMING ============
        self.sim_time = 0.0
        self.is_finished = False
        self.last_log_time = -1

        # ============ LOAD SAVED DATA ============
        self.h_d_initial = 0.0
        self._load_data()

        # ============ INITIALIZE FIELDS ============
        self._init_fields()

        # ============ HISTORY TRACKING ============
        self.u_act_history = []
        self.u_wm_history = []
        self.time_history = []

        # ============ THRESHOLD TRACKING ============
        self.threshold_crossed = {pos: False for pos in self.input_positions}
        self.crossing_times = {}
        self.predicted_sequence = []

        # ============ THREAD SAFETY ============
        self.data_lock = threading.Lock()
        self.plot_needs_update = False

        # ============ PUBLISHER ============
        self.prediction_pub = self.create_publisher(String, 'dnf_predictions', 10)

        # ============ PLOTTING ============
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 5))
        self._setup_plots()
        plt.show(block=False)
        plt.pause(0.1)

        # ============ TIMER ============
        self.update_timer = self.create_timer(self.dt, self.update_callback)

        self.get_logger().info("Starting recall...")

    # ========================================
    # KERNEL FUNCTION (same as learning node)
    # ========================================
    def kernel_osc(self, a, b, alpha):
        """Oscillatory kernel for lateral interactions."""
        return a * (np.exp(-b * np.abs(self.x)) * 
                   (b * np.sin(np.abs(alpha * self.x)) + np.cos(alpha * self.x)))

    # ========================================
    # LOAD DATA
    # ========================================
    def _load_data(self):
        """Load sequence memory and task duration from saved files."""
        self.get_logger().info("-" * 50)
        self.get_logger().info("Loading saved data...")
        
        try:
            # Find files - EXCLUDE history files
            all_u_sm = glob.glob(os.path.join(self.data_path, "u_sm_*.npy"))
            u_sm_files = sorted([f for f in all_u_sm if 'history' not in f])
            
            all_u_d = glob.glob(os.path.join(self.data_path, "u_d_*.npy"))
            u_d_files = sorted([f for f in all_u_d if 'history' not in f])
            
            all_h_u_sm = glob.glob(os.path.join(self.data_path, "h_u_sm_*.npy"))
            h_u_sm_files = sorted([f for f in all_h_u_sm if 'history' not in f])
            
            if not u_sm_files:
                self.get_logger().error(f"No u_sm files found in {self.data_path}")
                self.u_sm = np.zeros_like(self.x)
                self.h_u_sm_loaded = np.zeros_like(self.x)
                self.u_d = np.zeros_like(self.x)
                return
            
            # Load most recent files (final field states, not history)
            u_sm_path = u_sm_files[-1]
            self.u_sm = np.load(u_sm_path)
            self.get_logger().info(f"✓ Loaded u_sm from: {u_sm_path}")
            self.get_logger().info(f"  Shape: {self.u_sm.shape}, Max: {np.max(self.u_sm):.3f}")
            
            if h_u_sm_files:
                h_u_sm_path = h_u_sm_files[-1]
                self.h_u_sm_loaded = np.load(h_u_sm_path)
                self.get_logger().info(f"✓ Loaded h_u_sm from: {h_u_sm_path}")
            else:
                self.h_u_sm_loaded = np.zeros_like(self.x)
            
            if u_d_files:
                u_d_path = u_d_files[-1]
                self.u_d = np.load(u_d_path)
                self.h_d_initial = np.max(self.u_d)
                self.get_logger().info(f"✓ Loaded u_d from: {u_d_path}")
                self.get_logger().info(f"  Max (h_d_initial): {self.h_d_initial:.3f}")
            else:
                self.u_d = np.zeros_like(self.x)
                self.h_d_initial = np.max(self.h_u_sm_loaded) if np.max(self.h_u_sm_loaded) > 0 else 3.0
                self.get_logger().warn(f"No u_d found, using h_d_initial = {self.h_d_initial:.3f}")
            
            # Try to load learned sequence for reference
            sequence_path = os.path.join(self.data_path, "learned_sequence.json")
            if os.path.exists(sequence_path):
                with open(sequence_path, 'r') as f:
                    self.learned_sequence = json.load(f)
                self.get_logger().info(f"✓ Loaded learned sequence:")
                for i, (t, pos) in enumerate(self.learned_sequence['events']):
                    label = self.object_map.get(pos, f"x={pos}")
                    self.get_logger().info(f"    {i+1}. t={t:.1f}s → {label}")
            else:
                self.learned_sequence = None
                self.get_logger().warn("No learned_sequence.json found")
                
            # Log values at input positions
            self.get_logger().info("-" * 50)
            self.get_logger().info("Loaded field values at cube positions:")
            for pos, label in self.object_map.items():
                idx = np.argmin(np.abs(self.x - pos))
                self.get_logger().info(
                    f"  {label} (x={pos:+.0f}): u_sm={self.u_sm[idx]:.3f}, "
                    f"h_u_sm={self.h_u_sm_loaded[idx]:.3f}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error loading data: {e}")
            import traceback
            traceback.print_exc()
            self.u_sm = np.zeros_like(self.x)
            self.h_u_sm_loaded = np.zeros_like(self.x)
            self.u_d = np.zeros_like(self.x)

    # ========================================
    # INITIALIZE FIELDS
    # ========================================
    def _init_fields(self):
        """Initialize recall fields."""
        # Thresholds
        self.theta_act = 1.5
        self.theta_wm = 0.8

        # Time constants
        self.tau_h_act = 20.0

        # Kernels (precompute FFT)
        self.kernel_pars = (1, 0.7, 0.9)
        self.w_hat_act = np.fft.fft(self.kernel_osc(*self.kernel_pars))
        self.w_hat_wm = np.fft.fft(self.kernel_osc(1.75, 0.5, 0.8))

        # Action onset field initialization
        self.offset = 1.5
        self.h_u_act = -self.h_d_initial * np.ones_like(self.x) + self.offset
        
        # Use loaded sequence memory as the input pattern
        self.input_action_onset = self.u_sm.copy()
        
        # Initialize u_act
        self.u_act = self.input_action_onset + self.h_u_act
        
        # Working memory field
        self.u_wm = -1.0 * np.ones_like(self.x)
        self.h_u_wm = -1.0 * np.ones_like(self.x)

        self.get_logger().info("-" * 50)
        self.get_logger().info("Fields initialized:")
        self.get_logger().info(f"  h_d_initial: {self.h_d_initial:.3f}")
        self.get_logger().info(f"  tau_h_act: {self.tau_h_act:.1f}")
        self.get_logger().info("Initial u_act at cube positions:")
        for pos, label in self.object_map.items():
            idx = np.argmin(np.abs(self.x - pos))
            self.get_logger().info(f"    {label}: u_act={self.u_act[idx]:.3f}")

    # ========================================
    # SETUP PLOTS
    # ========================================
    def _setup_plots(self):
        """Setup matplotlib plots."""
        object_positions = [-20, 0, 20]
        object_labels = ['Cube1', 'Cube2', 'Cube3']

        # Plot 1: Action Onset Field
        self.ax1.set_xlim(-self.x_lim, self.x_lim)
        self.ax1.set_ylim(-6, 3)
        self.ax1.set_xlabel("Position")
        self.ax1.set_ylabel("u_act(x)")
        self.ax1.set_title("Action Onset Field")
        self.ax1.axhline(y=self.theta_act, color='r', linestyle='--', 
                        label=f'θ={self.theta_act}', linewidth=2)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_xticks(object_positions)
        self.ax1.set_xticklabels(object_labels)
        for pos in object_positions:
            self.ax1.axvline(x=pos, color='gray', linestyle=':', alpha=0.5)
        self.line_act, = self.ax1.plot(self.x, self.u_act, 'b-', linewidth=2, label='u_act')
        self.ax1.legend(loc='upper right')

        # Plot 2: Working Memory Field
        self.ax2.set_xlim(-self.x_lim, self.x_lim)
        self.ax2.set_ylim(-2, 10)
        self.ax2.set_xlabel("Position")
        self.ax2.set_ylabel("u_wm(x)")
        self.ax2.set_title("Working Memory Field")
        self.ax2.axhline(y=self.theta_wm, color='r', linestyle='--', 
                        label=f'θ={self.theta_wm}', linewidth=2)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_xticks(object_positions)
        self.ax2.set_xticklabels(object_labels)
        for pos in object_positions:
            self.ax2.axvline(x=pos, color='gray', linestyle=':', alpha=0.5)
        self.line_wm, = self.ax2.plot(self.x, self.u_wm, 'g-', linewidth=2, label='u_wm')
        self.ax2.legend(loc='upper right')

        self.fig.tight_layout()

    # ========================================
    # UPDATE CALLBACK
    # ========================================
    def update_callback(self):
        """Main update loop."""
        if self.is_finished:
            return

        self.perform_recall_step()
        self.update_plot()

        try:
            plt.pause(0.001)
        except:
            pass

    # ========================================
    # RECALL DYNAMICS
    # ========================================
    def perform_recall_step(self):
        """Perform one step of recall dynamics."""
        with self.data_lock:
            # Convolution helper
            def conv(field, w_hat, theta):
                f = np.heaviside(field - theta, 1)
                return self.dx * np.fft.ifftshift(
                    np.real(np.fft.ifft(np.fft.fft(f) * w_hat))
                )

            conv_act = conv(self.u_act, self.w_hat_act, self.theta_act)
            conv_wm = conv(self.u_wm, self.w_hat_wm, self.theta_wm)
            f_wm = np.heaviside(self.u_wm - self.theta_wm, 1)

            # Update h_u_act (rises with time)
            self.h_u_act += self.dt / self.tau_h_act

            # Update action onset field
            self.u_act += self.dt * (
                -self.u_act 
                + conv_act 
                + self.input_action_onset 
                + self.h_u_act 
                - 6.0 * f_wm * conv_wm
            )

            # Update working memory field
            self.u_wm += self.dt * (-self.u_wm + conv_wm + self.h_u_wm)

            # Record history
            self.time_history.append(self.sim_time)
            self.u_act_history.append([self.u_act[idx] for idx in self.input_indices])
            self.u_wm_history.append([self.u_wm[idx] for idx in self.input_indices])

            # Check threshold crossings
            for i, pos in enumerate(self.input_positions):
                idx = self.input_indices[i]
                if not self.threshold_crossed[pos] and self.u_act[idx] > self.theta_act:
                    self.threshold_crossed[pos] = True
                    self.crossing_times[pos] = self.sim_time
                    
                    label = self.object_map[pos]
                    self.predicted_sequence.append((self.sim_time, label, pos))
                    
                    self.get_logger().info(
                        f"[{self.sim_time:5.1f}s] ⭐ PREDICTION: {label} "
                        f"(x={pos:+.0f}, u_act={self.u_act[idx]:.2f})"
                    )
                    
                    # Publish prediction
                    msg = String()
                    msg.data = label
                    self.prediction_pub.publish(msg)
                    
                    # Activate working memory at this position
                    self.h_u_wm[max(0, idx-10):min(self.field_size, idx+11)] = 3.0

            self.plot_needs_update = True
            self.sim_time += self.dt

            # Log progress every 5 seconds
            current_second = int(self.sim_time)
            if current_second > self.last_log_time and current_second % 5 == 0:
                self.last_log_time = current_second
                n_predicted = len(self.predicted_sequence)
                self.get_logger().info(
                    f"[{self.sim_time:5.1f}s / {self.t_lim:.0f}s] "
                    f"Predicted: {n_predicted}/3"
                )

            # Check if finished
            if self.sim_time >= self.t_lim: #or all(self.threshold_crossed.values()):
                self.is_finished = True
                self.get_logger().info("=" * 50)
                self.get_logger().info(f"✅ RECALL COMPLETE at t={self.sim_time:.1f}s")
                self.get_logger().info("=" * 50)

    # ========================================
    # PLOTTING
    # ========================================
    def update_plot(self):
        """Update the plots."""
        if not self.plot_needs_update:
            return
            
        try:
            with self.data_lock:
                u_act_copy = self.u_act.copy()
                u_wm_copy = self.u_wm.copy()
                sim_time_copy = self.sim_time
                self.plot_needs_update = False

            self.line_act.set_ydata(u_act_copy)
            self.line_wm.set_ydata(u_wm_copy)
            self.ax1.set_title(f"Action Onset Field (t={sim_time_copy:.1f}s)")
            
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception as e:
            self.get_logger().warn(f"Plot update failed: {e}")

    # ========================================
    # SAVE RESULTS
    # ========================================
    def save_results(self):
        """Save recall results and generate summary plots."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("SAVING RECALL RESULTS...")
        
        try:
            results_dir = os.path.join(self.data_path, 'recall_results')
            os.makedirs(results_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            with self.data_lock:
                u_act_history = np.array(self.u_act_history)
                u_wm_history = np.array(self.u_wm_history)
                time_history = np.array(self.time_history)

                # Print summary
                self.get_logger().info("-" * 50)
                self.get_logger().info("PREDICTED SEQUENCE:")
                for i, (t, label, pos) in enumerate(self.predicted_sequence):
                    self.get_logger().info(f"  {i+1}. t={t:.2f}s → {label} (x={pos:+.0f})")

                # Compare with learned sequence
                if self.learned_sequence and self.learned_sequence['events']:
                    self.get_logger().info("-" * 50)
                    self.get_logger().info("COMPARISON WITH LEARNED SEQUENCE:")
                    self.get_logger().info(f"  {'Learned':<20} {'Predicted':<20} {'Δt':<10}")
                    
                    for i, (t_learn, pos) in enumerate(self.learned_sequence['events']):
                        label = self.object_map.get(pos, f"x={pos}")
                        t_pred = self.crossing_times.get(pos, None)
                        if t_pred is not None:
                            delta_t = t_pred - t_learn
                            self.get_logger().info(
                                f"  {label} @ {t_learn:.1f}s      "
                                f"{label} @ {t_pred:.2f}s      "
                                f"{delta_t:+.2f}s"
                            )
                        else:
                            self.get_logger().info(
                                f"  {label} @ {t_learn:.1f}s      NOT PREDICTED"
                            )

                # Create time-course plot
                fig, ax = plt.subplots(figsize=(12, 6))
                
                colors = ['red', 'green', 'blue']
                for i, (pos, label) in enumerate(self.object_map.items()):
                    ax.plot(time_history, u_act_history[:, i], 
                           color=colors[i], linewidth=2, label=label)
                    
                    if pos in self.crossing_times:
                        t_cross = self.crossing_times[pos]
                        ax.axvline(x=t_cross, color=colors[i], linestyle='--', alpha=0.5)
                        ax.annotate(f'{label}: {t_cross:.2f}s',
                                   xy=(t_cross, self.theta_act),
                                   xytext=(t_cross + 0.5, self.theta_act + 0.3 + i*0.3),
                                   fontsize=10, color=colors[i])

                ax.axhline(y=self.theta_act, color='black', linestyle='--',
                          linewidth=1.5, label=f'Threshold (θ={self.theta_act})')
                ax.set_xlabel('Time (s)', fontsize=12)
                ax.set_ylabel('u_act Activation', fontsize=12)
                ax.set_title('DNF Recall: Action Onset Field Activity Over Time', fontsize=14)
                ax.legend(loc='upper right')
                ax.grid(True, alpha=0.3)
                ax.set_xlim(0, max(time_history) if len(time_history) > 0 else self.t_lim)
                ax.set_ylim(-2, 5)

                plt.tight_layout()

                # Save plot
                timecourse_path = os.path.join(results_dir, f'recall_timecourse_{timestamp}.png')
                fig.savefig(timecourse_path, dpi=150, bbox_inches='tight')
                plt.close(fig)

                # Save data
                npz_path = os.path.join(results_dir, f'recall_data_{timestamp}.npz')
                np.savez_compressed(
                    npz_path,
                    time_history=time_history,
                    u_act_history=u_act_history,
                    u_wm_history=u_wm_history,
                    input_positions=np.array(self.input_positions),
                    object_labels=self.object_labels,
                    crossing_times=self.crossing_times,
                    predicted_sequence=self.predicted_sequence
                )

                # Save summary JSON
                summary = {
                    'timestamp': timestamp,
                    'predicted_sequence': [
                        {'time': t, 'label': label, 'position': pos}
                        for t, label, pos in self.predicted_sequence
                    ],
                    'crossing_times': {str(k): v for k, v in self.crossing_times.items()}
                }
                
                summary_path = os.path.join(results_dir, f'recall_summary_{timestamp}.json')
                with open(summary_path, 'w') as f:
                    json.dump(summary, f, indent=2)

            self.get_logger().info("-" * 50)
            self.get_logger().info(f"Results saved in: {os.path.abspath(results_dir)}/")
            self.get_logger().info("=" * 50)
            self.get_logger().info("🎉 RECALL COMPLETE!")
            self.get_logger().info("=" * 50)

        except Exception as e:
            self.get_logger().error(f"Error saving results: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = DNFRecallNode()

    try:
        while rclpy.ok() and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.01)

        if node.is_finished:
            node.save_results()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user - saving results...")
        node.save_results()
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()