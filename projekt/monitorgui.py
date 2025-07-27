import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import time
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
from monitor import LSNMPvSMonitor

AGENT_IP = '127.0.0.1'
AGENT_PORT = 12345
REFRESH_INTERVAL_MS = 2000

class MonitorApp(tk.Tk):
    """GUI frontend for the L-SNMPvS monitor, displaying device and sensor data."""
    def __init__(self, monitor):
        """Initializes the GUI with a monitor backend and sets up data structures."""
        super().__init__()
        self.monitor = monitor
        self.title("L-SNMPvS Monitor")
        self.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.geometry("700x950")
        self.device_id_var = tk.StringVar(value="N/A")
        self.device_type_var = tk.StringVar(value="N/A")
        self.device_sensors_var = tk.StringVar(value="N/A")
        self.device_uptime_var = tk.StringVar(value="N/A")
        self.device_status_var = tk.StringVar(value="N/A")
        self.device_beacon_rate_var = tk.StringVar(value="N/A")
        self.sensor1_value_var = tk.StringVar(value="N/A")
        self.sensor1_last_sample_var = tk.StringVar(value="N/A")
        self.sensor1_rate_var = tk.StringVar(value="N/A")
        self.sensor2_value_var = tk.StringVar(value="N/A")
        self.sensor2_last_sample_var = tk.StringVar(value="N/A")
        self.sensor2_rate_var = tk.StringVar(value="N/A")
        self.status_var = tk.StringVar(value="Initializing...")
        self.max_history_points = 100
        self.sensor_data_history = {
            "sensors.1": {"values": deque(maxlen=self.max_history_points),
                          "timestamps": deque(maxlen=self.max_history_points),
                          "name": "Sensor 1"},
            "sensors.2": {"values": deque(maxlen=self.max_history_points),
                          "timestamps": deque(maxlen=self.max_history_points),
                          "name": "Sensor 2"}
        }
        self.history_lock = threading.Lock()
        self.sensor_plot_elements = {}
        for sensor_key, sensor_info in self.sensor_data_history.items():
            fig = Figure(figsize=(6, 2.8), dpi=90)
            ax = fig.add_subplot(111)
            self.sensor_plot_elements[sensor_key] = {
                "fig": fig, "ax": ax, "canvas": None, "name": sensor_info["name"]
            }
        self.is_running = True
        self.create_widgets()
        self._update_sensor_plots()
        self.update_data()

    def create_widgets(self):
        """Creates and arranges GUI widgets for device and sensor information."""
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(expand=True, fill=tk.BOTH)
        device_frame = ttk.LabelFrame(main_frame, text="Device Info", padding="10")
        device_frame.pack(fill=tk.X, pady=5)
        device_frame.columnconfigure(1, weight=1)
        ttk.Label(device_frame, text="ID:").grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Label(device_frame, textvariable=self.device_id_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(device_frame, text="Type:").grid(row=1, column=0, sticky=tk.W, padx=5)
        ttk.Label(device_frame, textvariable=self.device_type_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(device_frame, text="Sensors:").grid(row=2, column=0, sticky=tk.W, padx=5)
        ttk.Label(device_frame, textvariable=self.device_sensors_var).grid(row=2, column=1, sticky=tk.W)
        ttk.Label(device_frame, text="Uptime:").grid(row=3, column=0, sticky=tk.W, padx=5)
        ttk.Label(device_frame, textvariable=self.device_uptime_var).grid(row=3, column=1, sticky=tk.W)
        ttk.Label(device_frame, text="Status:").grid(row=4, column=0, sticky=tk.W, padx=5)
        ttk.Label(device_frame, textvariable=self.device_status_var).grid(row=4, column=1, sticky=tk.W)
        ttk.Label(device_frame, text="Beacon Rate (s):").grid(row=5, column=0, sticky=tk.W, padx=5, pady=(5,0))
        ttk.Label(device_frame, textvariable=self.device_beacon_rate_var).grid(row=5, column=1, sticky=tk.W, pady=(5,0))
        self.beacon_rate_entry = ttk.Entry(device_frame, width=5)
        self.beacon_rate_entry.grid(row=5, column=2, padx=5, pady=(5,0))
        ttk.Button(device_frame, text="Set", command=self.set_beacon_rate).grid(row=5, column=3, padx=5, pady=(5,0))
        ttk.Button(device_frame, text="Reset Device", command=self.reset_device).grid(row=6, column=0, columnspan=4, pady=5)
        sensor1_frame = ttk.LabelFrame(main_frame, text="Sensor 1 Info", padding="10")
        sensor1_frame.pack(fill=tk.X, pady=5)
        sensor1_frame.columnconfigure(1, weight=1)
        ttk.Label(sensor1_frame, text="Value (%):").grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor1_frame, textvariable=self.sensor1_value_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(sensor1_frame, text="Last Sample:").grid(row=1, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor1_frame, textvariable=self.sensor1_last_sample_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(sensor1_frame, text="Sampling Rate (Hz):").grid(row=2, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor1_frame, textvariable=self.sensor1_rate_var).grid(row=2, column=1, sticky=tk.W)
        self.sensor1_rate_entry = ttk.Entry(sensor1_frame, width=5)
        self.sensor1_rate_entry.grid(row=2, column=2, padx=5)
        ttk.Button(sensor1_frame, text="Set", command=lambda: self.set_sensor_rate(1)).grid(row=2, column=3, padx=5)
        s1_plot_elements = self.sensor_plot_elements["sensors.1"]
        s1_canvas = FigureCanvasTkAgg(s1_plot_elements["fig"], master=sensor1_frame)
        s1_plot_elements["canvas"] = s1_canvas
        s1_canvas_widget = s1_canvas.get_tk_widget()
        s1_canvas_widget.grid(row=3, column=0, columnspan=4, pady=(10,0), sticky="ew")
        sensor2_frame = ttk.LabelFrame(main_frame, text="Sensor 2 Info", padding="10")
        sensor2_frame.pack(fill=tk.X, pady=5)
        sensor2_frame.columnconfigure(1, weight=1)
        ttk.Label(sensor2_frame, text="Value (%):").grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor2_frame, textvariable=self.sensor2_value_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(sensor2_frame, text="Last Sample:").grid(row=1, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor2_frame, textvariable=self.sensor2_last_sample_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(sensor2_frame, text="Sampling Rate (Hz):").grid(row=2, column=0, sticky=tk.W, padx=5)
        ttk.Label(sensor2_frame, textvariable=self.sensor2_rate_var).grid(row=2, column=1, sticky=tk.W)
        self.sensor2_rate_entry = ttk.Entry(sensor2_frame, width=5)
        self.sensor2_rate_entry.grid(row=2, column=2, padx=5)
        ttk.Button(sensor2_frame, text="Set", command=lambda: self.set_sensor_rate(2)).grid(row=2, column=3, padx=5)
        s2_plot_elements = self.sensor_plot_elements["sensors.2"]
        s2_canvas = FigureCanvasTkAgg(s2_plot_elements["fig"], master=sensor2_frame)
        s2_plot_elements["canvas"] = s2_canvas
        s2_canvas_widget = s2_canvas.get_tk_widget()
        s2_canvas_widget.grid(row=3, column=0, columnspan=4, pady=(10,0), sticky="ew")
        status_bar = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding="2 5")
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def format_timestamp(self, ts):
        """Formats a Unix timestamp into a readable string."""
        if isinstance(ts, int) and ts > 0:
            try:
                return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ts))
            except (OverflowError, OSError):
                return f"Invalid TS ({ts})"
        return "N/A"

    def format_uptime(self, seconds):
        """Converts seconds to a formatted uptime string (HH:MM:SS)."""
        if isinstance(seconds, int) and seconds >= 0:
            m, s = divmod(seconds, 60)
            h, m = divmod(m, 60)
            return f"{h:d}:{m:02d}:{s:02d}"
        return "N/A"

    def update_data(self):
        """Schedules periodic data updates from the L-SNMPvS monitor."""
        self.status_var.set(f"Fetching data... ({time.strftime('%H:%M:%S')})")
        self.update()
        oids_to_get = [
            'device.id', 'device.type', 'device.nSensors', 'device.upTime',
            'device.opStatus', 'device.beaconRate',
            'sensors.1.sampleValue', 'sensors.1.lastSamplingTime', 'sensors.1.samplingRate',
            'sensors.2.sampleValue', 'sensors.2.lastSamplingTime', 'sensors.2.samplingRate'
        ]
        thread = threading.Thread(target=self._fetch_and_update_ui, args=(oids_to_get,), daemon=True)
        thread.start()
        if self.is_running:
            self.after(REFRESH_INTERVAL_MS, self.update_data)

    def _fetch_and_update_ui(self, oids_to_get):
        """Fetches data from the monitor and updates the GUI."""
        data = self.monitor.get_bulk(oids_to_get)
        self.after(0, self._update_ui_labels, data)

    def _update_ui_labels(self, data):
        """Updates GUI labels and sensor history with fetched data."""
        if data is None:
            self.status_var.set("Error: Failed to get data from agent.")
            return
        self.device_id_var.set(data.get('device.id', 'N/A'))
        self.device_type_var.set(data.get('device.type', 'N/A'))
        num_sensors = data.get('device.nSensors')
        self.device_sensors_var.set(str(num_sensors) if num_sensors is not None else 'N/A')
        self.device_uptime_var.set(self.format_uptime(data.get('device.upTime')))
        op_status_val = data.get('device.opStatus')
        self.device_status_var.set('Normal' if op_status_val == 1 else ('Abnormal' if op_status_val is not None else 'N/A'))
        self.device_beacon_rate_var.set(str(data.get('device.beaconRate', 'N/A')))
        s1_val = data.get('sensors.1.sampleValue')
        s1_ts = data.get('sensors.1.lastSamplingTime')
        self.sensor1_value_var.set(str(s1_val) if s1_val is not None else 'N/A')
        self.sensor1_last_sample_var.set(self.format_timestamp(s1_ts))
        self.sensor1_rate_var.set(str(data.get('sensors.1.samplingRate', 'N/A')))
        if s1_val is not None and s1_ts is not None:
            self.sensor_data_history["sensors.1"]["values"].append(s1_val)
            self.sensor_data_history["sensors.1"]["timestamps"].append(s1_ts)
        s2_val = data.get('sensors.2.sampleValue')
        s2_ts = data.get('sensors.2.lastSamplingTime')
        self.sensor2_value_var.set(str(s2_val) if s2_val is not None else 'N/A')
        self.sensor2_last_sample_var.set(self.format_timestamp(s2_ts))
        self.sensor2_rate_var.set(str(data.get('sensors.2.samplingRate', 'N/A')))
        if s2_val is not None and s2_ts is not None:
            self.sensor_data_history["sensors.2"]["values"].append(s2_val)
            self.sensor_data_history["sensors.2"]["timestamps"].append(s2_ts)
        self._update_sensor_plots()
        self.status_var.set(f"Ready. Last update: {time.strftime('%H:%M:%S')}")

    def set_sensor_rate(self, sensor_num):
        """Sets the sampling rate for a specified sensor."""
        oid_name = f'sensors.{sensor_num}.samplingRate'
        entry_widget = self.sensor1_rate_entry if sensor_num == 1 else self.sensor2_rate_entry
        try:
            new_rate = int(entry_widget.get())
            if new_rate <= 0:
                raise ValueError("Rate must be positive")
            self.status_var.set(f"Setting {oid_name} to {new_rate}...")
            self.update()
            thread = threading.Thread(target=self._set_value_worker, args=(oid_name, new_rate), daemon=True)
            thread.start()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid positive integer for the sampling rate.")
            self.status_var.set("Ready.")
        finally:
            entry_widget.delete(0, tk.END)

    def set_beacon_rate(self):
        """Sets the beacon rate for the device."""
        oid_name = 'device.beaconRate'
        entry_widget = self.beacon_rate_entry
        try:
            new_rate = int(entry_widget.get())
            if new_rate <= 0:
                raise ValueError("Rate must be positive")
            self.status_var.set(f"Setting {oid_name} to {new_rate}...")
            self.update()
            thread = threading.Thread(target=self._set_value_worker, args=(oid_name, new_rate), daemon=True)
            thread.start()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid positive integer for the beacon rate.")
            self.status_var.set("Ready.")
        finally:
            entry_widget.delete(0, tk.END)

    def reset_device(self):
        """Sends a reset command to the agent after user confirmation."""
        oid_name = 'device.reset'
        value = 1
        if messagebox.askyesno("Confirm Reset", "Are you sure you want to reset the agent device?"):
            self.status_var.set(f"Sending reset command...")
            self.update()
            thread = threading.Thread(target=self._set_value_worker, args=(oid_name, value), daemon=True)
            thread.start()

    def _set_value_worker(self, oid_name, value):
        """Executes SET operations in a separate thread."""
        success = self.monitor.set(oid_name, value)
        self.after(0, self._update_set_status, oid_name, success)

    def _update_set_status(self, oid_name, success):
        """Updates the GUI status after a SET operation."""
        if success:
            self.status_var.set(f"Successfully set {oid_name}.")
            self.update_data()
        else:
            self.status_var.set(f"Failed to set {oid_name}. Check console/agent logs.")
            messagebox.showwarning("Set Failed", f"Could not set {oid_name}. See logs for details.")

    def _update_sensor_plots(self):
        """Updates the sensor data plots for each sensor."""
        for sensor_key, history in self.sensor_data_history.items():
            if sensor_key not in self.sensor_plot_elements:
                continue
            plot_elements = self.sensor_plot_elements[sensor_key]
            fig = plot_elements["fig"]
            ax = plot_elements["ax"]
            canvas = plot_elements["canvas"]
            sensor_display_name = plot_elements["name"]
            with self.history_lock:
                values = list(history["values"])
                timestamps_unix = list(history["timestamps"])
                ax.clear()
                if values and timestamps_unix and len(values) == len(timestamps_unix) and len(values) > 1:
                    start_time = timestamps_unix[0]
                    relative_times = [ts - start_time for ts in timestamps_unix]
                    ax.plot(relative_times, values, marker='.', linestyle='-', markersize=5, color='dodgerblue')
                    num_points = len(relative_times)
                    if num_points > 1:
                        max_ticks = 7
                        step = max(1, num_points // max_ticks)
                        tick_indices = list(range(0, num_points, step))
                        if num_points - 1 not in tick_indices:
                            tick_indices.append(num_points - 1)
                        ax.set_xticks([relative_times[i] for i in tick_indices])
                        ax.set_xticklabels([time.strftime('%H:%M:%S', time.localtime(timestamps_unix[i])) for i in tick_indices],
                                           rotation=30, ha="right", fontsize=8)
                    ax.set_xlabel("Relative Time (seconds)", fontsize=9)
                    ax.set_xlim(left=0)
                else:
                    ax.set_xlabel("Relative Time (collecting data...)", fontsize=9)
                    ax.set_xlim(left=0)
                    ax.set_xticks([])
                    ax.set_xticklabels([])
                ax.set_ylabel("Value", fontsize=9)
                ax.set_title(f"{sensor_display_name} Data", fontsize=10)
                ax.grid(True, linestyle='--', alpha=0.7)
                fig.tight_layout(pad=0.5)
                if canvas:
                    canvas.draw()

    def _on_closing(self):
        """Handles window closure by stopping updates and destroying the GUI."""
        self.is_running = False
        self.destroy()

if __name__ == "__main__":
    """Initializes and runs the L-SNMPvS monitor GUI."""
    monitor_backend = LSNMPvSMonitor(AGENT_IP, AGENT_PORT, monitor_id="MONITOR1", secrets_file="monitor_secrets.json")
    app = MonitorApp(monitor_backend)
    app.mainloop()
