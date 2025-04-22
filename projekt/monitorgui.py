import tkinter as tk
from tkinter import ttk # Themed widgets
from tkinter import messagebox
import time
import threading

# Import the monitor class from monitor.py
from monitor import LSNMPvSMonitor

# --- Configuration ---
AGENT_IP = '127.0.0.1'
AGENT_PORT = 12345
REFRESH_INTERVAL_MS = 2000 # How often to refresh data (e.g., 2000ms = 2s)

class MonitorApp(tk.Tk):
    def __init__(self, monitor):
        super().__init__()
        self.monitor = monitor
        self.title("L-SNMPvS Monitor")
        self.geometry("550x500") # Adjusted size

        # --- Data Variables ---
        # Use StringVar for labels that change
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

        # --- UI Setup ---
        self.create_widgets()

        # --- Initial Data Fetch & Start Refresh Loop ---
        self.update_data() # Fetch initial data
        # self.after(REFRESH_INTERVAL_MS, self.update_data) # Schedule first refresh

    def create_widgets(self):
        # --- Main Frame ---
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(expand=True, fill=tk.BOTH)

        # --- Device Info Frame ---
        device_frame = ttk.LabelFrame(main_frame, text="Device Info", padding="10")
        device_frame.pack(fill=tk.X, pady=5)
        device_frame.columnconfigure(1, weight=1) # Make value column expand

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

        # Beacon Rate Control
        ttk.Label(device_frame, text="Beacon Rate (s):").grid(row=5, column=0, sticky=tk.W, padx=5, pady=(5,0))
        ttk.Label(device_frame, textvariable=self.device_beacon_rate_var).grid(row=5, column=1, sticky=tk.W, pady=(5,0))
        self.beacon_rate_entry = ttk.Entry(device_frame, width=5)
        self.beacon_rate_entry.grid(row=5, column=2, padx=5, pady=(5,0))
        ttk.Button(device_frame, text="Set", command=self.set_beacon_rate).grid(row=5, column=3, padx=5, pady=(5,0))

        # Reset Button
        ttk.Button(device_frame, text="Reset Device", command=self.reset_device).grid(row=6, column=0, columnspan=4, pady=5)


        # --- Sensor 1 Frame ---
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


        # --- Sensor 2 Frame ---
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

        # --- Status Bar ---
        status_bar = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding="2 5")
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def format_timestamp(self, ts):
        """Formats a Unix timestamp or returns N/A."""
        if isinstance(ts, int) and ts > 0:
            try:
                return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ts))
            except (OverflowError, OSError):
                return f"Invalid TS ({ts})"
        return "N/A"

    def format_uptime(self, seconds):
        """Formats uptime in seconds to H:M:S or returns N/A."""
        if isinstance(seconds, int) and seconds >= 0:
            m, s = divmod(seconds, 60)
            h, m = divmod(m, 60)
            return f"{h:d}:{m:02d}:{s:02d}"
        return "N/A"

    def update_data(self):
        """Fetches data from the agent and updates the UI."""
        self.status_var.set(f"Fetching data... ({time.strftime('%H:%M:%S')})")
        self.update() # Force UI update for status message

        # Define OIDs to fetch
        oids_to_get = [
            'device.id', 'device.type', 'device.nSensors', 'device.upTime',
            'device.opStatus', 'device.beaconRate',
            'sensors.1.sampleValue', 'sensors.1.lastSamplingTime', 'sensors.1.samplingRate',
            'sensors.2.sampleValue', 'sensors.2.lastSamplingTime', 'sensors.2.samplingRate'
        ]

        # Perform the bulk get in a separate thread to avoid blocking the GUI
        # Using threading directly here for simplicity, could use asyncio/other methods
        thread = threading.Thread(target=self._fetch_and_update_ui, args=(oids_to_get,), daemon=True)
        thread.start()

        # Schedule the next update
        self.after(REFRESH_INTERVAL_MS, self.update_data)


    def _fetch_and_update_ui(self, oids_to_get):
        """Worker function to fetch data and schedule UI update."""
        data = self.monitor.get_bulk(oids_to_get)

        # Schedule the UI update back on the main thread
        self.after(0, self._update_ui_labels, data)


    def _update_ui_labels(self, data):
        """Updates the UI labels with fetched data (runs in main thread)."""
        if data is None:
            self.status_var.set("Error: Failed to get data from agent.")
            # Optionally clear all fields or set to Error
            self.device_id_var.set("Error")
            # ... set others to Error or N/A ...
            return

        # Update Device Info
        self.device_id_var.set(data.get('device.id', 'N/A'))
        self.device_type_var.set(data.get('device.type', 'N/A'))
        num_sensors = data.get('device.nSensors')
        self.device_sensors_var.set(str(num_sensors) if num_sensors is not None else 'N/A')
        self.device_uptime_var.set(self.format_uptime(data.get('device.upTime')))
        op_status_val = data.get('device.opStatus')
        self.device_status_var.set('Normal' if op_status_val == 1 else ('Abnormal' if op_status_val is not None else 'N/A'))
        self.device_beacon_rate_var.set(str(data.get('device.beaconRate', 'N/A')))

        # Update Sensor 1 Info
        self.sensor1_value_var.set(str(data.get('sensors.1.sampleValue', 'N/A')))
        self.sensor1_last_sample_var.set(self.format_timestamp(data.get('sensors.1.lastSamplingTime')))
        self.sensor1_rate_var.set(str(data.get('sensors.1.samplingRate', 'N/A')))

        # Update Sensor 2 Info
        self.sensor2_value_var.set(str(data.get('sensors.2.sampleValue', 'N/A')))
        self.sensor2_last_sample_var.set(self.format_timestamp(data.get('sensors.2.lastSamplingTime')))
        self.sensor2_rate_var.set(str(data.get('sensors.2.samplingRate', 'N/A')))

        self.status_var.set(f"Ready. Last update: {time.strftime('%H:%M:%S')}")


    def set_sensor_rate(self, sensor_num):
        """Attempts to set the sampling rate for a given sensor."""
        oid_name = f'sensors.{sensor_num}.samplingRate'
        entry_widget = self.sensor1_rate_entry if sensor_num == 1 else self.sensor2_rate_entry

        try:
            new_rate = int(entry_widget.get())
            if new_rate <= 0:
                raise ValueError("Rate must be positive")

            self.status_var.set(f"Setting {oid_name} to {new_rate}...")
            self.update()

            # Run set in a thread
            thread = threading.Thread(target=self._set_value_worker, args=(oid_name, new_rate), daemon=True)
            thread.start()

        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid positive integer for the sampling rate.")
            self.status_var.set("Ready.")
        finally:
            entry_widget.delete(0, tk.END) # Clear entry field


    def set_beacon_rate(self):
        """Attempts to set the device beacon rate."""
        oid_name = 'device.beaconRate'
        entry_widget = self.beacon_rate_entry

        try:
            new_rate = int(entry_widget.get())
            if new_rate <= 0:
                raise ValueError("Rate must be positive")

            self.status_var.set(f"Setting {oid_name} to {new_rate}...")
            self.update()

            # Run set in a thread
            thread = threading.Thread(target=self._set_value_worker, args=(oid_name, new_rate), daemon=True)
            thread.start()

        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid positive integer for the beacon rate.")
            self.status_var.set("Ready.")
        finally:
            entry_widget.delete(0, tk.END)


    def reset_device(self):
        """Attempts to trigger the device reset."""
        oid_name = 'device.reset'
        value = 1 # Trigger reset by setting to 1

        if messagebox.askyesno("Confirm Reset", "Are you sure you want to reset the agent device?"):
            self.status_var.set(f"Sending reset command...")
            self.update()
            # Run set in a thread
            thread = threading.Thread(target=self._set_value_worker, args=(oid_name, value), daemon=True)
            thread.start()


    def _set_value_worker(self, oid_name, value):
        """Worker function to perform the set operation."""
        success = self.monitor.set(oid_name, value)
        # Schedule UI update for status
        self.after(0, self._update_set_status, oid_name, success)


    def _update_set_status(self, oid_name, success):
        """Updates the status bar after a set operation."""
        if success:
            self.status_var.set(f"Successfully set {oid_name}.")
            # Trigger an immediate refresh after successful set
            self.update_data()
        else:
            self.status_var.set(f"Failed to set {oid_name}. Check console/agent logs.")
            messagebox.showwarning("Set Failed", f"Could not set {oid_name}. See logs for details.")


# --- Main Execution ---
if __name__ == "__main__":
    # Create the monitor instance
    monitor_backend = LSNMPvSMonitor(AGENT_IP, AGENT_PORT)

    # Create and run the Tkinter app
    app = MonitorApp(monitor_backend)
    app.mainloop()
