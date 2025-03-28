import time
import threading
from flask import Flask, render_template, request, redirect, url_for, session
from flask_socketio import SocketIO, emit
from werkzeug.security import generate_password_hash, check_password_hash
from agent import AgenteLSNMPvS, VirtualSensor  # Import from agent.py

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key'  # Replace with a strong secret key
socketio = SocketIO(app, async_mode='threading')

# --- Authentication ---
users = {
    "admin": generate_password_hash("admin_password")  # Replace with a strong password
}

def login_required(func):
    def wrapper(*args, **kwargs):
        if 'logged_in' in session:
            return func(*args, **kwargs)
        else:
            return redirect(url_for('login'))
    wrapper.__name__ = func.__name__
    return wrapper

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
        if username in users and check_password_hash(users[username], password):
            session['logged_in'] = True
            return redirect(url_for('index'))
        else:
            return render_template('login.html', error="Invalid credentials")
    return render_template('login.html')

@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    return redirect(url_for('login'))

# --- Sensor Management ---
agent = AgenteLSNMPvS(
    "0.0.0.0",
    12345,
    "",
    "",
)

sensor1 = VirtualSensor("1", 0, 100, 1, 2)
sensor2 = VirtualSensor("2", 50, 150, 2, 1)
agent.add_sensor(sensor1)
agent.add_sensor(sensor2)

# --- Web Routes ---
@app.route('/')
@login_required
def index():
    return render_template('index.html', sensors=agent.sensors, device=agent.mib.device_data)

@app.route('/config', methods=['GET', 'POST'])
@login_required
def config():
    if request.method == 'POST':
        try:
            new_sampling_rate = float(request.form['sampling_rate'])
            if new_sampling_rate > 0:
                for sensor in agent.sensors.values():
                    agent.mib.set_data(f"sensors.{sensor.sensor_id}.samplingRate", int(new_sampling_rate))
            else:
                raise ValueError("Sampling rate must be greater than zero.")
            
            active_sensors = request.form.getlist('active_sensors')
            for sensor_id, sensor in agent.sensors.items():
                if sensor_id in active_sensors:
                    sensor.active = True
                else:
                    sensor.active = False
            
            new_beacon_rate = int(request.form['beacon_rate'])
            agent.mib.set_data("device.beaconRate", new_beacon_rate)

            new_date_and_time = int(request.form['date_and_time'])
            agent.mib.set_data("device.dateAndTime", new_date_and_time)

        except ValueError as e:
            return render_template('config.html', error=str(e), sensors=agent.sensors, device=agent.mib.device_data)
        return redirect(url_for('index'))
    return render_template('config.html', sensors=agent.sensors, device=agent.mib.device_data)

@app.route('/reset', methods=['POST'])
@login_required
def reset():
    agent.mib.set_data("device.reset", 1)
    return redirect(url_for('index'))

# --- SocketIO Events ---
@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

# --- Shutdown Flag ---
shutdown_flag = threading.Event()

def send_sensor_updates(agent):
    while not shutdown_flag.is_set():
        time.sleep(0.5)  # Adjust as needed
        sensor_data = {}
        for sensor_id, sensor in agent.sensors.items():
            if sensor.active:
                sensor_data[sensor_id] = {
                    "value": agent.mib.get_data(f"sensors.{sensor_id}.sampleValue"),
                    "last_sampling_time": agent.mib.get_data(f"sensors.{sensor_id}.lastSamplingTime"),
                    "sampling_rate": agent.mib.get_data(f"sensors.{sensor_id}.samplingRate"),
                }
        socketio.emit('sensor_update', sensor_data)

def agent_run_wrapper(agent):
    agent.run(shutdown_flag)

# --- Main ---
if __name__ == '__main__':
    # Start the agent in a separate thread
    agent_thread = threading.Thread(target=agent_run_wrapper, args=(agent,))
    agent_thread.start()

    # Start sending sensor updates in a separate thread
    update_thread = threading.Thread(target=send_sensor_updates, args=(agent,))
    update_thread.start()

    try:
        socketio.run(app, debug=True, host="0.0.0.0")
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Signal threads to stop
        shutdown_flag.set()

        # Wait for threads to finish
        update_thread.join()
        agent_thread.join()
        print("Shutdown complete.")
        
        # Shutdown the server
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()
