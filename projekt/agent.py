import socket
import ssl
import threading
import random
import time
from multiprocessing import Process, Queue
from datetime import datetime, timedelta

# --- Virtual Sensor Class ---
class VirtualSensor:
    def __init__(self, sensor_id, min_value, max_value, update_interval, precision):
        self.sensor_id = sensor_id
        self.min_value = min_value
        self.max_value = max_value
        self.update_interval = update_interval
        self.precision = precision
        self.current_value = self.generate_value()
        self.last_sampling_time = time.time()
        self.sampling_rate = 10  # Default sampling rate is 10 Hz
        self.active = True  # Add the active attribute here, defaulting to True
        self.notification_rate = self.sampling_rate

    def generate_value(self):
        value = random.uniform(self.min_value, self.max_value)
        return round(value, self.precision)

    def update_value(self):
        self.current_value = self.generate_value()
        self.last_sampling_time = time.time()

    def get_value(self):
        return self.current_value
    
    def get_last_sampling_time(self):
        return int(self.last_sampling_time)
    
    def get_sampling_rate(self):
        return self.sampling_rate

    def get_min_value(self):
        return self.min_value

    def get_max_value(self):
        return self.max_value
    
    def get_sample_value_percentage(self):
        if self.min_value == self.max_value:
            return 0  # Avoid division by zero
        percentage = ((self.current_value - self.min_value) / (self.max_value - self.min_value)) * 100
        return int(percentage)

    def start(self, queue, shutdown_flag):
        while not shutdown_flag.is_set():
            if self.active:
                self.update_value()
                queue.put((self.sensor_id, self.current_value))
                print(f"Sensor {self.sensor_id}: {self.current_value}")
            time.sleep(1/self.update_interval)

# --- L-MIB Data Structure ---
class LMIB:
    def __init__(self, sensors):
        self.device_data = {
            "device.lMibId": 1,  # Integer
            "device.id": "Device123",  # String
            "device.type": "Sensing Hub",  # String
            "device.beaconRate": 5,  # Integer (default 5 seconds)
            "device.nSensors": 0,  # Integer (updated dynamically)
            "device.dateAndTime": int(time.time()),  # Timestamp
            "device.upTime": 0,  # Timestamp (seconds since boot)
            "device.opStatus": 1,  # Integer (1 = normal)
            "device.reset": 0,  # Integer (0 = no reset)
        }
        self.sensors_data = {}  # Dictionary to store sensor data
        self.start_time = time.time()
        self.sensors = sensors
        self.lock = threading.Lock()

    def add_sensor(self, sensor):
        with self.lock:
            self.sensors_data[sensor.sensor_id] = {
                "sensors.id": sensor.sensor_id,  # String
                "sensors.type": "temperature",  # String
                "sensors.sampleValue": sensor.get_sample_value_percentage(),  # Integer
                "sensors.minValue": sensor.get_min_value(),  # Integer
                "sensors.maxValue": sensor.get_max_value(),  # Integer
                "sensors.lastSamplingTime": sensor.get_last_sampling_time(),  # Timestamp
                "sensors.samplingRate": sensor.get_sampling_rate(),  # Integer
            }
            self.device_data["device.nSensors"] = len(self.sensors_data)

    def update_sensor_data(self, sensor_id, value):
        with self.lock:
            if sensor_id in self.sensors_data:
                self.sensors_data[sensor_id]["sensors.sampleValue"] = self.get_sensor(sensor_id).get_sample_value_percentage()
                self.sensors_data[sensor_id]["sensors.lastSamplingTime"] = self.get_sensor(sensor_id).get_last_sampling_time()

    def get_data(self, object_name):
        with self.lock:
            if object_name.startswith("device."):
                if object_name == "device.upTime":
                    self.device_data["device.upTime"] = int(time.time() - self.start_time)
                return self.device_data.get(object_name, "N/A")
            elif object_name.startswith("sensors."):
                sensor_id = object_name.split(".")[1]
                attribute = object_name.split(".")[2]
                if sensor_id in self.sensors_data:
                    return self.sensors_data[sensor_id].get(f"sensors.{attribute}", "N/A")
            return "N/A"
    
    def set_data(self, object_name, value):
        with self.lock:
            if object_name == "device.beaconRate":
                if not isinstance(value, int):
                    raise ValueError("device.beaconRate must be an integer")
                if value < 1:
                    raise ValueError("device.beaconRate must be at least 1")
                self.device_data[object_name] = value
            elif object_name == "device.dateAndTime":
                if not isinstance(value, int):
                    raise ValueError("device.dateAndTime must be an integer")
                self.device_data[object_name] = value
            elif object_name == "device.reset":
                if not isinstance(value, int):
                    raise ValueError("device.reset must be an integer")
                if value == 1:
                    self.reset_device()
                self.device_data[object_name] = value
            elif object_name.startswith("sensors."):
                sensor_id = object_name.split(".")[1]
                attribute = object_name.split(".")[2]
                if sensor_id in self.sensors_data:
                    if attribute == "samplingRate":
                        if not isinstance(value, int):
                            raise ValueError("sensors.samplingRate must be an integer")
                        self.sensors_data[sensor_id][f"sensors.{attribute}"] = value
                        self.get_sensor(sensor_id).sampling_rate = value
                    else:
                        raise ValueError("Invalid attribute for sensors table")
                else:
                    raise ValueError("Invalid sensor id")
            else:
                raise ValueError("Invalid object name")

    def get_sensor(self, sensor_id):
        return self.sensors.get(sensor_id)
    
    def reset_device(self):
        with self.lock:
            self.device_data["device.upTime"] = 0
            self.device_data["device.dateAndTime"] = int(time.time())
            self.start_time = time.time()
            for sensor_id in self.sensors_data:
                self.sensors_data[sensor_id]["sensors.lastSamplingTime"] = self.get_sensor(sensor_id).get_last_sampling_time()
                self.sensors_data[sensor_id]["sensors.sampleValue"] = self.get_sensor(sensor_id).get_sample_value_percentage()

# --- L-SNMPvS Agent Class ---
class AgenteLSNMPvS:
    def __init__(self, endereco_agente, porta_agente, caminho_certificado, caminho_chave_privada):
        self.endereco_agente = endereco_agente
        self.porta_agente = porta_agente
        self.caminho_certificado = caminho_certificado
        self.caminho_chave_privada = caminho_chave_privada
        self.sensors = {}
        self.mib = LMIB(self.sensors)
        self.queue = Queue()
        self.lock = threading.Lock()

    def add_sensor(self, sensor):
        with self.lock:
            self.sensors[sensor.sensor_id] = sensor
            self.mib.add_sensor(sensor)

    def get_sensor_value(self, sensor_id):
        with self.lock:
            if sensor_id in self.sensors:
                return self.sensors[sensor_id].get_value()
            else:
                return None

    def handle_client(self, connection, shutdown_flag):
        try:
            message = connection.recv(1024).decode()
            print(f"Received message: {message}")

            if message.startswith("GET"):
                object_name = message.split(" ")[1]
                value = self.mib.get_data(object_name)
                connection.send(f"{value}".encode())
            elif message.startswith("SET"):
                object_name, value = message.split(" ")[1], message.split(" ")[2]
                try:
                    self.mib.set_data(object_name, int(value))
                    connection.send(f"OK".encode())
                except ValueError as e:
                    connection.send(f"ERROR: {e}".encode())

        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            connection.close()

    def start_server(self, shutdown_flag):
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(certfile='/home/fuzzymind/Documents/GSR/projekt/certificado.pem', keyfile='/home/fuzzymind/Documents/GSR/projekt/chave_privada.pem')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((self.endereco_agente, self.porta_agente))
            server_socket.listen()
            print(f"L-SNMPvS Agent started on {self.endereco_agente}:{self.porta_agente}")

            while not shutdown_flag.is_set():
                client_connection, client_address = server_socket.accept()
                ssl_client_connection = ssl_context.wrap_socket(client_connection, server_side=True)
                client_thread = threading.Thread(target=self.handle_client, args=(ssl_client_connection, shutdown_flag))
                client_thread.start()

    def update_sensor_data(self, shutdown_flag):
        while not shutdown_flag.is_set():
            if not self.queue.empty():
                sensor_id, value = self.queue.get()
                self.mib.update_sensor_data(sensor_id, value)
            time.sleep(0.1)

    def start_sensors(self, shutdown_flag):
        sensor_threads = []
        for sensor in self.sensors.values():
            sensor_thread = threading.Thread(target=sensor.start, args=(self.queue, shutdown_flag))
            sensor_threads.append(sensor_thread)
            sensor_thread.start()
        for sensor_thread in sensor_threads:
            sensor_thread.join()

    def run(self, shutdown_flag):
        # Start the server in a separate thread
        server_thread = threading.Thread(target=self.start_server, args=(shutdown_flag,))
        server_thread.start()

        # Start the sensors in a separate thread
        sensors_thread = threading.Thread(target=self.start_sensors, args=(shutdown_flag,))
        sensors_thread.start()

        # Start the update_sensor_data in a separate thread
        update_thread = threading.Thread(target=self.update_sensor_data, args=(shutdown_flag,))
        update_thread.start()

        # Wait for the threads to finish
        server_thread.join()
        sensors_thread.join()
        update_thread.join()

# --- Main Execution ---
if __name__ == "__main__":
    agent = AgenteLSNMPvS(
        "127.0.0.1",
        12345,
        "/home/fuzzymind/Documents/GSR/projekt/certificado.pem",
        "/home/fuzzymind/Documents/GSR/projekt/chave_privada.pem",
    )

    # Add virtual sensors
    sensor1 = VirtualSensor("1", 0, 100, 1, 2)
    sensor2 = VirtualSensor("2", 50, 150, 2, 1)
    agent.add_sensor(sensor1)
    agent.add_sensor(sensor2)

    # Run the agent
    agent.run()
