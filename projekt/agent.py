import socket
import threading
import random
import time
from multiprocessing import Process, Queue
import struct

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
        self.sensors = {}
        self.mib = LMIB(self.sensors)
        self.queue = Queue()
        self.lock = threading.Lock()
        self.msg_id_counter = 0

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

    def encode_pdu(self, tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list):
        """Encodes the PDU into a byte string."""
        tag_bytes = tag.encode()
        type_bytes = struct.pack("!B", msg_type)
        timestamp_bytes = struct.pack("!I", timestamp)
        msg_id_bytes = struct.pack("!I", msg_id)
        iid_list_bytes = b"".join(struct.pack("!H", iid) for iid in iid_list)

        v_list_bytes = b""
        for value in v_list:
            if isinstance(value, int):
                v_list_bytes += struct.pack("!i", value)
            elif isinstance(value, bytes):
                v_list_bytes += struct.pack("!H", len(value)) + value
            else:
                raise ValueError("Invalid value type in v_list")

        t_list_bytes = b"".join(struct.pack("!I", time_val) for time_val in t_list)
        e_list_bytes = b"".join(struct.pack("!H", error) for error in e_list)

        return tag_bytes + type_bytes + timestamp_bytes + msg_id_bytes + iid_list_bytes + v_list_bytes + t_list_bytes + e_list_bytes

    def decode_pdu(self, data):
        """Decodes the PDU from a byte string."""
        tag = data[:1].decode()
        msg_type = struct.unpack("!B", data[1:2])[0]
        timestamp = struct.unpack("!I", data[2:6])[0]
        msg_id = struct.unpack("!I", data[6:10])[0]

        iid_list = []
        offset = 10
        while offset < len(data) and data[offset:offset+2] != b'':
            iid = struct.unpack("!H", data[offset:offset+2])[0]
            iid_list.append(iid)
            offset += 2

        v_list = []
        while offset < len(data):
            if offset + 4 <= len(data):
                try:
                    value = struct.unpack("!i", data[offset:offset+4])[0]
                    v_list.append(value)
                    offset += 4
                except struct.error:
                    if offset + 2 <= len(data):
                        str_len = struct.unpack("!H", data[offset:offset+2])[0]
                        offset += 2
                        if offset + str_len <= len(data):
                            value = data[offset:offset+str_len].decode()
                            v_list.append(value)
                            offset += str_len
                        else:
                            break
                    else:
                        break
            else:
                break
        
        t_list = []
        while offset < len(data) and offset + 4 <= len(data):
            time_val = struct.unpack("!I", data[offset:offset+4])[0]
            t_list.append(time_val)
            offset += 4

        e_list = []
        while offset < len(data) and offset + 2 <= len(data):
            error = struct.unpack("!H", data[offset:offset+2])[0]
            e_list.append(error)
            offset += 2

        return tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list

    def handle_client(self, connection, address, shutdown_flag):
        try:
            while not shutdown_flag.is_set():
                data, addr = connection.recvfrom(1024)
                print(f"Received message from {addr}: {data}")

                tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list = self.decode_pdu(data)

                if tag == 'G':
                    response_data = self.handle_get(iid_list)
                    response_pdu = self.encode_pdu('R', 1, int(time.time()), self.get_next_msg_id(), iid_list, response_data, [], [])
                    connection.sendto(response_pdu, addr)
                elif tag == 'S':
                    self.handle_set(iid_list, v_list)
                    response_pdu = self.encode_pdu('R', 1, int(time.time()), self.get_next_msg_id(), iid_list, [], [], [])
                    connection.sendto(response_pdu, addr)
                elif tag == 'B':
                    self.handle_beacon(connection, addr)

        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            connection.close()

    def handle_get(self, iid_list):
        response_data = []
        for iid in iid_list:
            if iid == 11:
                response_data.append(int(self.mib.get_data("device.lMibId")))
            elif iid == 12:
                response_data.append(self.mib.get_data("device.id").encode())
            elif iid == 13:
                response_data.append(self.mib.get_data("device.type").encode())
            elif iid == 14:
                response_data.append(int(self.mib.get_data("device.beaconRate")))
            elif iid == 15:
                response_data.append(int(self.mib.get_data("device.nSensors")))
            elif iid == 16:
                response_data.append(int(self.mib.get_data("device.dateAndTime")))
            elif iid == 17:
                response_data.append(int(self.mib.get_data("device.upTime")))
            elif iid == 18:
                response_data.append(int(self.mib.get_data("device.opStatus")))
            elif iid == 19:
                response_data.append(int(self.mib.get_data("device.reset")))
            elif iid == 21:
                response_data.append(self.mib.get_data("sensors.1.id").encode())
            elif iid == 22:
                response_data.append(self.mib.get_data("sensors.1.type").encode())
            elif iid == 23:
                response_data.append(int(self.mib.get_data("sensors.1.sampleValue")))
            elif iid == 24:
                response_data.append(int(self.mib.get_data("sensors.1.minValue")))
            elif iid == 25:
                response_data.append(int(self.mib.get_data("sensors.1.maxValue")))
            elif iid == 26:
                response_data.append(int(self.mib.get_data("sensors.1.lastSamplingTime")))
            elif iid == 27:
                response_data.append(int(self.mib.get_data("sensors.1.samplingRate")))
            elif iid == 31:
                response_data.append(self.mib.get_data("sensors.2.id").encode())
            elif iid == 32:
                response_data.append(self.mib.get_data("sensors.2.type").encode())
            elif iid == 33:
                response_data.append(int(self.mib.get_data("sensors.2.sampleValue")))
            elif iid == 34:
                response_data.append(int(self.mib.get_data("sensors.2.minValue")))
            elif iid == 35:
                response_data.append(int(self.mib.get_data("sensors.2.maxValue")))
            elif iid == 36:
                response_data.append(int(self.mib.get_data("sensors.2.lastSamplingTime")))
            elif iid == 37:
                response_data.append(int(self.mib.get_data("sensors.2.samplingRate")))
            else:
                response_data.append(b"N/A")
        return response_data

    def handle_set(self, iid_list, v_list):
        for iid, value in zip(iid_list, v_list):
            if iid == 14:
                self.mib.set_data("device.beaconRate", value)
            elif iid == 16:
                self.mib.set_data("device.dateAndTime", value)
            elif iid == 19:
                self.mib.set_data("device.reset", value)
            elif iid == 27:
                self.mib.set_data("sensors.1.samplingRate", value)
            elif iid == 37:
                self.mib.set_data("sensors.2.samplingRate", value)

    def handle_beacon(self, connection, addr):
        iid_list = [11, 12, 13, 15, 16, 17, 18]
        v_list = [int(self.mib.get_data("device.lMibId")), self.mib.get_data("device.id").encode(), self.mib.get_data("device.type").encode(), int(self.mib.get_data("device.nSensors")), int(self.mib.get_data("device.dateAndTime")), int(self.mib.get_data("device.upTime")), int(self.mib.get_data("device.opStatus"))]
        beacon_pdu = self.encode_pdu('B', 1, int(time.time()), self.get_next_msg_id(), iid_list, v_list, [], [])
        connection.sendto(beacon_pdu, addr)

    def get_next_msg_id(self):
        self.msg_id_counter += 1
        return self.msg_id_counter

    def start_server(self, shutdown_flag):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            server_socket.bind((self.endereco_agente, self.porta_agente))
            print(f"L-SNMPvS Agent started on {self.endereco_agente}:{self.porta_agente}")

            while not shutdown_flag.is_set():
                client_thread = threading.Thread(target=self.handle_client, args=(server_socket, self.endereco_agente, shutdown_flag))
                client_thread.start()
                time.sleep(0.1)

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

        # Start the beacon in a separate thread
        beacon_thread = threading.Thread(target=self.send_beacon, args=(shutdown_flag,))
        beacon_thread.start()

        # Wait for the threads to finish
        server_thread.join()
        sensors_thread.join()
        update_thread.join()
        beacon_thread.join()

    def send_beacon(self, shutdown_flag):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as beacon_socket:
            beacon_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while not shutdown_flag.is_set():
                self.handle_beacon(beacon_socket, ('<broadcast>', self.porta_agente))
                time.sleep(self.mib.get_data("device.beaconRate"))

# --- Main Execution ---
if __name__ == "__main__":
    agent = AgenteLSNMPvS(
        "0.0.0.0",
        12345,
        "",
        "",
    )

    # Add virtual sensors
    sensor1 = VirtualSensor("1", 0, 100, 1, 2)
    sensor2 = VirtualSensor("2", 50, 150, 2, 1)
    agent.add_sensor(sensor1)
    agent.add_sensor(sensor2)

    # Run the agent
    agent.run()
