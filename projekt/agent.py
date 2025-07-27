import socket
import threading
import random
import time
from queue import Queue
import struct
import traceback
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import hashes, hmac
from cryptography.hazmat.backends import default_backend
import json
import os

MASTER_KEY = b"master_key_32_bytes_1234567890ab"

def load_secrets(file_path):
    """Decrypts and loads the agent's configuration from a JSON file."""
    try:
        if not os.path.exists(file_path):
            print(f"Error: Secrets file {file_path} not found.")
            return None
        with open(file_path, 'rb') as f:
            encrypted_data = f.read()
        if len(encrypted_data) < 16 or len(encrypted_data) % 16 != 0:
            print(f"Error: Invalid encrypted data size in {file_path} (must be multiple of 16 bytes).")
            return None
        cipher = Cipher(algorithms.AES(MASTER_KEY), modes.ECB(), backend=default_backend())
        decryptor = cipher.decryptor()
        decrypted_padded = decryptor.update(encrypted_data) + decryptor.finalize()
        padding_len = decrypted_padded[-1]
        if padding_len < 1 or padding_len > 16:
            print(f"Error: Invalid padding length {padding_len} in {file_path}.")
            return None
        decrypted = decrypted_padded[:-padding_len]
        secrets = json.loads(decrypted.decode('utf-8'))
        if not secrets.get("agent_id") or not secrets.get("KA") or not secrets.get("monitors"):
            print(f"Error: Missing required fields in {file_path}.")
            return None
        return secrets
    except Exception as e:
        print(f"Error loading secrets from {file_path}: {e}")
        traceback.print_exc()
        return None

class VirtualSensor:
    """Simulates an IoT sensor generating random values within a specified range."""
    def __init__(self, sensor_id, min_value, max_value, update_interval, precision):
        """Initializes the sensor with ID, value range, update interval, and precision."""
        self.sensor_id = str(sensor_id)
        self.min_value = min_value
        self.max_value = max_value
        self.update_interval = update_interval
        self.precision = precision
        self.current_value = self.generate_value()
        self.last_sampling_time = time.time()
        self.sampling_rate = 10
        self.active = True

    def generate_value(self):
        """Generates a random value within the sensor's range."""
        value = random.uniform(self.min_value, self.max_value)
        return round(value, self.precision)

    def update_value(self):
        """Updates the sensor's value if the update interval has elapsed."""
        now = time.time()
        if self.active and (now - self.last_sampling_time) >= (1.0 / self.update_interval):
            self.current_value = self.generate_value()
            self.last_sampling_time = now
            return True
        return False

    def get_value(self):
        """Returns the current sensor value."""
        return self.current_value

    def get_last_sampling_time(self):
        """Returns the timestamp of the last value update."""
        return int(self.last_sampling_time)

    def get_sampling_rate(self):
        """Returns the current sampling rate."""
        return self.sampling_rate

    def set_sampling_rate(self, rate):
        """Sets the sampling rate if valid."""
        if isinstance(rate, int) and rate > 0:
            self.sampling_rate = rate
            print(f"Sensor {self.sensor_id} sampling rate set to {rate} Hz")
            return True
        return False

    def get_min_value(self):
        """Returns the minimum value of the sensor's range."""
        return self.min_value

    def get_max_value(self):
        """Returns the maximum value of the sensor's range."""
        return self.max_value

    def get_sample_value_percentage(self):
        """Calculates the current value as a percentage of the sensor's range."""
        if self.min_value == self.max_value:
            return 0
        percentage = ((self.current_value - self.min_value) / (self.max_value - self.min_value)) * 100
        return max(0, min(100, int(percentage)))

    def start(self, queue, shutdown_flag):
        """Runs the sensor in a loop, updating values and queuing data."""
        while not shutdown_flag.is_set():
            updated = self.update_value()
            if updated:
                queue.put((self.sensor_id, self.current_value, self.last_sampling_time))
            time.sleep(0.05)

class LMIB:
    """Manages the L-SNMPvS Management Information Base (L-MIB) for device and sensor data."""
    def __init__(self, sensors_dict_ref):
        """Initializes the L-MIB with device data and a reference to sensors."""
        self._sensors_ref = sensors_dict_ref
        self._device_data = {
            11: 1,
            12: "DeviceABC",
            13: "Hub LSV1",
            14: 5,
            15: 0,
            16: int(time.time()),
            17: 0,
            18: 1,
            19: 0,
        }
        self._sensors_data = {}
        self._start_time = time.time()
        self.lock = threading.Lock()
        self.iid_info = {
            11: {"name": "device.lMibId", "type": int, "settable": False},
            12: {"name": "device.id", "type": str, "settable": False},
            13: {"name": "device.type", "type": str, "settable": False},
            14: {"name": "device.beaconRate", "type": int, "settable": True, "min": 1},
            15: {"name": "device.nSensors", "type": int, "settable": False},
            16: {"name": "device.dateAndTime", "type": int, "settable": True},
            17: {"name": "device.upTime", "type": int, "settable": False},
            18: {"name": "device.opStatus", "type": int, "settable": False},
            19: {"name": "device.reset", "type": int, "settable": True},
            21: {"name": "sensors.1.id", "type": str, "settable": False},
            22: {"name": "sensors.1.type", "type": str, "settable": False},
            23: {"name": "sensors.1.sampleValue", "type": int, "settable": False},
            24: {"name": "sensors.1.minValue", "type": int, "settable": False},
            25: {"name": "sensors.1.maxValue", "type": int, "settable": False},
            26: {"name": "sensors.1.lastSamplingTime", "type": int, "settable": False},
            27: {"name": "sensors.1.samplingRate", "type": int, "settable": True, "min": 1},
            31: {"name": "sensors.2.id", "type": str, "settable": False},
            32: {"name": "sensors.2.type", "type": str, "settable": False},
            33: {"name": "sensors.2.sampleValue", "type": int, "settable": False},
            34: {"name": "sensors.2.minValue", "type": int, "settable": False},
            35: {"name": "sensors.2.maxValue", "type": int, "settable": False},
            36: {"name": "sensors.2.lastSamplingTime", "type": int, "settable": False},
            37: {"name": "sensors.2.samplingRate", "type": int, "settable": True, "min": 1},
        }

    def _get_sensor_id_from_iid(self, iid):
        """Maps an IID to its corresponding sensor ID."""
        if 21 <= iid <= 29: return "1"
        if 31 <= iid <= 39: return "2"
        return None

    def add_sensor(self, sensor):
        """Adds a sensor's data to the L-MIB."""
        with self.lock:
            sensor_id_str = sensor.sensor_id
            if sensor_id_str == "1": base_iid = 20
            elif sensor_id_str == "2": base_iid = 30
            else:
                print(f"Warning: Cannot add sensor with unsupported ID '{sensor_id_str}' to MIB IID map.")
                return
            self._sensors_data[base_iid] = {
                base_iid + 1: sensor.sensor_id,
                base_iid + 2: "Simulated",
                base_iid + 3: sensor.get_sample_value_percentage(),
                base_iid + 4: sensor.get_min_value(),
                base_iid + 5: sensor.get_max_value(),
                base_iid + 6: sensor.get_last_sampling_time(),
                base_iid + 7: sensor.get_sampling_rate(),
            }
            self._device_data[15] = len(self._sensors_data)
            print(f"Sensor {sensor_id_str} added to MIB. nSensors = {self._device_data[15]}")

    def update_sensor_mib_data(self, sensor_id, raw_value, timestamp):
        """Updates sensor data in the L-MIB."""
        with self.lock:
            if sensor_id == "1": base_iid = 20
            elif sensor_id == "2": base_iid = 30
            else: return
            if base_iid in self._sensors_data:
                sensor_obj = self._sensors_ref.get(sensor_id)
                if sensor_obj:
                    self._sensors_data[base_iid][base_iid + 3] = sensor_obj.get_sample_value_percentage()
                    self._sensors_data[base_iid][base_iid + 6] = int(timestamp)

    def get_data(self, iid):
        """Retrieves data for a given IID from the L-MIB."""
        with self.lock:
            if iid in self._device_data:
                if iid == 17:
                    self._device_data[iid] = int(time.time() - self._start_time)
                return self._device_data.get(iid)
            else:
                for base_iid, sensor_dict in self._sensors_data.items():
                    if iid in sensor_dict:
                        return sensor_dict.get(iid)
            return None

    def set_data(self, iid, value):
        """Sets data for a given IID in the L-MIB, validating type and constraints."""
        with self.lock:
            if iid not in self.iid_info:
                print(f"SET Error: Unknown IID {iid}")
                return 4
            info = self.iid_info[iid]
            expected_type = info["type"]
            is_settable = info["settable"]
            if not is_settable:
                print(f"SET Error: IID {iid} ({info['name']}) is not settable.")
                return 3
            try:
                if expected_type == int:
                    if not isinstance(value, int):
                        try:
                            value = int(value)
                        except (ValueError, TypeError):
                            raise ValueError(f"Expected integer for IID {iid}")
                elif expected_type == str:
                    if not isinstance(value, str):
                        try:
                            value = str(value)
                        except (ValueError, TypeError):
                            raise ValueError(f"Expected string for IID {iid}")
            except ValueError as e:
                print(f"SET Error: Type mismatch for IID {iid}. Value: '{value}'. Error: {e}")
                return 2
            min_val = info.get("min")
            if min_val is not None and value < min_val:
                print(f"SET Error: Value {value} for IID {iid} is below minimum {min_val}")
                return 2
            if iid in self._device_data:
                self._device_data[iid] = value
                print(f"MIB SET: {info['name']} (IID {iid}) = {value}")
                if iid == 19 and value == 1:
                    self.reset_device()
                    self._device_data[iid] = 0
            else:
                sensor_id_str = self._get_sensor_id_from_iid(iid)
                sensor_obj = self._sensors_ref.get(sensor_id_str)
                if not sensor_obj:
                    print(f"SET Error: Sensor object '{sensor_id_str}' not found for IID {iid}")
                    return 5
                if iid % 10 == 7:
                    if sensor_obj.set_sampling_rate(value):
                        base_iid = (iid // 10) * 10
                        if base_iid in self._sensors_data:
                            self._sensors_data[base_iid][iid] = value
                            print(f"MIB SET: {info['name']} (IID {iid}) = {value}")
                        else:
                            print(f"SET Warning: MIB cache inconsistency for sensor {sensor_id_str}")
                            return 5
                    else:
                        print(f"SET Error: Failed to set sampling rate on sensor object {sensor_id_str}")
                        return 2
                else:
                    print(f"SET Error: Attempt to set unhandled sensor IID {iid}")
                    return 3
            return 0

    def reset_device(self):
        """Resets the device, updating uptime and timestamp."""
        print("--- Device Reset Triggered ---")
        self._start_time = time.time()
        self._device_data[17] = 0
        self._device_data[16] = int(self._start_time)
        print("--- Device Reset Complete ---")

class AgenteLSNMPvS:
    """Implements the L-SNMPvS agent, handling UDP requests and managing sensors."""
    def __init__(self, endereco_agente, porta_agente, secrets_file="agent_secrets.json"):
        """Initializes the agent with address, port, and configuration."""
        self.endereco_agente = endereco_agente
        self.porta_agente = porta_agente
        self.sensors = {}
        self.mib = LMIB(self.sensors)
        self.queue = Queue()
        self.lock = threading.Lock()
        self.msg_id_counter = random.randint(0, 1000)
        self.shutdown_flag = threading.Event()
        self.secrets = load_secrets(secrets_file)
        if not self.secrets:
            print("Warning: Using default secrets due to load failure.")
            self.secrets = {
                "agent_id": "AGENT001",
                "KA": "agent_secret_key_32_bytes_1234567890",
                "monitors": {"MONITOR1": "monitor_secret_key_32_bytes_1234567890"}
            }
        self.agent_id = self.secrets.get("agent_id", "AGENT001")
        self.KA = self.secrets.get("KA", "").encode('utf-8')
        self.monitor_keys = self.secrets.get("monitors", {})

    def add_sensor(self, sensor):
        """Adds a sensor to the agent and its L-MIB."""
        with self.lock:
            sensor_id_str = str(sensor.sensor_id)
            self.sensors[sensor_id_str] = sensor
            self.mib.add_sensor(sensor)

    def get_next_msg_id(self):
        """Generates a unique message ID for PDU responses."""
        with self.lock:
            self.msg_id_counter += 1
            return self.msg_id_counter % (2**32)

    def derive_key(self, receiver_id):
        """Derives a session key for a given receiver ID."""
        if receiver_id == b"\x00" * 8:
            return None, None
        KM = self.monitor_keys.get(receiver_id.decode('ascii'), "").encode('utf-8')
        if not KM:
            print(f"No KM found for receiver {receiver_id.decode('ascii')}")
            return None, None
        key = hashes.Hash(hashes.SHA256(), backend=default_backend())
        key.update(self.KA + KM)
        return key.finalize(), KM

    def encode_pdu(self, tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list, sender_id, receiver_id):
        """Encodes a PDU with AES-256-CBC encryption and HMAC-SHA-256 authentication."""
        try:
            core_pdu = b""
            core_pdu += tag.encode('ascii')
            core_pdu += struct.pack("!B", msg_type)
            core_pdu += struct.pack("!I", timestamp)
            core_pdu += struct.pack("!I", msg_id)
            core_pdu += b"".join(struct.pack("!H", iid) for iid in iid_list)
            for value in v_list:
                if isinstance(value, int):
                    core_pdu += struct.pack("!i", value)
                elif isinstance(value, str):
                    encoded_value = value.encode('utf-8')
                    core_pdu += struct.pack("!H", len(encoded_value)) + encoded_value
                elif isinstance(value, bytes):
                    core_pdu += struct.pack("!H", len(value)) + value
                else:
                    print(f"Encoding Error: Unsupported value type {type(value)}: {value}")
                    core_pdu += struct.pack("!H", 0)
            core_pdu += b"".join(struct.pack("!I", time_val) for time_val in t_list)
            core_pdu += b"".join(struct.pack("!H", error) for error in e_list)
            sender_id_bytes = sender_id.encode('ascii').ljust(8)[:8]
            receiver_id_bytes = receiver_id.ljust(8)[:8]
            sec_model_id = 0 if receiver_id == b"\x00" * 8 else 128
            if sec_model_id == 0:
                return sender_id_bytes + receiver_id_bytes + struct.pack("!B", 0) + struct.pack("!I", len(core_pdu)) + core_pdu
            key, KM = self.derive_key(receiver_id)
            if not key:
                print("Key derivation failed, cannot encode secured PDU")
                return None
            iv = os.urandom(16)
            cipher = Cipher(algorithms.AES(key), modes.CBC(iv), backend=default_backend())
            encryptor = cipher.encryptor()
            padding_len = 16 - (len(core_pdu) % 16)
            core_pdu_padded = core_pdu + bytes([padding_len] * padding_len)
            encrypted_pdu = encryptor.update(core_pdu_padded) + encryptor.finalize()
            hmac_obj = hmac.HMAC(key, hashes.SHA256(), backend=default_backend())
            hmac_obj.update(sender_id_bytes + receiver_id_bytes + struct.pack("!B", sec_model_id) + struct.pack("!I", len(iv + encrypted_pdu)) + iv + encrypted_pdu)
            hmac_value = hmac_obj.finalize()
            sec_data = iv + encrypted_pdu + hmac_value
            return sender_id_bytes + receiver_id_bytes + struct.pack("!B", sec_model_id) + struct.pack("!I", len(sec_data)) + sec_data
        except Exception as e:
            print(f"PDU Encoding failed: {e}")
            traceback.print_exc()
            return None

    def decode_pdu(self, data):
        """Decodes a PDU, verifying HMAC and decrypting with AES-256-CBC."""
        try:
            if len(data) < 21:
                raise ValueError("Message too short")
            sender_id = data[0:8]
            receiver_id = data[8:16]
            sec_model_id = struct.unpack("!B", data[16:17])[0]
            sec_data_size = struct.unpack("!I", data[17:21])[0]
            if len(data) < 21 + sec_data_size:
                raise ValueError("Sec-Data size mismatch")
            sec_data = data[21:21+sec_data_size]
            if sec_model_id == 0:
                tag = sec_data[0:1].decode('ascii')
                msg_type = struct.unpack("!B", sec_data[1:2])[0]
                timestamp = struct.unpack("!I", sec_data[2:6])[0]
                msg_id = struct.unpack("!I", sec_data[6:10])[0]
                remaining_data = sec_data[10:]
                return tag, msg_type, timestamp, msg_id, remaining_data, sender_id, receiver_id
            key, KM = self.derive_key(sender_id)
            if not key:
                print(f"Authentication failed: No key for sender {sender_id.decode('ascii')}")
                return None, None, None, None, None, sender_id, receiver_id
            if len(sec_data) < 48:
                raise ValueError("Sec-Data too short for IV+PDU+HMAC")
            iv = sec_data[:16]
            encrypted_pdu = sec_data[16:-32]
            received_hmac = sec_data[-32:]
            hmac_obj = hmac.HMAC(key, hashes.SHA256(), backend=default_backend())
            hmac_obj.update(sender_id + receiver_id + struct.pack("!B", sec_model_id) + struct.pack("!I", len(iv + encrypted_pdu)) + iv + encrypted_pdu)
            hmac_obj.verify(received_hmac)
            cipher = Cipher(algorithms.AES(key), modes.CBC(iv), backend=default_backend())
            decryptor = cipher.decryptor()
            decrypted_padded = decryptor.update(encrypted_pdu) + decryptor.finalize()
            padding_len = decrypted_padded[-1]
            core_pdu = decrypted_padded[:-padding_len]
            tag = core_pdu[0:1].decode('ascii')
            msg_type = struct.unpack("!B", core_pdu[1:2])[0]
            timestamp = struct.unpack("!I", core_pdu[2:6])[0]
            msg_id = struct.unpack("!I", core_pdu[6:10])[0]
            remaining_data = core_pdu[10:]
            return tag, msg_type, timestamp, msg_id, remaining_data, sender_id, receiver_id
        except Exception as e:
            print(f"PDU Decoding failed: {e}")
            traceback.print_exc()
            return None, None, None, None, None, sender_id, receiver_id

    def handle_request(self, data, addr, server_socket):
        """Processes incoming UDP requests (GET, SET, BEACON)."""
        print(f"Received {len(data)} bytes from {addr}")
        decoded = self.decode_pdu(data)
        if decoded[0] is None:
            print(f"Failed to decode PDU from {addr}")
            return
        tag, msg_type, timestamp, req_msg_id, pdu_payload, sender_id, receiver_id = decoded
        resp_msg_id = self.get_next_msg_id()
        print(f"Request Decoded: Tag={tag}, Type={msg_type}, MsgID={req_msg_id}, Sender={sender_id.decode('ascii')}")
        if receiver_id != self.agent_id.encode('ascii') and receiver_id != b"\x00" * 8:
            print(f"Ignoring request not intended for this agent (Receiver ID: {receiver_id.decode('ascii')})")
            return
        resp_pdu = None
        if tag == 'G':
            req_iids = []
            offset = 0
            try:
                while offset + 2 <= len(pdu_payload):
                    iid = struct.unpack("!H", pdu_payload[offset:offset+2])[0]
                    req_iids.append(iid)
                    offset += 2
                print(f"GET Request IIDs: {req_iids}")
                resp_values = self.handle_get(req_iids)
                resp_pdu = self.encode_pdu('R', 1, int(time.time()), resp_msg_id, req_iids, resp_values, [], [], self.agent_id, sender_id)
            except struct.error as e:
                print(f"Error parsing IIDs: {e}")
                return
        elif tag == 'S':
            parsed_iids = []
            value_payload_for_set = b''
            offset = 0
            if len(pdu_payload) >= 2:
                try:
                    iid = struct.unpack("!H", pdu_payload[offset:offset+2])[0]
                    parsed_iids.append(iid)
                    offset += 2
                    value_payload_for_set = pdu_payload[offset:]
                    print(f"SET Request IIDs: {parsed_iids}, Value Payload Length: {len(value_payload_for_set)}")
                    error_codes = self.handle_set(parsed_iids, value_payload_for_set)
                    resp_pdu = self.encode_pdu('R', 1, int(time.time()), resp_msg_id, parsed_iids, [], [], error_codes, self.agent_id, sender_id)
                except struct.error as e:
                    print(f"Error parsing IID: {e}")
                    return
            else:
                print(f"SET payload too short: {pdu_payload.hex()}")
                return
        elif tag == 'B':
            self.send_beacon_response(server_socket, addr)
            return
        else:
            print(f"Unknown PDU tag '{tag}' from {addr}")
            return
        if resp_pdu:
            try:
                server_socket.sendto(resp_pdu, addr)
                print(f"Sent response (Tag R, MsgID {resp_msg_id}) to {addr}")
            except Exception as e:
                print(f"Error sending response: {e}")

    def handle_get(self, iid_list):
        """Handles GET requests by retrieving values for specified IIDs."""
        response_values = []
        for iid in iid_list:
            value = self.mib.get_data(iid)
            if value is None:
                print(f"GET: IID {iid} not found.")
                info = self.mib.iid_info.get(iid)
                if info and info["type"] == int:
                    value = 0
                elif info and info["type"] == str:
                    value = ""
                else:
                    value = b""
            elif isinstance(value, str):
                value = value.encode('utf-8')
            response_values.append(value)
        return response_values

    def handle_set(self, iid_list, value_data):
        """Handles SET requests by updating L-MIB with provided values."""
        error_codes = []
        offset = 0
        for i, iid in enumerate(iid_list):
            info = self.mib.iid_info.get(iid)
            expected_type = info["type"] if info else None
            value = None
            parse_error = False
            try:
                if expected_type == int:
                    if offset + 4 > len(value_data):
                        raise ValueError("Not enough data for int")
                    value = struct.unpack("!i", value_data[offset:offset+4])[0]
                    offset += 4
                elif expected_type == str:
                    if offset + 2 > len(value_data):
                        raise ValueError("Not enough data for str length")
                    str_len = struct.unpack("!H", value_data[offset:offset+2])[0]
                    offset += 2
                    if offset + str_len > len(value_data):
                        raise ValueError("Not enough data for str content")
                    value = value_data[offset:offset+str_len].decode('utf-8')
                    offset += str_len
                else:
                    print(f"SET Parse Error: Unknown type for IID {iid}")
                    parse_error = True
                    break
            except (struct.error, ValueError, UnicodeDecodeError) as e:
                print(f"SET Parse Error: IID {iid}: {e}")
                parse_error = True
                break
            if not parse_error:
                error_code = self.mib.set_data(iid, value)
                error_codes.append(error_code)
            else:
                error_codes.append(1)
        while len(error_codes) < len(iid_list):
            error_codes.append(1)
        print(f"SET Result: IIDs={iid_list}, Errors={error_codes}")
        return error_codes

    def send_beacon_response(self, sock, addr):
        """Sends a beacon response with device information."""
        beacon_iids = [11, 12, 13, 15, 16, 17, 18]
        beacon_values = self.handle_get(beacon_iids)
        resp_msg_id = self.get_next_msg_id()
        beacon_pdu = self.encode_pdu('B', 1, int(time.time()), resp_msg_id, beacon_iids, beacon_values, [], [], self.agent_id, b"\x00" * 8)
        if beacon_pdu:
            try:
                sock.sendto(beacon_pdu, addr)
                print(f"Sent Beacon Response (MsgID {resp_msg_id}) to {addr}")
            except Exception as e:
                print(f"Error sending beacon: {e}")

    def send_periodic_beacon(self, shutdown_flag):
        """Sends periodic beacon PDUs to broadcast device information."""
        print("Starting periodic beacon sender...")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as beacon_socket:
            beacon_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            broadcast_addr = ('<broadcast>', self.porta_agente)
            while not shutdown_flag.wait(0):
                try:
                    beacon_rate = self.mib.get_data(14)
                    if not isinstance(beacon_rate, int) or beacon_rate < 1:
                        print("Warning: Invalid beacon rate, defaulting to 5s.")
                        beacon_rate = 5
                    beacon_iids = [11, 12, 13, 15, 16, 17, 18]
                    beacon_values = self.handle_get(beacon_iids)
                    beacon_msg_id = self.get_next_msg_id()
                    beacon_pdu = self.encode_pdu('B', 1, int(time.time()), beacon_msg_id, beacon_iids, beacon_values, [], [], self.agent_id, b"\x00" * 8)
                    if beacon_pdu:
                        beacon_socket.sendto(beacon_pdu, broadcast_addr)
                    else:
                        print("Error encoding beacon PDU.")
                    shutdown_flag.wait(beacon_rate)
                except Exception as e:
                    print(f"Error in beacon thread: {e}")
                    shutdown_flag.wait(5)
        print("Periodic beacon sender stopped.")

    def start_server(self, shutdown_flag):
        """Runs the UDP server to handle incoming requests."""
        print(f"L-SNMPvS Agent starting on {self.endereco_agente}:{self.porta_agente}")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            try:
                server_socket.bind((self.endereco_agente, self.porta_agente))
                server_socket.settimeout(1.0)
            except OSError as e:
                print(f"FATAL: Could not bind: {e}")
                shutdown_flag.set()
                return
            print("Agent listening...")
            while not shutdown_flag.is_set():
                try:
                    data, addr = server_socket.recvfrom(2048)
                    self.handle_request(data, addr, server_socket)
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error in server loop: {e}")

    def update_mib_from_queue(self, shutdown_flag):
        """Updates the L-MIB with sensor data from the queue."""
        print("Starting MIB update thread...")
        while not shutdown_flag.is_set():
            try:
                sensor_id, value, timestamp = self.queue.get(timeout=0.5)
                self.mib.update_sensor_mib_data(sensor_id, value, timestamp)
                self.queue.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Error in MIB update thread: {e}")
        print("MIB update thread stopped.")

    def start_sensors(self, shutdown_flag):
        """Starts threads for all sensors to generate data."""
        print("Starting sensor threads...")
        sensor_threads = []
        if not self.sensors:
            print("Warning: No sensors added.")
            return
        for sensor in self.sensors.values():
            thread = threading.Thread(target=sensor.start, args=(self.queue, shutdown_flag), daemon=True)
            sensor_threads.append(thread)
            thread.start()
        print(f"{len(sensor_threads)} sensor threads started.")

    def run(self):
        """Starts the agent, initializing sensors, MIB updates, and server."""
        print("--- Starting L-SNMPvS Agent ---")
        all_threads = []
        self.start_sensors(self.shutdown_flag)
        mib_update_thread = threading.Thread(target=self.update_mib_from_queue, args=(self.shutdown_flag,), daemon=True)
        all_threads.append(mib_update_thread)
        mib_update_thread.start()
        beacon_thread = threading.Thread(target=self.send_periodic_beacon, args=(self.shutdown_flag,), daemon=True)
        all_threads.append(beacon_thread)
        beacon_thread.start()
        try:
            self.start_server(self.shutdown_flag)
        except KeyboardInterrupt:
            print("\nShutdown signal received...")
        except Exception as e:
            print(f"\nFATAL error: {e}")
        finally:
            print("Signalling shutdown...")
            self.shutdown_flag.set()
            time.sleep(1.5)
            print("--- Agent Shutdown Complete ---")

if __name__ == "__main__":
    """Initializes and runs the L-SNMPvS agent with two virtual sensors."""
    from queue import Empty
    agent = AgenteLSNMPvS("0.0.0.0", 12345)
    sensor1 = VirtualSensor("1", 0, 100, update_interval=1, precision=1)
    sensor2 = VirtualSensor("2", -20, 50, update_interval=0.5, precision=2)
    agent.add_sensor(sensor1)
    agent.add_sensor(sensor2)
    agent.run()
