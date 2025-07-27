import socket
import struct
import time
import random
import traceback
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import hashes, hmac
from cryptography.hazmat.backends import default_backend
import json
import os

# Master key for decrypting configuration files, must match agent and encrypt_config scripts
MASTER_KEY = b"master_key_32_bytes_1234567890ab"

def load_secrets(file_path):
    """Decrypts and loads the monitor's configuration from a JSON file."""
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
        if not secrets.get("monitor_id") or not secrets.get("KM") or not secrets.get("agent_id") or not secrets.get("KA"):
            print(f"Error: Missing required fields in {file_path}.")
            return None
        return secrets
    except Exception as e:
        print(f"Error loading secrets from {file_path}: {e}")
        traceback.print_exc()
        return None

class LSNMPvSMonitor:
    """Implements the L-SNMPvS client for communicating with an IoT agent over UDP."""
    def __init__(self, agent_address, agent_port=12345, timeout=2, monitor_id="MONITOR1", secrets_file="monitor_secrets.json"):
        """Initializes the monitor with agent connection details and loads secrets."""
        self.agent_address = agent_address
        self.agent_port = agent_port
        self.timeout = timeout
        self.msg_id_counter = random.randint(0, 1000)
        self.monitor_id = monitor_id
        self.secrets = load_secrets(secrets_file)
        if not self.secrets:
            print("Warning: Using default secrets due to load failure.")
            self.secrets = {
                "monitor_id": "MONITOR1",
                "KM": "monitor_secret_key_32_bytes_1234567890",
                "agent_id": "AGENT001",
                "KA": "agent_secret_key_32_bytes_1234567890"
            }
        self.KM = self.secrets.get("KM", "").encode('utf-8')
        self.agent_id = self.secrets.get("agent_id", "AGENT001")
        self.KA = self.secrets.get("KA", "").encode('utf-8')
        self.iid_map = {
            "device.lMibId": 11,
            "device.id": 12,
            "device.type": 13,
            "device.beaconRate": 14,
            "device.nSensors": 15,
            "device.dateAndTime": 16,
            "device.upTime": 17,
            "device.opStatus": 18,
            "device.reset": 19,
            "sensors.1.id": 21,
            "sensors.1.type": 22,
            "sensors.1.sampleValue": 23,
            "sensors.1.minValue": 24,
            "sensors.1.maxValue": 25,
            "sensors.1.lastSamplingTime": 26,
            "sensors.1.samplingRate": 27,
            "sensors.2.id": 31,
            "sensors.2.type": 32,
            "sensors.2.sampleValue": 33,
            "sensors.2.minValue": 34,
            "sensors.2.maxValue": 35,
            "sensors.2.lastSamplingTime": 36,
            "sensors.2.samplingRate": 37,
        }
        self.name_map = {v: k for k, v in self.iid_map.items()}

    def _get_next_msg_id(self):
        """Generates a unique message ID for PDU requests."""
        self.msg_id_counter += 1
        return self.msg_id_counter % (2**32)

    def derive_key(self):
        """Derives a session key by hashing agent and monitor keys."""
        key = hashes.Hash(hashes.SHA256(), backend=default_backend())
        key.update(self.KA + self.KM)
        return key.finalize()

    def encode_pdu(self, tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list):
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
                    raise ValueError(f"Unsupported value type: {type(value)}")
            core_pdu += b"".join(struct.pack("!I", time_val) for time_val in t_list)
            core_pdu += b"".join(struct.pack("!H", error) for error in e_list)
            sender_id_bytes = self.monitor_id.encode('ascii').ljust(8)[:8]
            receiver_id_bytes = self.agent_id.encode('ascii').ljust(8)[:8]
            sec_model_id = 128
            key = self.derive_key()
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
            print(f"Error encoding PDU: {e}")
            traceback.print_exc()
            return None

    def decode_pdu(self, data):
        """Decodes a PDU, verifying HMAC and decrypting with AES-256-CBC."""
        if not data:
            raise ValueError("Cannot decode empty data")
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
            return tag, msg_type, timestamp, msg_id, remaining_data
        else:
            if len(sec_data) < 48:
                raise ValueError("Sec-Data too short for IV+PDU+HMAC")
            iv = sec_data[:16]
            encrypted_pdu = sec_data[16:-32]
            received_hmac = sec_data[-32:]
            key = self.derive_key()
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
            return tag, msg_type, timestamp, msg_id, remaining_data

    def _parse_get_response_data(self, remaining_data, expected_iids):
        """Parses GET response data, extracting IIDs and their values."""
        offset = 0
        iid_list = []
        v_list = []
        t_list = []
        e_list = []
        num_expected_iids = len(expected_iids)
        for i in range(num_expected_iids):
            if offset + 2 > len(remaining_data):
                print(f"Warning: PDU data ended prematurely while expecting IID {i+1}/{num_expected_iids}.")
                break
            try:
                iid = struct.unpack("!H", remaining_data[offset:offset+2])[0]
                iid_list.append(iid)
                offset += 2
            except struct.error as e:
                print(f"Warning: Struct error unpacking IID {i+1}/{num_expected_iids}: {e}")
                break
        num_parsed_iids = len(iid_list)
        for i in range(num_parsed_iids):
            iid = iid_list[i]
            object_name = self.name_map.get(iid, f"Unknown IID {iid}")
            is_byte_type = iid in [12, 13, 21, 22, 31, 32]
            try:
                if is_byte_type:
                    if offset + 2 > len(remaining_data):
                        raise ValueError("short data for str/bytes length")
                    str_len = struct.unpack("!H", remaining_data[offset:offset+2])[0]
                    offset += 2
                    if offset + str_len > len(remaining_data):
                        raise ValueError("short data for str/bytes content")
                    value_bytes = remaining_data[offset:offset+str_len]
                    try:
                        value = value_bytes.decode('utf-8')
                    except UnicodeDecodeError:
                        value = value_bytes
                    v_list.append(value)
                    offset += str_len
                else:
                    if offset + 4 > len(remaining_data):
                        raise ValueError("short data for integer")
                    value = struct.unpack("!i", remaining_data[offset:offset+4])[0]
                    v_list.append(value)
                    offset += 4
            except (struct.error, ValueError) as e:
                print(f"Warning: Error parsing value for IID {iid} ({object_name}): {e}")
                v_list.append(None)
                break
        while len(v_list) < len(iid_list):
            v_list.append(None)
        return iid_list, v_list, t_list, e_list

    def _parse_set_response_data(self, remaining_data, expected_iids):
        """Parses SET response data, extracting IIDs and error codes."""
        offset = 0
        iid_list = []
        v_list = []
        t_list = []
        e_list = []
        num_expected_iids = len(expected_iids)
        for i in range(num_expected_iids):
            if offset + 2 > len(remaining_data):
                print(f"Warning: SET Response data ended prematurely while expecting IID {i+1}/{num_expected_iids}.")
                break
            try:
                iid = struct.unpack("!H", remaining_data[offset:offset+2])[0]
                iid_list.append(iid)
                offset += 2
            except struct.error as e:
                print(f"Warning: Struct error unpacking IID {i+1}/{num_expected_iids}: {e}")
                break
        num_parsed_iids = len(iid_list)
        for i in range(num_parsed_iids):
            if offset + 2 > len(remaining_data):
                print(f"Warning: SET Response data ended prematurely while expecting Error Code {i+1}/{num_parsed_iids}.")
                break
            try:
                error_code = struct.unpack("!H", remaining_data[offset:offset+2])[0]
                e_list.append(error_code)
                offset += 2
            except struct.error as e:
                print(f"Warning: Struct error unpacking Error Code {i+1}/{num_parsed_iids}: {e}")
                e_list.append(99)
                break
        while len(e_list) < len(iid_list):
            e_list.append(1)
        return iid_list, v_list, t_list, e_list

    def get(self, object_name):
        """Fetches a single value from the agent for a given object name."""
        if object_name not in self.iid_map:
            print(f"Error: Unknown object name: {object_name}")
            return None
        iid = self.iid_map[object_name]
        req_iids = [iid]
        req_msg_id = self._get_next_msg_id()
        req_pdu = self.encode_pdu('G', 1, int(time.time()), req_msg_id, req_iids, [], [], [])
        if not req_pdu:
            return None
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout)
            try:
                sock.sendto(req_pdu, (self.agent_address, self.agent_port))
                data, addr = sock.recvfrom(1024)
                decoded = self.decode_pdu(data)
                if decoded[0] is None:
                    return None
                tag, msg_type, timestamp, resp_msg_id, remaining_data = decoded
                if tag != 'R':
                    print(f"Error: Unexpected tag '{tag}' in GET response")
                    return None
                resp_iids, resp_values, _, _ = self._parse_get_response_data(remaining_data, req_iids)
                try:
                    value_index = resp_iids.index(iid)
                    if value_index < len(resp_values):
                        return resp_values[value_index]
                    else:
                        print(f"Error: Value missing for IID {iid}")
                        return None
                except (ValueError, IndexError):
                    print(f"Error: IID {iid} not found in response")
                    return None
            except socket.timeout:
                print(f"Error: Request timed out for {object_name}")
                return None
            except (ValueError, struct.error) as e:
                print(f"Error processing GET response: {e}")
                return None
            except Exception as e:
                print(f"Unexpected error during GET: {e}")
                return None

    def get_bulk(self, object_names):
        """Fetches multiple values from the agent in a single request."""
        req_iids = []
        valid_object_names = []
        for name in object_names:
            if name in self.iid_map:
                req_iids.append(self.iid_map[name])
                valid_object_names.append(name)
            else:
                print(f"Warning: Unknown object name '{name}' skipped")
        if not req_iids:
            return {}
        req_msg_id = self._get_next_msg_id()
        req_pdu = self.encode_pdu('G', 1, int(time.time()), req_msg_id, req_iids, [], [], [])
        if not req_pdu:
            return {name: None for name in valid_object_names}
        results = {name: None for name in valid_object_names}
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout)
            try:
                sock.sendto(req_pdu, (self.agent_address, self.agent_port))
                data, addr = sock.recvfrom(2048)
                decoded = self.decode_pdu(data)
                if decoded[0] is None:
                    return results
                tag, msg_type, timestamp, resp_msg_id, remaining_data = decoded
                if tag != 'R':
                    print(f"Error: Unexpected tag '{tag}' in bulk response")
                    return results
                resp_iids, resp_values, _, _ = self._parse_get_response_data(remaining_data, req_iids)
                for i, returned_iid in enumerate(resp_iids):
                    if i < len(resp_values):
                        returned_name = self.name_map.get(returned_iid)
                        if returned_name in results:
                            results[returned_name] = resp_values[i]
                        else:
                            print(f"Warning: Unexpected IID {returned_iid}")
                    else:
                        returned_name = self.name_map.get(returned_iid, f"Unknown IID {returned_iid}")
                        print(f"Warning: Missing value for IID {returned_iid}")
                if len(resp_iids) != len(req_iids) or len(resp_values) != len(req_iids):
                    print(f"Warning: Mismatch in expected vs received items")
                return results
            except socket.timeout:
                print(f"Error: Bulk get timed out")
                return results
            except (ValueError, struct.error) as e:
                print(f"Error processing bulk GET: {e}")
                return results
            except Exception as e:
                print(f"Unexpected error during bulk GET: {e}")
                return results

    def set(self, object_name, value):
        """Sets a value for a specified object on the agent."""
        if object_name not in self.iid_map:
            print(f"Error: Unknown object name for SET: {object_name}")
            return False
        iid = self.iid_map[object_name]
        req_iids = [iid]
        req_values = [value]
        req_msg_id = self._get_next_msg_id()
        if iid in [14, 16, 19, 27, 37]:
            if not isinstance(value, int):
                try:
                    req_values = [int(value)]
                except (ValueError, TypeError):
                    print(f"Error: Invalid value type for {object_name}")
                    return False
        req_pdu = self.encode_pdu('S', 1, int(time.time()), req_msg_id, req_iids, req_values, [], [])
        if not req_pdu:
            return False
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout)
            try:
                sock.sendto(req_pdu, (self.agent_address, self.agent_port))
                data, addr = sock.recvfrom(1024)
                decoded = self.decode_pdu(data)
                if decoded[0] is None:
                    return False
                tag, msg_type, timestamp, resp_msg_id, remaining_data = decoded
                if tag != 'R':
                    print(f"Error: Unexpected tag '{tag}' for SET response")
                    return False
                resp_iids, _, _, resp_errors = self._parse_set_response_data(remaining_data, req_iids)
                try:
                    idx = resp_iids.index(iid)
                    if idx < len(resp_errors):
                        error_code = resp_errors[idx]
                        if error_code == 0:
                            print(f"SET successful for {object_name} (IID {iid})")
                            return True
                        else:
                            error_map = {1: "General Error", 2: "Bad Value", 3: "Not Settable", 4: "Unknown IID", 5: "Internal Error"}
                            print(f"SET failed for {object_name}: Error {error_code} ({error_map.get(error_code, 'Unknown')})")
                            return False
                    else:
                        print(f"SET failed: Error code missing")
                        return False
                except (ValueError, IndexError):
                    print(f"SET failed: IID {iid} not found in response")
                    return False
            except socket.timeout:
                print(f"Error: SET timed out for {object_name}")
                return False
            except (ValueError, struct.error) as e:
                print(f"Error processing SET response: {e}")
                return False
            except Exception as e:
                print(f"Unexpected error during SET: {e}")
                return False

    def monitor_device_status(self, interval=5):
        """Continuously displays the agent's status and sensor data at specified intervals."""
        try:
            while True:
                print(f"\n--- Device Status ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---")
                device_info = self.get_bulk([
                    'device.id', 'device.type', 'device.nSensors', 'device.upTime',
                    'device.opStatus', 'device.beaconRate'
                ])
                if device_info is None:
                    print("Failed to retrieve device information")
                    time.sleep(interval)
                    continue
                dev_id = device_info.get('device.id', 'N/A')
                dev_type = device_info.get('device.type', 'N/A')
                num_sensors_val = device_info.get('device.nSensors')
                num_sensors_str = str(num_sensors_val) if num_sensors_val is not None else 'N/A'
                uptime = device_info.get('device.upTime', 'N/A')
                op_status_val = device_info.get('device.opStatus')
                op_status = 'Normal' if op_status_val == 1 else ('Abnormal' if op_status_val is not None else 'N/A')
                beacon_rate = device_info.get('device.beaconRate', 'N/A')
                print(f"Device ID: {dev_id}")
                print(f"Type: {dev_type}")
                print(f"Sensors: {num_sensors_str}")
                print(f"Uptime: {uptime} seconds")
                print(f"Status: {op_status}")
                print(f"Beacon Rate: {beacon_rate} s")
                num_sensors = 0
                if isinstance(num_sensors_val, int) and num_sensors_val > 0:
                    num_sensors = num_sensors_val
                elif num_sensors_val is not None:
                    print(f"Warning: Unexpected value for device.nSensors: {num_sensors_val}")
                for sensor_num in range(1, num_sensors + 1):
                    sensor_id_name = f'sensors.{sensor_num}.id'
                    if sensor_id_name not in self.iid_map:
                        print(f"\n--- Sensor {sensor_num} ---")
                        print(f"Sensor {sensor_num} definitions not found")
                        continue
                    sensor_oids_to_get = [
                        f'sensors.{sensor_num}.sampleValue',
                        f'sensors.{sensor_num}.lastSamplingTime',
                        f'sensors.{sensor_num}.samplingRate'
                    ]
                    sensor_info = self.get_bulk(sensor_oids_to_get)
                    print(f"\n--- Sensor {sensor_num} ---")
                    if sensor_info:
                        sample_val = sensor_info.get(f'sensors.{sensor_num}.sampleValue', 'N/A')
                        last_sample_ts = sensor_info.get(f'sensors.{sensor_num}.lastSamplingTime')
                        sampling_rate = sensor_info.get(f'sensors.{sensor_num}.samplingRate', 'N/A')
                        last_sample_str = 'N/A'
                        if isinstance(last_sample_ts, int):
                            try:
                                last_sample_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(last_sample_ts))
                            except (OverflowError, OSError):
                                last_sample_str = f"Invalid Timestamp ({last_sample_ts})"
                        elif last_sample_ts is not None:
                            last_sample_str = f"Non-integer Timestamp ({last_sample_ts})"
                        print(f"Sample Value: {sample_val}%")
                        print(f"Last Sampling: {last_sample_str}")
                        print(f"Sampling Rate: {sampling_rate} Hz")
                    else:
                        print(f"Could not retrieve sensor {sensor_num} information")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user.")
        except Exception as e:
            print(f"\nError during monitoring: {e}")
            traceback.print_exc()

if __name__ == "__main__":
    """Initializes and runs the monitor in continuous status display mode."""
    monitor = LSNMPvSMonitor('127.0.0.1', 12345)
    monitor.monitor_device_status(interval=10)
