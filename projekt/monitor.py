import socket
import struct
import time
import random
import traceback # For detailed error logging

class LSNMPvSMonitor:
    def __init__(self, agent_address, agent_port=12345, timeout=2):
        self.agent_address = agent_address
        self.agent_port = agent_port
        self.timeout = timeout
        self.msg_id_counter = random.randint(0, 1000) # Start with a random offset

        # IID mapping (matching agent.py handle_get/handle_set)
        self.iid_map = {
            "device.lMibId": 11,
            "device.id": 12,
            "device.type": 13,
            "device.beaconRate": 14,
            "device.nSensors": 15,
            "device.dateAndTime": 16,
            "device.upTime": 17,
            "device.opStatus": 18,
            "device.reset": 19, # Note: Reset is often triggered via SET
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
        # Reverse map for easier lookup in responses
        self.name_map = {v: k for k, v in self.iid_map.items()}

    def _get_next_msg_id(self):
        self.msg_id_counter += 1
        return self.msg_id_counter % (2**32) # Keep it within unsigned 32-bit range

    # --- PDU Encoding/Decoding ---
    def encode_pdu(self, tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list):
        """Encodes the PDU into a byte string."""
        try:
            tag_bytes = tag.encode('ascii') # Ensure ASCII encoding
            type_bytes = struct.pack("!B", msg_type)
            timestamp_bytes = struct.pack("!I", timestamp)
            msg_id_bytes = struct.pack("!I", msg_id)

            # Pack IIDs
            iid_list_bytes = b"".join(struct.pack("!H", iid) for iid in iid_list)

            # Pack Values (handle int and str/bytes)
            v_list_bytes = b""
            for value in v_list:
                if isinstance(value, int):
                    v_list_bytes += struct.pack("!i", value) # Signed Int for values
                elif isinstance(value, str):
                    encoded_value = value.encode('utf-8') # Encode string to bytes
                    v_list_bytes += struct.pack("!H", len(encoded_value)) + encoded_value
                elif isinstance(value, bytes):
                    v_list_bytes += struct.pack("!H", len(value)) + value
                else:
                    raise ValueError(f"Unsupported value type in v_list: {type(value)}")

            # Pack Timestamps
            t_list_bytes = b"".join(struct.pack("!I", time_val) for time_val in t_list)
            # Pack Errors
            e_list_bytes = b"".join(struct.pack("!H", error) for error in e_list)

            # Combine all parts
            pdu_parts = [
                tag_bytes, type_bytes, timestamp_bytes, msg_id_bytes,
                iid_list_bytes, v_list_bytes, t_list_bytes, e_list_bytes
            ]
            return b"".join(pdu_parts)
        except Exception as e:
             print(f"Error encoding PDU: {e}")
             traceback.print_exc()
             return None


    def decode_pdu(self, data):
        """Decodes the PDU header from a byte string."""
        if not data:
            raise ValueError("Cannot decode empty data")

        offset = 0
        try:
            # Tag (1 byte)
            if offset + 1 > len(data): raise ValueError("PDU too short for tag")
            tag = data[offset:offset+1].decode('ascii')
            offset += 1

            # Type (1 byte)
            if offset + 1 > len(data): raise ValueError("PDU too short for type")
            msg_type = struct.unpack("!B", data[offset:offset+1])[0]
            offset += 1

            # Timestamp (4 bytes)
            if offset + 4 > len(data): raise ValueError("PDU too short for timestamp")
            timestamp = struct.unpack("!I", data[offset:offset+4])[0]
            offset += 4

            # Msg ID (4 bytes)
            if offset + 4 > len(data): raise ValueError("PDU too short for msg_id")
            msg_id = struct.unpack("!I", data[offset:offset+4])[0]
            offset += 4

            # --- Variable length lists ---
            # Parse remaining data in context (within _parse_response_data/_parse_set_response)
            remaining_data = data[offset:]

            # Return parsed fixed fields and the rest for contextual parsing
            return tag, msg_type, timestamp, msg_id, remaining_data
        except (ValueError, struct.error, UnicodeDecodeError) as e:
             print(f"Error decoding PDU header: {e}")
             traceback.print_exc()
             # Return Nones to indicate failure
             return None, None, None, None, None


    def _parse_get_response_data(self, remaining_data, expected_iids):
        """Helper to parse IIDs and Values from the remaining PDU data for GET responses."""
        offset = 0
        iid_list = []
        v_list = []
        t_list = [] # Assuming empty for GET response based on agent code
        e_list = [] # Assuming empty for GET response based on agent code

        # 1. Parse IIDs (Agent should return IIDs matching the request)
        num_expected_iids = len(expected_iids)
        for i in range(num_expected_iids):
            if offset + 2 > len(remaining_data):
                print(f"Warning: PDU data ended prematurely while expecting IID {i+1}/{num_expected_iids}.")
                break # Stop parsing IIDs
            try:
                iid = struct.unpack("!H", remaining_data[offset:offset+2])[0]
                iid_list.append(iid)
                offset += 2
            except struct.error as e:
                 print(f"Warning: Struct error unpacking IID {i+1}/{num_expected_iids}: {e}. Stopping IID parsing.")
                 break

        # 2. Parse Values (corresponding to each *parsed* IID)
        num_parsed_iids = len(iid_list)
        for i in range(num_parsed_iids):
            iid = iid_list[i] # Use the *returned* iid for mapping
            object_name = self.name_map.get(iid, f"Unknown IID {iid}")

            # Determine type based on agent's MIB definition (from iid_info in agent)
            # Replicating basic type info here for parsing
            # Agent encodes device.id, device.type, sensors.X.id, sensors.X.type as bytes
            is_byte_type = iid in [12, 13, 21, 22, 31, 32]

            try:
                if is_byte_type:
                    if offset + 2 > len(remaining_data): raise ValueError("short data for str/bytes length")
                    str_len = struct.unpack("!H", remaining_data[offset:offset+2])[0]
                    offset += 2
                    if offset + str_len > len(remaining_data): raise ValueError("short data for str/bytes content")
                    value_bytes = remaining_data[offset:offset+str_len]
                    try:
                        value = value_bytes.decode('utf-8') # Try decoding
                    except UnicodeDecodeError:
                        value = value_bytes # Keep as bytes if decode fails
                    v_list.append(value)
                    offset += str_len
                else: # Assume integer type (packed as signed int '!i' by agent)
                    if offset + 4 > len(remaining_data): raise ValueError("short data for integer")
                    value = struct.unpack("!i", remaining_data[offset:offset+4])[0] # Signed Int
                    v_list.append(value)
                    offset += 4
            except (struct.error, ValueError) as e:
                 print(f"Warning: Error parsing value for IID {iid} ({object_name}): {e}. Stopping value parsing.")
                 # Add None for this value and potentially subsequent ones
                 v_list.append(None)
                 break # Stop parsing values for this response

        # Pad v_list with None if fewer values were parsed than IIDs (due to errors)
        while len(v_list) < len(iid_list):
            v_list.append(None)

        return iid_list, v_list, t_list, e_list

    def _parse_set_response_data(self, remaining_data, expected_iids):
        """Helper to parse IIDs and Error Codes from the remaining PDU data for SET responses."""
        offset = 0
        iid_list = []
        v_list = [] # Empty for SET response
        t_list = [] # Empty for SET response
        e_list = [] # Error codes expected

        # 1. Parse IIDs (Agent should return IIDs matching the request)
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
                 print(f"Warning: Struct error unpacking IID {i+1}/{num_expected_iids} in SET response: {e}. Stopping IID parsing.")
                 break

        # 2. Parse Error Codes (corresponding to each *parsed* IID)
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
                 print(f"Warning: Struct error unpacking Error Code {i+1}/{num_parsed_iids} in SET response: {e}. Stopping error parsing.")
                 e_list.append(99) # Add placeholder error
                 break

        # Pad e_list with a general error code if fewer were parsed
        while len(e_list) < len(iid_list):
            e_list.append(1) # General error

        return iid_list, v_list, t_list, e_list


    def get(self, object_name):
        """Get a single value from the agent using L-SNMPvS protocol"""
        if object_name not in self.iid_map:
            print(f"Error: Unknown object name requested: {object_name}")
            return None

        iid = self.iid_map[object_name]
        req_iids = [iid]
        req_msg_id = self._get_next_msg_id()
        req_pdu = self.encode_pdu('G', 1, int(time.time()), req_msg_id, req_iids, [], [], [])
        if not req_pdu: return None # Encoding failed

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout)
            try:
                sock.sendto(req_pdu, (self.agent_address, self.agent_port))
                data, addr = sock.recvfrom(1024) # Buffer size

                decoded = self.decode_pdu(data)
                if decoded[0] is None: return None # Decoding header failed
                tag, msg_type, timestamp, resp_msg_id, remaining_data = decoded

                if tag != 'R':
                    print(f"Error: Received unexpected tag '{tag}' in GET response for {object_name}")
                    return None

                resp_iids, resp_values, _, _ = self._parse_get_response_data(remaining_data, req_iids)

                try:
                    value_index = resp_iids.index(iid)
                    if value_index < len(resp_values):
                         return resp_values[value_index]
                    else:
                         print(f"Error: Value missing for returned IID {iid} for {object_name}.")
                         return None
                except (ValueError, IndexError):
                     print(f"Error: Requested IID {iid} ({object_name}) not found or value missing in response. Got IIDs: {resp_iids}")
                     return None

            except socket.timeout:
                print(f"Error: Request timed out for {object_name}")
                return None
            except (ValueError, struct.error) as e:
                print(f"Error processing GET response for {object_name}: {e}")
                traceback.print_exc()
                return None
            except Exception as e:
                print(f"An unexpected error occurred during GET for {object_name}: {e}")
                traceback.print_exc()
                return None

    def get_bulk(self, object_names):
        """Get multiple values from the agent using L-SNMPvS protocol"""
        req_iids = []
        valid_object_names = []
        for name in object_names:
            if name in self.iid_map:
                req_iids.append(self.iid_map[name])
                valid_object_names.append(name)
            else:
                print(f"Warning: Unknown object name '{name}' skipped in bulk request.")

        if not req_iids: return {}

        req_msg_id = self._get_next_msg_id()
        req_pdu = self.encode_pdu('G', 1, int(time.time()), req_msg_id, req_iids, [], [], [])
        if not req_pdu: return {name: None for name in valid_object_names} # Encoding failed

        results = {name: None for name in valid_object_names} # Initialize results

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout)
            try:
                sock.sendto(req_pdu, (self.agent_address, self.agent_port))
                data, addr = sock.recvfrom(2048) # Larger buffer for bulk

                decoded = self.decode_pdu(data)
                if decoded[0] is None: return results # Decoding header failed
                tag, msg_type, timestamp, resp_msg_id, remaining_data = decoded

                if tag != 'R':
                    print(f"Error: Received unexpected tag '{tag}' in bulk response")
                    return results

                resp_iids, resp_values, _, _ = self._parse_get_response_data(remaining_data, req_iids)

                # Map results back based on returned IIDs
                for i, returned_iid in enumerate(resp_iids):
                    if i < len(resp_values):
                        returned_name = self.name_map.get(returned_iid)
                        if returned_name in results:
                            results[returned_name] = resp_values[i]
                        else:
                            print(f"Warning: Received unexpected/unmappable IID {returned_iid} in bulk response.")
                    else:
                         returned_name = self.name_map.get(returned_iid, f"Unknown IID {returned_iid}")
                         print(f"Warning: Missing value for returned IID {returned_iid} ({returned_name}) in bulk response.")

                if len(resp_iids) != len(req_iids) or len(resp_values) != len(req_iids):
                     print(f"Warning: Mismatch in expected vs received items for bulk get. Expected {len(req_iids)}, Got {len(resp_iids)} IIDs, {len(resp_values)} Values.")

                return results

            except socket.timeout:
                print(f"Error: Request timed out for bulk get")
                return results # Return dict with None values
            except (ValueError, struct.error) as e:
                print(f"Error processing bulk GET response: {e}")
                traceback.print_exc()
                return results
            except Exception as e:
                print(f"An unexpected error occurred during bulk GET: {e}")
                traceback.print_exc()
                return results

    # --- SET Method (Restored) ---
    def set(self, object_name, value):
        """Set a value on the agent using L-SNMPvS protocol. Returns True on success, False on failure."""
        if object_name not in self.iid_map:
            print(f"Error: Unknown object name for SET: {object_name}")
            return False

        iid = self.iid_map[object_name]
        req_iids = [iid]
        req_values = [value] # Value needs to be correct type (int/str) for encoding
        req_msg_id = self._get_next_msg_id()

        # Basic type check before encoding (agent does more validation)
        # Agent expects int for rates, reset, time
        if iid in [14, 16, 19, 27, 37]:
            if not isinstance(value, int):
                try:
                    req_values = [int(value)] # Attempt conversion
                except (ValueError, TypeError):
                    print(f"Error: Invalid value type for {object_name}. Expected int, got {type(value)} ('{value}')")
                    return False
        # Add checks for string types if any become settable

        req_pdu = self.encode_pdu('S', 1, int(time.time()), req_msg_id, req_iids, req_values, [], [])
        if not req_pdu: return False # Encoding failed

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(self.timeout)
            try:
                sock.sendto(req_pdu, (self.agent_address, self.agent_port))
                data, addr = sock.recvfrom(1024)

                decoded = self.decode_pdu(data)
                if decoded[0] is None: return False # Decoding header failed
                tag, msg_type, timestamp, resp_msg_id, remaining_data = decoded

                if tag != 'R':
                    print(f"Error: Received unexpected tag '{tag}' for SET response")
                    return False

                # Parse the response to check error codes
                resp_iids, _, _, resp_errors = self._parse_set_response_data(remaining_data, req_iids)

                # Check if the correct IID was returned and if the error code is 0 (success)
                try:
                    idx = resp_iids.index(iid)
                    if idx < len(resp_errors):
                        error_code = resp_errors[idx]
                        if error_code == 0:
                            print(f"SET successful for {object_name} (IID {iid})")
                            return True
                        else:
                            # Map error code to message (optional)
                            error_map = {1: "General Error", 2: "Bad Value", 3: "Not Settable", 4: "Unknown IID", 5: "Internal Error"}
                            print(f"SET failed for {object_name} (IID {iid}). Agent returned error code: {error_code} ({error_map.get(error_code, 'Unknown Error')})")
                            return False
                    else:
                         print(f"SET failed for {object_name}: Error code missing in response.")
                         return False
                except (ValueError, IndexError):
                     print(f"SET failed for {object_name}: Requested IID {iid} not found in response or error list.")
                     return False

            except socket.timeout:
                print(f"Error: SET Request timed out for {object_name}")
                return False
            except (ValueError, struct.error) as e:
                print(f"Error processing SET response for {object_name}: {e}")
                traceback.print_exc()
                return False
            except Exception as e:
                print(f"An unexpected error occurred during SET for {object_name}: {e}")
                traceback.print_exc()
                return False

    # --- monitor_device_status (Unchanged from previous version) ---
    def monitor_device_status(self, interval=5):
        """Continuously monitor device status using L-SNMPvS GET operations"""
        try:
            while True:
                print(f"\n--- Device Status ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---")

                # Get device information
                device_info = self.get_bulk([
                    'device.id',
                    'device.type',
                    'device.nSensors',
                    'device.upTime',
                    'device.opStatus',
                    'device.beaconRate' # Also get beacon rate
                ])

                # Check if bulk get was successful (returned a dict)
                if device_info is None:
                     print("Failed to retrieve device information (bulk get failed).")
                     time.sleep(interval)
                     continue # Skip sensor part if device info failed

                # Process device info (handle potential None values)
                dev_id = device_info.get('device.id', 'N/A')
                dev_type = device_info.get('device.type', 'N/A')
                num_sensors_val = device_info.get('device.nSensors') # Get raw value
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

                # Get sensor information only if num_sensors is valid
                num_sensors = 0
                if isinstance(num_sensors_val, int) and num_sensors_val > 0:
                    num_sensors = num_sensors_val
                elif num_sensors_val is not None:
                     print(f"Warning: Unexpected value for device.nSensors: {num_sensors_val}")


                for sensor_num in range(1, num_sensors + 1):
                     # Check if sensor definitions exist in our map
                     sensor_id_name = f'sensors.{sensor_num}.id'
                     if sensor_id_name not in self.iid_map:
                         print(f"\n--- Sensor {sensor_num} ---")
                         print(f"Sensor {sensor_num} definitions not found in monitor's map, skipping details.")
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
                         last_sample_ts = sensor_info.get(f'sensors.{sensor_num}.lastSamplingTime') # Get raw value
                         sampling_rate = sensor_info.get(f'sensors.{sensor_num}.samplingRate', 'N/A')

                         last_sample_str = 'N/A'
                         if isinstance(last_sample_ts, int):
                             try:
                                 last_sample_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(last_sample_ts))
                             except (OverflowError, OSError): # Handle invalid timestamps
                                 last_sample_str = f"Invalid Timestamp ({last_sample_ts})"
                         elif last_sample_ts is not None:
                              last_sample_str = f"Non-integer Timestamp ({last_sample_ts})"


                         print(f"Sample Value: {sample_val}%")
                         print(f"Last Sampling: {last_sample_str}")
                         print(f"Sampling Rate: {sampling_rate} Hz")
                     else:
                         print(f"Could not retrieve sensor {sensor_num} information (bulk get failed).")


                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user.")
        except Exception as e:
            print(f"\nAn error occurred during monitoring: {e}")
            traceback.print_exc()

# --- Main block (for command-line testing, not used by GUI) ---
if __name__ == "__main__":
    monitor = LSNMPvSMonitor('127.0.0.1', 12345)
    print("--- Running Command-Line Monitor ---")
    # Example SET calls for testing
    # print("\nAttempting SET operations...")
    # monitor.set('device.beaconRate', 8)
    # monitor.set('sensors.1.samplingRate', 15)
    # monitor.set('device.reset', 1)
    monitor.monitor_device_status(interval=10)
