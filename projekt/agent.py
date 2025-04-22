import socket
import threading
import random
import time
# Remove multiprocessing Queue if not strictly needed between agent parts
# from multiprocessing import Process, Queue
from queue import Queue # Use standard threading queue
import struct
import traceback # For better error logging

# --- Virtual Sensor Class (Unchanged) ---
class VirtualSensor:
    def __init__(self, sensor_id, min_value, max_value, update_interval, precision):
        self.sensor_id = str(sensor_id) # Ensure ID is string
        self.min_value = min_value
        self.max_value = max_value
        self.update_interval = update_interval # Updates per second (Hz)
        self.precision = precision
        self.current_value = self.generate_value()
        self.last_sampling_time = time.time()
        # Sampling rate is how often the MIB *could* be sampled,
        # Update interval is how often the sensor *actually* generates a new value.
        # Let's make samplingRate configurable via SET.
        self.sampling_rate = 10  # Default sampling rate (Hz) - settable
        self.active = True
        # Notification rate isn't used in current PDU, remove for now
        # self.notification_rate = self.sampling_rate

    def generate_value(self):
        value = random.uniform(self.min_value, self.max_value)
        return round(value, self.precision)

    def update_value(self):
        # Only generate a new value if enough time has passed based on update_interval
        # This prevents generating values much faster than needed if start loop is fast
        now = time.time()
        if self.active and (now - self.last_sampling_time) >= (1.0 / self.update_interval):
            self.current_value = self.generate_value()
            self.last_sampling_time = now
            return True # Value updated
        return False # Value not updated

    def get_value(self):
        # Return the latest generated value
        return self.current_value

    def get_last_sampling_time(self):
        # Return the timestamp of the last generated value
        return int(self.last_sampling_time)

    def get_sampling_rate(self):
        return self.sampling_rate

    def set_sampling_rate(self, rate):
        if isinstance(rate, int) and rate > 0:
            self.sampling_rate = rate
            print(f"Sensor {self.sensor_id} sampling rate set to {rate} Hz")
            return True
        return False

    def get_min_value(self):
        return self.min_value

    def get_max_value(self):
        return self.max_value

    def get_sample_value_percentage(self):
        # Return the latest value as a percentage
        if self.min_value == self.max_value:
            return 0
        percentage = ((self.current_value - self.min_value) / (self.max_value - self.min_value)) * 100
        # Clamp between 0 and 100, return as int
        return max(0, min(100, int(percentage)))

    def start(self, queue, shutdown_flag):
        """Sensor simulation loop"""
        while not shutdown_flag.is_set():
            updated = self.update_value()
            if updated:
                # Put the *raw* value and timestamp on the queue for MIB update
                queue.put((self.sensor_id, self.current_value, self.last_sampling_time))
                # print(f"Sensor {self.sensor_id} generated: {self.current_value}") # Debug

            # Sleep for a short duration, the update_value logic handles the actual interval
            time.sleep(0.05) # Sleep briefly to avoid busy-waiting


# --- L-MIB Data Structure ---
class LMIB:
    def __init__(self, sensors_dict_ref):
        # Keep a reference to the agent's sensors dictionary
        self._sensors_ref = sensors_dict_ref
        self._device_data = {
            11: 1,                  # device.lMibId (Integer)
            12: "DeviceABC",        # device.id (String)
            13: "Hub LSV1",         # device.type (String)
            14: 5,                  # device.beaconRate (Integer, seconds)
            15: 0,                  # device.nSensors (Integer)
            16: int(time.time()),   # device.dateAndTime (Timestamp)
            17: 0,                  # device.upTime (Integer, seconds)
            18: 1,                  # device.opStatus (Integer, 1=normal)
            19: 0,                  # device.reset (Integer, 0=no reset)
        }
        # Store sensor data keyed by IID base (e.g., 20 for sensor 1, 30 for sensor 2)
        self._sensors_data = {}
        self._start_time = time.time()
        self.lock = threading.Lock() # Lock for accessing MIB data

        # Map IIDs to names/info for easier lookup and type handling
        self.iid_info = {
            11: {"name": "device.lMibId", "type": int, "settable": False},
            12: {"name": "device.id", "type": str, "settable": False},
            13: {"name": "device.type", "type": str, "settable": False},
            14: {"name": "device.beaconRate", "type": int, "settable": True, "min": 1},
            15: {"name": "device.nSensors", "type": int, "settable": False},
            16: {"name": "device.dateAndTime", "type": int, "settable": True},
            17: {"name": "device.upTime", "type": int, "settable": False},
            18: {"name": "device.opStatus", "type": int, "settable": False}, # Could be settable if needed
            19: {"name": "device.reset", "type": int, "settable": True},
            # Sensor 1 (Base 20)
            21: {"name": "sensors.1.id", "type": str, "settable": False},
            22: {"name": "sensors.1.type", "type": str, "settable": False}, # Assuming fixed type for now
            23: {"name": "sensors.1.sampleValue", "type": int, "settable": False}, # Percentage
            24: {"name": "sensors.1.minValue", "type": int, "settable": False},
            25: {"name": "sensors.1.maxValue", "type": int, "settable": False},
            26: {"name": "sensors.1.lastSamplingTime", "type": int, "settable": False},
            27: {"name": "sensors.1.samplingRate", "type": int, "settable": True, "min": 1},
            # Sensor 2 (Base 30)
            31: {"name": "sensors.2.id", "type": str, "settable": False},
            32: {"name": "sensors.2.type", "type": str, "settable": False},
            33: {"name": "sensors.2.sampleValue", "type": int, "settable": False},
            34: {"name": "sensors.2.minValue", "type": int, "settable": False},
            35: {"name": "sensors.2.maxValue", "type": int, "settable": False},
            36: {"name": "sensors.2.lastSamplingTime", "type": int, "settable": False},
            37: {"name": "sensors.2.samplingRate", "type": int, "settable": True, "min": 1},
            # Add more sensors/IIDs as needed
        }

    def _get_sensor_id_from_iid(self, iid):
        """Helper to get sensor ID string ('1', '2', etc.) from IID"""
        if 21 <= iid <= 29: return "1"
        if 31 <= iid <= 39: return "2"
        # Add ranges for other sensors
        return None

    def add_sensor(self, sensor):
        """Adds sensor info to the MIB"""
        with self.lock:
            sensor_id_str = sensor.sensor_id
            if sensor_id_str == "1": base_iid = 20
            elif sensor_id_str == "2": base_iid = 30
            # Add logic for more sensors
            else:
                print(f"Warning: Cannot add sensor with unsupported ID '{sensor_id_str}' to MIB IID map.")
                return

            self._sensors_data[base_iid] = {
                base_iid + 1: sensor.sensor_id, # sensors.X.id (String)
                base_iid + 2: "Simulated",       # sensors.X.type (String) - make dynamic if needed
                base_iid + 3: sensor.get_sample_value_percentage(), # sensors.X.sampleValue (Int %)
                base_iid + 4: sensor.get_min_value(), # sensors.X.minValue (Int)
                base_iid + 5: sensor.get_max_value(), # sensors.X.maxValue (Int)
                base_iid + 6: sensor.get_last_sampling_time(), # sensors.X.lastSamplingTime (Timestamp)
                base_iid + 7: sensor.get_sampling_rate(), # sensors.X.samplingRate (Int Hz)
            }
            # Update sensor count
            self._device_data[15] = len(self._sensors_data)
            print(f"Sensor {sensor_id_str} added to MIB. nSensors = {self._device_data[15]}")

    def update_sensor_mib_data(self, sensor_id, raw_value, timestamp):
        """Updates MIB entries for a specific sensor based on new raw value"""
        with self.lock:
            if sensor_id == "1": base_iid = 20
            elif sensor_id == "2": base_iid = 30
            # Add logic for more sensors
            else: return # Sensor not tracked in MIB

            if base_iid in self._sensors_data:
                sensor_obj = self._sensors_ref.get(sensor_id)
                if sensor_obj:
                    # Update sampleValue (percentage) and lastSamplingTime
                    self._sensors_data[base_iid][base_iid + 3] = sensor_obj.get_sample_value_percentage() # Use sensor's method
                    self._sensors_data[base_iid][base_iid + 6] = int(timestamp)
                    # samplingRate (iid + 7) is updated via SET

    def get_data(self, iid):
        """Gets data from MIB using IID"""
        with self.lock:
            if iid in self._device_data:
                if iid == 17: # device.upTime
                    self._device_data[iid] = int(time.time() - self._start_time)
                return self._device_data.get(iid)
            else:
                # Check sensors
                for base_iid, sensor_dict in self._sensors_data.items():
                    if iid in sensor_dict:
                        return sensor_dict.get(iid)
            return None # Indicate IID not found

    def set_data(self, iid, value):
        """Sets data in MIB using IID, performs validation. Returns error code (0=success)."""
        with self.lock:
            if iid not in self.iid_info:
                print(f"SET Error: Unknown IID {iid}")
                return 4 # Unknown IID error code

            info = self.iid_info[iid]
            expected_type = info["type"]
            is_settable = info["settable"]

            if not is_settable:
                print(f"SET Error: IID {iid} ({info['name']}) is not settable.")
                return 3 # Not settable error code

            # --- Type Validation and Coercion ---
            try:
                if expected_type == int:
                    if not isinstance(value, int):
                        # Try to convert if possible (e.g., from string representation)
                        try:
                            value = int(value)
                        except (ValueError, TypeError):
                             raise ValueError(f"Expected integer for IID {iid}")
                elif expected_type == str:
                     if not isinstance(value, str):
                         # Try to convert if possible
                         try:
                             value = str(value)
                         except (ValueError, TypeError):
                             raise ValueError(f"Expected string for IID {iid}")
                # Add other type checks if needed (float, etc.)
            except ValueError as e:
                 print(f"SET Error: Type mismatch for IID {iid}. Value: '{value}'. Error: {e}")
                 return 2 # Bad value / type error code

            # --- Specific Value Validation ---
            min_val = info.get("min")
            if min_val is not None and value < min_val:
                 print(f"SET Error: Value {value} for IID {iid} is below minimum {min_val}")
                 return 2 # Bad value error code

            # --- Apply the SET ---
            if iid in self._device_data:
                self._device_data[iid] = value
                print(f"MIB SET: {info['name']} (IID {iid}) = {value}")
                # Handle side effects
                if iid == 19 and value == 1: # device.reset
                    self.reset_device()
                    self._device_data[iid] = 0 # Reset the trigger back to 0
            else:
                # Sensor data SET
                sensor_id_str = self._get_sensor_id_from_iid(iid)
                sensor_obj = self._sensors_ref.get(sensor_id_str)
                if not sensor_obj:
                     print(f"SET Error: Sensor object '{sensor_id_str}' not found for IID {iid}")
                     return 5 # Internal/config error

                if iid % 10 == 7: # samplingRate (e.g., 27, 37)
                    if sensor_obj.set_sampling_rate(value):
                         # Update MIB cache as well
                         base_iid = (iid // 10) * 10
                         if base_iid in self._sensors_data:
                             self._sensors_data[base_iid][iid] = value
                             print(f"MIB SET: {info['name']} (IID {iid}) = {value}")
                         else:
                              print(f"SET Warning: MIB cache inconsistency for sensor {sensor_id_str}")
                              return 5 # Internal error
                    else:
                         print(f"SET Error: Failed to set sampling rate on sensor object {sensor_id_str}")
                         return 2 # Bad value (rejected by sensor object)
                else:
                    # This case shouldn't be reached if iid_info['settable'] is correct
                    print(f"SET Error: Attempt to set unhandled sensor IID {iid}")
                    return 3 # Not settable

            return 0 # Success

    def reset_device(self):
        """Resets device uptime and related state."""
        print("--- Device Reset Triggered ---")
        self._start_time = time.time()
        self._device_data[17] = 0 # Reset uptime
        self._device_data[16] = int(self._start_time) # Update dateAndTime
        # Potentially reset sensor states or last sample times if needed
        # for base_iid, sensor_data in self._sensors_data.items():
        #     sensor_data[base_iid + 6] = int(self._start_time) # Reset last sample time
        print("--- Device Reset Complete ---")


# --- L-SNMPvS Agent Class ---
class AgenteLSNMPvS:
    def __init__(self, endereco_agente, porta_agente):
        self.endereco_agente = endereco_agente
        self.porta_agente = porta_agente
        self.sensors = {} # Dictionary to hold VirtualSensor objects { '1': sensor_obj, ... }
        self.mib = LMIB(self.sensors) # Pass sensor dict reference to MIB
        self.queue = Queue() # Thread-safe queue for sensor updates
        self.lock = threading.Lock() # General lock if needed for agent state
        self.msg_id_counter = random.randint(0, 1000) # Start with random offset
        self.shutdown_flag = threading.Event()

    def add_sensor(self, sensor):
        with self.lock: # Protect access to sensors dict
            sensor_id_str = str(sensor.sensor_id)
            self.sensors[sensor_id_str] = sensor
            self.mib.add_sensor(sensor) # Inform MIB about the new sensor

    # Removed get_sensor_value as MIB handles data access

    def get_next_msg_id(self):
        with self.lock: # Protect counter
            self.msg_id_counter += 1
            return self.msg_id_counter % (2**32)

    def encode_pdu(self, tag, msg_type, timestamp, msg_id, iid_list, v_list, t_list, e_list):
        """Encodes the PDU into a byte string."""
        try:
            tag_bytes = tag.encode('ascii')
            type_bytes = struct.pack("!B", msg_type)
            timestamp_bytes = struct.pack("!I", timestamp)
            msg_id_bytes = struct.pack("!I", msg_id)
            iid_list_bytes = b"".join(struct.pack("!H", iid) for iid in iid_list)

            v_list_bytes = b""
            for value in v_list:
                if isinstance(value, int):
                    v_list_bytes += struct.pack("!i", value) # Signed 32-bit integer
                elif isinstance(value, str):
                    encoded_value = value.encode('utf-8')
                    v_list_bytes += struct.pack("!H", len(encoded_value)) + encoded_value # Length prefix + bytes
                elif isinstance(value, bytes):
                    # Assume already encoded bytes (e.g., from MIB if stored as bytes)
                    v_list_bytes += struct.pack("!H", len(value)) + value # Length prefix + bytes
                else:
                    # Log error and skip value or encode as error string?
                    print(f"Encoding Error: Unsupported value type {type(value)}: {value}")
                    # Encode as empty string to avoid crashing
                    v_list_bytes += struct.pack("!H", 0)


            t_list_bytes = b"".join(struct.pack("!I", time_val) for time_val in t_list)
            e_list_bytes = b"".join(struct.pack("!H", error) for error in e_list) # Error codes

            return tag_bytes + type_bytes + timestamp_bytes + msg_id_bytes + iid_list_bytes + v_list_bytes + t_list_bytes + e_list_bytes
        except Exception as e:
            print(f"PDU Encoding failed: {e}")
            traceback.print_exc()
            return None # Indicate failure

    def decode_pdu(self, data):
        """Decodes fixed PDU fields and IIDs. Returns remaining data for value parsing."""
        try:
            if len(data) < 10: # Minimum length (Tag+Type+TS+MsgID)
                raise ValueError("PDU too short for header")

            offset = 0
            tag = data[offset:offset+1].decode('ascii')
            offset += 1
            msg_type = struct.unpack("!B", data[offset:offset+1])[0]
            offset += 1
            timestamp = struct.unpack("!I", data[offset:offset+4])[0]
            offset += 4
            msg_id = struct.unpack("!I", data[offset:offset+4])[0]
            offset += 4

            iid_list = []
            # Read IIDs (2 bytes each) until data runs out or structure changes
            while offset + 2 <= len(data):
                # Heuristic: Stop if we encounter bytes that are unlikely IIDs?
                # For now, assume all remaining pairs are IIDs until value parsing starts.
                # A better PDU would have an IID count.
                # Let's assume IIDs come first, then values. The number of values
                # should match the number of IIDs in a SET request.
                # We will parse IIDs here, and let handle_set parse values.
                try:
                    iid = struct.unpack("!H", data[offset:offset+2])[0]
                    # Basic sanity check for IID range (optional)
                    # if not (10 <= iid < 1000): # Example range
                    #    break # Stop if value looks out of expected range
                    iid_list.append(iid)
                    offset += 2
                except struct.error:
                    break # Stop if unpacking fails

            remaining_data = data[offset:]

            return tag, msg_type, timestamp, msg_id, iid_list, remaining_data

        except (ValueError, struct.error, UnicodeDecodeError) as e:
            print(f"PDU Decoding failed: {e}")
            traceback.print_exc()
            return None, None, None, None, None, None # Indicate failure


    def handle_request(self, data, addr, server_socket):
        """Handles a single incoming UDP request."""
        print(f"Received {len(data)} bytes from {addr}")
        decoded = self.decode_pdu(data)
        if decoded[0] is None: # Check if decoding failed
             print(f"Failed to decode PDU from {addr}")
             return # Ignore malformed packet

        tag, msg_type, timestamp, req_msg_id, iid_list, remaining_data = decoded
        resp_msg_id = self.get_next_msg_id() # Generate response ID
        resp_pdu = None

        print(f"Request Decoded: Tag={tag}, Type={msg_type}, MsgID={req_msg_id}, IIDs={iid_list}")

        if tag == 'G': # GET Request
            resp_values = self.handle_get(iid_list)
            # Response: Tag 'R', Type 1, TS, MsgID, IIDs, Values, empty T, empty E
            resp_pdu = self.encode_pdu('R', 1, int(time.time()), resp_msg_id, iid_list, resp_values, [], [])

        elif tag == 'S': # SET Request
            # handle_set now parses values from remaining_data and returns error codes
            error_codes = self.handle_set(iid_list, remaining_data)
            # Response: Tag 'R', Type 1, TS, MsgID, IIDs, empty V, empty T, Error Codes
            resp_pdu = self.encode_pdu('R', 1, int(time.time()), resp_msg_id, iid_list, [], [], error_codes)

        elif tag == 'B': # Beacon Request (Client asks agent to identify itself)
             # Respond with a standard beacon PDU
             self.send_beacon_response(server_socket, addr) # Send directly back
             return # No further processing needed here

        else:
            print(f"Received unknown PDU tag '{tag}' from {addr}")
            # Optionally send an error response PDU
            # error_codes = [1] * len(iid_list) # General error
            # resp_pdu = self.encode_pdu('R', 1, int(time.time()), resp_msg_id, iid_list, [], [], error_codes)


        # Send Response PDU if one was generated
        if resp_pdu:
            try:
                server_socket.sendto(resp_pdu, addr)
                print(f"Sent response (Tag R, MsgID {resp_msg_id}) to {addr}")
            except Exception as e:
                print(f"Error sending response to {addr}: {e}")
        else:
             print(f"No response generated for request from {addr} (Tag: {tag})")


    def handle_get(self, iid_list):
        """Processes a GET request, retrieving values from MIB."""
        response_values = []
        for iid in iid_list:
            value = self.mib.get_data(iid)
            if value is None:
                print(f"GET: IID {iid} not found in MIB.")
                # Represent 'not found' - use a specific value or type?
                # Standard SNMP uses NoSuchObject/NoSuchInstance.
                # Let's return a specific integer or empty bytes?
                # For simplicity, maybe return 0 for int, empty string/bytes for others.
                # Or rely on monitor knowing the type and handling None?
                # Let's try returning the type's default value if known.
                info = self.mib.iid_info.get(iid)
                if info and info["type"] == int: value = 0
                elif info and info["type"] == str: value = ""
                else: value = b"" # Default to empty bytes
            elif isinstance(value, str):
                 value = value.encode('utf-8') # Ensure strings are encoded to bytes for PDU

            response_values.append(value)
            # print(f"GET: IID={iid}, Value={value}") # Debug
        return response_values

    def handle_set(self, iid_list, value_data):
        """Processes a SET request. Parses values and calls MIB set_data."""
        error_codes = []
        offset = 0
        values_parsed = 0

        for i, iid in enumerate(iid_list):
            info = self.mib.iid_info.get(iid)
            expected_type = info["type"] if info else None
            value = None
            parse_error = False

            # --- Parse value based on expected type ---
            try:
                if expected_type == int:
                    if offset + 4 > len(value_data): raise ValueError("Not enough data for int")
                    value = struct.unpack("!i", value_data[offset:offset+4])[0]
                    offset += 4
                elif expected_type == str:
                    if offset + 2 > len(value_data): raise ValueError("Not enough data for str length")
                    str_len = struct.unpack("!H", value_data[offset:offset+2])[0]
                    offset += 2
                    if offset + str_len > len(value_data): raise ValueError("Not enough data for str content")
                    # Decode directly to string here for mib.set_data
                    value = value_data[offset:offset+str_len].decode('utf-8')
                    offset += str_len
                # Add elif for bytes if needed (similar to str but no decode)
                else:
                    # Unknown type or IID not found - cannot parse value reliably
                    print(f"SET Parse Error: Cannot determine type for IID {iid}. Skipping value.")
                    # How to advance offset? Assume error for this IID and stop parsing?
                    # For now, mark error and try to continue if possible (might corrupt offset)
                    parse_error = True # Mark error, will result in error code below
                    # We cannot reliably advance offset here. Stop parsing values.
                    break


                values_parsed += 1

            except (struct.error, ValueError, UnicodeDecodeError) as e:
                print(f"SET Parse Error: Failed to parse value for IID {iid} at offset {offset}: {e}")
                parse_error = True
                # Stop parsing further values as offset is likely wrong
                break

            # --- Call MIB set_data ---
            if not parse_error:
                error_code = self.mib.set_data(iid, value)
                error_codes.append(error_code)
            else:
                 error_codes.append(1) # General parse/protocol error

        # If parsing stopped early, add error codes for remaining IIDs
        while len(error_codes) < len(iid_list):
            error_codes.append(1) # General error for unparsed/failed items

        print(f"SET Result: IIDs={iid_list}, Errors={error_codes}")
        return error_codes

    def send_beacon_response(self, sock, addr):
        """Sends a beacon PDU back to the requesting address."""
        # Define IIDs included in the beacon response
        beacon_iids = [11, 12, 13, 15, 16, 17, 18] # lMibId, id, type, nSensors, time, uptime, status
        beacon_values = self.handle_get(beacon_iids) # Reuse handle_get logic
        resp_msg_id = self.get_next_msg_id()

        beacon_pdu = self.encode_pdu('B', 1, int(time.time()), resp_msg_id, beacon_iids, beacon_values, [], [])
        if beacon_pdu:
            try:
                sock.sendto(beacon_pdu, addr)
                print(f"Sent Beacon Response (Tag B, MsgID {resp_msg_id}) to {addr}")
            except Exception as e:
                print(f"Error sending beacon response to {addr}: {e}")

    def send_periodic_beacon(self, shutdown_flag):
        """Sends unsolicited beacons periodically to the broadcast address."""
        print("Starting periodic beacon sender...")
        # Use a separate socket for sending broadcasts
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as beacon_socket:
            beacon_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            broadcast_addr = ('<broadcast>', self.porta_agente)
            while not shutdown_flag.wait(0): # Non-blocking check
                try:
                    beacon_rate = self.mib.get_data(14) # Get current beaconRate (IID 14)
                    if not isinstance(beacon_rate, int) or beacon_rate < 1:
                        print("Warning: Invalid beacon rate in MIB, defaulting to 5s.")
                        beacon_rate = 5

                    # Define IIDs for periodic beacon
                    beacon_iids = [11, 12, 13, 15, 16, 17, 18]
                    beacon_values = self.handle_get(beacon_iids)
                    beacon_msg_id = self.get_next_msg_id()

                    beacon_pdu = self.encode_pdu('B', 1, int(time.time()), beacon_msg_id, beacon_iids, beacon_values, [], [])

                    if beacon_pdu:
                        beacon_socket.sendto(beacon_pdu, broadcast_addr)
                        # print(f"Sent Periodic Beacon (MsgID {beacon_msg_id})") # Debug
                    else:
                         print("Error encoding periodic beacon PDU.")

                    # Wait for the configured interval, checking shutdown flag periodically
                    shutdown_flag.wait(beacon_rate)

                except Exception as e:
                    print(f"Error in periodic beacon thread: {e}")
                    traceback.print_exc()
                    # Avoid busy-looping on error
                    shutdown_flag.wait(5) # Wait 5s before retrying after error
        print("Periodic beacon sender stopped.")


    def start_server(self, shutdown_flag):
        """Main UDP server loop."""
        print(f"L-SNMPvS Agent starting UDP listener on {self.endereco_agente}:{self.porta_agente}")
        # Use try-with-resources for the socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            try:
                server_socket.bind((self.endereco_agente, self.porta_agente))
                # Set a timeout so the loop can check the shutdown flag
                server_socket.settimeout(1.0)
            except OSError as e:
                 print(f"FATAL: Could not bind to {self.endereco_agente}:{self.porta_agente} - {e}")
                 shutdown_flag.set() # Signal other threads to stop
                 return

            print("Agent listening...")
            while not shutdown_flag.is_set():
                try:
                    # Wait for an incoming packet
                    data, addr = server_socket.recvfrom(2048) # Increased buffer size
                    # Handle the request directly (or spawn thread if processing is heavy)
                    self.handle_request(data, addr, server_socket)

                except socket.timeout:
                    # No data received, loop continues to check shutdown_flag
                    continue
                except Exception as e:
                    # Log other potential errors during recvfrom or handling
                    print(f"Error in server loop: {e}")
                    traceback.print_exc()

        print("UDP listener stopped.")


    def update_mib_from_queue(self, shutdown_flag):
        """Thread to update MIB data from sensor queue."""
        print("Starting MIB update thread...")
        while not shutdown_flag.is_set():
            try:
                # Get data from queue, block for a short time
                sensor_id, value, timestamp = self.queue.get(timeout=0.5)
                self.mib.update_sensor_mib_data(sensor_id, value, timestamp)
                self.queue.task_done() # Mark task as complete
            except Empty: # Use queue.Empty exception
                 # Queue was empty, continue loop to check shutdown flag
                 continue
            except Exception as e:
                 print(f"Error in MIB update thread: {e}")
                 traceback.print_exc()
        print("MIB update thread stopped.")


    def start_sensors(self, shutdown_flag):
        """Starts all registered sensors in separate threads."""
        print("Starting sensor simulation threads...")
        sensor_threads = []
        if not self.sensors:
             print("Warning: No sensors added to the agent.")
             return

        for sensor in self.sensors.values():
            # Pass the shared queue and shutdown flag to each sensor's start method
            thread = threading.Thread(target=sensor.start, args=(self.queue, shutdown_flag), daemon=True)
            sensor_threads.append(thread)
            thread.start()
        print(f"{len(sensor_threads)} sensor threads started.")
        # Note: We don't join here, they run until shutdown_flag is set.


    def run(self):
        """Starts all agent components and waits for shutdown."""
        print("--- Starting L-SNMPvS Agent ---")
        all_threads = []

        # 1. Start Sensors
        # Sensors need to be started before MIB updater potentially reads from queue
        self.start_sensors(self.shutdown_flag) # Doesn't block

        # 2. Start MIB Update Thread
        mib_update_thread = threading.Thread(target=self.update_mib_from_queue, args=(self.shutdown_flag,), daemon=True)
        all_threads.append(mib_update_thread)
        mib_update_thread.start()

        # 3. Start Periodic Beacon Sender Thread
        beacon_thread = threading.Thread(target=self.send_periodic_beacon, args=(self.shutdown_flag,), daemon=True)
        all_threads.append(beacon_thread)
        beacon_thread.start()

        # 4. Start UDP Server (runs in the main thread after others are launched)
        # This makes it easier to catch KeyboardInterrupt in the main thread
        try:
             self.start_server(self.shutdown_flag)
        except KeyboardInterrupt:
             print("\nShutdown signal received (KeyboardInterrupt)...")
        except Exception as e:
             print(f"\nFATAL error in main server execution: {e}")
             traceback.print_exc()
        finally:
             print("Signalling all threads to shut down...")
             self.shutdown_flag.set()

             # Wait briefly for threads to notice the flag
             time.sleep(1.5)

             # Check if threads are still alive (optional)
             # for t in all_threads:
             #     if t.is_alive():
             #         print(f"Waiting for thread {t.name} to finish...")
             #         t.join(timeout=2.0) # Give threads a chance to exit cleanly

             print("--- L-SNMPvS Agent Shutdown Complete ---")


# --- Main Execution ---
if __name__ == "__main__":
    # Import Empty from queue for the exception handling
    from queue import Empty

    agent = AgenteLSNMPvS(
        "0.0.0.0",  # Listen on all available interfaces
        12345      # Standard port
    )

    # Add virtual sensors
    # VirtualSensor(sensor_id, min_value, max_value, update_interval_hz, precision)
    sensor1 = VirtualSensor("1", 0, 100, update_interval=1, precision=1) # Updates 1 time per second
    sensor2 = VirtualSensor("2", -20, 50, update_interval=0.5, precision=2) # Updates every 2 seconds
    agent.add_sensor(sensor1)
    agent.add_sensor(sensor2)

    # Run the agent (blocks until shutdown)
    agent.run()
