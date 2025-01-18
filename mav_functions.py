from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy
import geopy.distance
from geopy.distance import great_circle
import math
import json
from threading import Lock
import serial
import subprocess
from datetime import datetime
import os
import threading
import time
class LogBook:
    def __init__(self):
        # Get current username and home directory
        self.username = os.path.expandvars("$USER")
        self.home_dir = os.path.expanduser("~")
        
        # Create log directory in user's home directory
        self.log_dir = os.path.join(self.home_dir, "drone_logs")
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create dated log file
        current_date = datetime.now().strftime("%Y-%m-%d")
        self.log_file = os.path.join(self.log_dir, f"drone_log_{current_date}.txt")
        
        # Log initialization
        self.log_event("INIT", f"Log started for user: {self.username}")
        
    def log_event(self, event_type, details):
        """Log an event with timestamp and username"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] [{self.username}] {event_type}: {details}\n"
        try:
            with open(self.log_file, 'a') as f:
                f.write(log_entry)
        except Exception as e:
            print(f"Error writing to logbook: {e}")
            # Try to create directory again if it doesn't exist
            try:
                os.makedirs(self.log_dir, exist_ok=True)
                with open(self.log_file, 'a') as f:
                    f.write(log_entry)
            except Exception as e2:
                print(f"Fatal error writing to logbook: {e2}")

class DroneVehicle:
    def __init__(self, connection_string, baud=115200):
        self.connection_string = connection_string
        self.baud = baud
        self.vehicle = None
        self.posalt = 2
        self.in_air = False
        self.start_time = time.time()
        self.attitude_data = {}
        self.init_vehicle()
        
    def init_vehicle(self):
        """Initialize or reinitialize the vehicle connection"""
        if self.vehicle:
            self.disconnect()
        try:
            self.vehicle = connect(self.connection_string, baud=self.baud)
            print("Vehicle initialized successfully")
            return True
        except Exception as e:
            print(f"Failed to initialize vehicle: {e}")
            return False
        
    def disconnect(self):
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
            time.sleep(2)
            print("Disconnected Successfully")

    def arm(self, mode='GUIDED'):
        """Arms the vehicle in specified mode with timeout"""
        print("Arming motors")
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.armed = True
        
        start_time = time.time()
        timeout = 10  # Timeout in seconds
        while not self.vehicle.armed and time.time() - start_time < timeout:
            print("Waiting for arming...")
            time.sleep(1)
        if self.vehicle.armed:
            print("Vehicle Armed")
        else:
            print("Arming timed out")

    def takeoff(self, alt=5):
        """Takes off to specified altitude"""
        self.arm()
        print("Taking off!")
        self.vehicle.simple_takeoff(alt)
        self.posalt = alt
        start_time = time.time()
        TIMEOUT_SECONDS = 15
        
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {current_altitude}")
            if current_altitude >= alt * 0.95:
                print("Reached target altitude")
                self.set_mode("POSHOLD")
                time.sleep(2)
                self.set_mode("GUIDED")
                break
            else:
                print("Waiting for altitude information...")
            if time.time() - start_time > TIMEOUT_SECONDS:
                break
            time.sleep(1)
        self.in_air = True

    def land(self):
        """Switches to LAND mode"""
        self.vehicle.mode = VehicleMode("LAND")
        print("Landing")
        self.in_air = False

    def rtl(self):
        """Return to launch"""
        self.vehicle.mode = VehicleMode("RTL")
        print("Returning to Launch")
        self.in_air = False

    def poshold(self):
        """Switches to POSHOLD mode"""
        self.vehicle.mode = VehicleMode("POSHOLD")
        print("Position Hold mode enabled")

    def disarm(self):
        try:
            print("Disarming motors")
            self.vehicle.armed = False

            while self.vehicle.armed:
                print("Waiting for disarming...")
                self.vehicle.armed = False
                time.sleep(1)

            print("Vehicle Disarmed")
        except Exception as e:
            print(f"Error during disarming: {e}")

    def get_location(self):
        """Returns current location and heading"""
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        heading = self.vehicle.heading
        return (lat, lon), heading

    def goto(self, location, alt, groundspeed=0.7):
        """Goes to specified GPS location and altitude"""
        destination = LocationGlobalRelative(location[0], location[1], alt)
        
        current_lat = self.vehicle.location.global_relative_frame.lat
        current_lon = self.vehicle.location.global_relative_frame.lon
        current_alt = self.vehicle.location.global_relative_frame.alt
        
        while ((self.distance_between_points((current_lat, current_lon), location) > 0.5) or 
               (abs(current_alt - alt) > 0.3)):
            self.vehicle.simple_goto(destination, groundspeed=groundspeed)
            time.sleep(0.5)
            current_lat = self.vehicle.location.global_relative_frame.lat
            current_lon = self.vehicle.location.global_relative_frame.lon
            current_alt = self.vehicle.location.global_relative_frame.alt

    def send_ned_velocity_drone(self, velocity_x, velocity_y, velocity_z):
        try:
            velocity_x = float(velocity_x)
            velocity_y = float(velocity_y)
            velocity_z = float(velocity_z)
            

            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

            self.vehicle.send_mavlink(msg)
            print("Drone Velocity : {}, {}, {}".format(velocity_x,velocity_y,velocity_z))

        except Exception as e:
            print(f"Error sending velocity commands: {e}")
            
    def send_ned_velocity(self, x, y, z, duration = None):
        self.no_vel_cmds = False
        if duration:
            for i in range(0,duration):
                self.send_ned_velocity_drone(x,y,z)
                print(i)
                time.sleep(1)

            self.send_ned_velocity_drone(0,0,0)
            time.sleep(1)
            self.no_vel_cmds = True
            
        else:
            self.send_ned_velocity_drone(x,y,z)

    def yaw(self, heading, relative=False):
        """
        Rotate vehicle to specified heading
        heading: degrees (0 is North)
        relative: if True, heading is relative to current heading
        """
        if relative:
            is_relative = 1
        else:
            is_relative = 0
            
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
            0,       # confirmation
            heading, # param 1, yaw in degrees
            0,       # param 2, yaw speed deg/s
            1,       # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0) # param 5 ~ 7 not used
        
        self.vehicle.send_mavlink(msg)


    def set_mode(self, mode_name):
        """Changes vehicle mode"""
        self.vehicle.mode = VehicleMode(mode_name)
        print(f"Mode changed to {mode_name}")

    def close_vehicle(self):
        """Closes vehicle connection"""
        self.vehicle.close()

    @staticmethod
    def distance_between_points(point1, point2):
        """Calculate distance between two GPS points"""
        return great_circle(point1, point2).meters

    @staticmethod
    def new_location(original_location, distance, bearing):
        """
        Calculate new location given distance and bearing from original location
        distance: in meters
        bearing: in degrees (0 is North)
        """
        vincentyDistance = geopy.distance.distance(meters=distance)
        original_point = geopy.Point(original_location[0], original_location[1])
        new_location = vincentyDistance.destination(point=original_point, bearing=bearing)
        
        return (round(new_location.latitude, 7), round(new_location.longitude, 7)) 
    
    def move_to_location(self, distance, altitude, direction_degree):
        """
        Move the drone to a new location based on distance, altitude, and direction.
        
        :param distance: Distance to move in meters.
        :param altitude: Altitude to maintain in meters.
        :param direction_degree: Direction in degrees (0 is North, 90 is east).
        """
        try:
            # Get the current GPS location of the drone
            current_location = (self.vehicle.location.global_relative_frame.lat, 
                                self.vehicle.location.global_relative_frame.lon)

            # Calculate the new coordinates based on distance and direction
            new_location = self.new_coords(current_location, distance, direction_degree)

            # Calculate the distance to the new location
            distance_to_new_location = self.calculate_distance(current_location, new_location)
            distance_to_new_location1 = self.distance_between_two_gps_coord(current_location, new_location)

            # print the intended movement
            print("{} moving to new location {} at altitude {}m with direction {} degrees".format(new_location, altitude, direction_degree))

            # Check if the new location is within 10 meters
            if distance_to_new_location <= 100 and distance_to_new_location1 <= 100:
                # Command the drone to go to the new location at the specified altitude
                self.goto(new_location, altitude)
            else:
                print("{} Move to Location Error: New location is too far ({} meters)".format(distance_to_new_location))

        except Exception as e:
            print("{} Move to Location Error: {}".format(e))
        # Example: drone.move_to_location(distance=100, altitude=10, direction_degree=90)

    def rc_ov(self, mode, ch1=0, ch2=0, ch3=0, ch4=0, ch5=0, ch6=0):
        """
        Override RC channels.
        
        :param mode: RC mode to override
        :param ch1: Value for RC channel 1
        :param ch2: Value for RC channel 2
        :param ch3: Value for RC channel 3
        :param ch4: Value for RC channel 4
        :param ch5: Value for RC channel 5
        :param ch6: Value for RC channel 6
        """
        try:
            if mode == 1:
                # Set the RC channel overrides
                self.vehicle.channels.overrides = {
                    '1': ch1,
                    '2': ch2,
                    '3': ch3,
                    '4': ch4,
                    '5': ch5,
                    '6': ch6
                }

                print(f"RC channels overridden: {ch1}, {ch2}, {ch3}, {ch4}, {ch5}, {ch6}")
            else:
                self.vehicle.channels.overrides = {
                    '1': None,
                    '2': None,
                    '3': None,
                    '4': None,
                    '5': None,
                    '6': None
                }
        except Exception as e:
            print(f"Error overriding RC channels: {e}")
        # Example: drone.rc_ov(mode=1, ch1=1500, ch2=1500, ch3=1500, ch4=1500)

#################################################################################################################

    def calculate_distance(self, location1, location2):
        """
        Calculate the distance between two GPS coordinates.
        
        :param location1: Tuple of (latitude, longitude) for the first location.
        :param location2: Tuple of (latitude, longitude) for the second location.
        :return: Distance in meters.
        """
        from geopy.distance import geodesic
        return geodesic(location1, location2).meters

    def distance_between_two_gps_coord(self, point1, point2):
        try:
            distance = great_circle(point1, point2).meters
            return distance
        except Exception as e:
            print(f"Error calculating distance between two GPS coordinates: {e}")

    def new_coords(self, original_gps_coord, displacement, rotation_degree_relative):
        try:
            vincentyDistance = geopy.distance.distance(meters=displacement)
            original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
            new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree_relative)
            new_gps_lat = new_gps_coord.latitude
            new_gps_lon = new_gps_coord.longitude

            return (round(new_gps_lat, 7), round(new_gps_lon, 7))
        except Exception as e:
            print(f"Error in calculating new coordinates: {e}")


    def handle_param_request(self, command):
        """
        Handles parameter data requests based on the command.
        Constructs and returns the requested data as a string.
        
        Args:
            command (str): The command specifying the data to request (e.g., 'LOC', 'GPS', etc.)
            
        Returns:
            str: A comma-separated string of the requested data
        """
        try:
            # Define the data requests based on protocol
            data_requests = {
                'LOC': ['lat', 'lon', 'alt'],
                'GPS': ['type', 'satellites'],
                'BATT': ['volt', 'amp', 'level'],
                'ATTITUDE': ['pitch', 'roll', 'yaw', 'heading'],
                'SPEED': ['airspd', 'gndspd', 'velocity'],
                'MODE': ['mode'],
                'ARMED': ['is_armed']
            }
            
            # Construct the response string
            response = ""
            if command in data_requests:
                params = data_requests[command]
                for param in params:
                    if param == 'lat':
                        response += f"{self.vehicle.location.global_relative_frame.lat},"
                    elif param == 'lon':
                        response += f"{self.vehicle.location.global_relative_frame.lon},"
                    elif param == 'alt':
                        response += f"{self.vehicle.location.global_relative_frame.alt},"
                    elif param == 'type':
                        response += f"{self.vehicle.gps_0.fix_type},"
                    elif param == 'satellites':
                        response += f"{self.vehicle.gps_0.satellites_visible},"
                    elif param == 'volt':
                        response += f"{self.vehicle.battery.voltage},"
                    elif param == 'amp':
                        response += f"{self.vehicle.battery.current},"
                    elif param == 'level':
                        response += f"{self.vehicle.battery.level},"
                    elif param == 'pitch':
                        response += f"{self.vehicle.attitude.pitch},"
                    elif param == 'roll':
                        response += f"{self.vehicle.attitude.roll},"
                    elif param == 'yaw':
                        response += f"{self.vehicle.attitude.yaw},"
                    elif param == 'heading':
                        response += f"{self.vehicle.heading},"
                    elif param == 'airspd':
                        response += f"{self.vehicle.airspeed},"
                    elif param == 'gndspd':
                        response += f"{self.vehicle.groundspeed},"
                    elif param == 'velocity':
                        response += f"{self.vehicle.velocity},"
                    elif param == 'mode':
                        response += f"{self.vehicle.mode.name},"
                    elif param == 'is_armed':
                        response += f"{self.vehicle.armed},"
                # Remove the trailing comma
                response = response.rstrip(',')
            else:
                response = "Invalid command"
                
            return response
                    
        except Exception as e:
            print(f"Error in handle_param_request: {e}")
            return "Error: " + str(e)
            
    def get_attitude_data(self):
        """
        Returns the last collected attitude data
        
        Returns:
            dict: The last collected attitude data
        """
        return self.attitude_data 

class SerialHandler:
    def __init__(self, drone_vehicle, port, baudrate):
        self.drone = drone_vehicle
        self.serial_port = serial.Serial(port=port, baudrate=baudrate)
        self.is_running = False
        self.read_thread = None
        self.write_lock = Lock()
        self.last_heartbeat = 0
        self.last_gui_update = 0
        self.heartbeat_interval = 1.0  # 1 second
        self.logbook = LogBook()
        self.command_threads = {}  # Dictionary to track command threads
        self.failsafe_thread = None
        self.MAX_ALTITUDE = 15  # meters
        self.FAILSAFE_CHECK_INTERVAL = 3  # seconds

    def start_failsafe_monitor(self):
        """Start the failsafe monitoring thread"""
        self.failsafe_thread = threading.Thread(target=self._failsafe_check)
        self.failsafe_thread.daemon = True
        self.failsafe_thread.start()
        self.logbook.log_event("FAILSAFE", "Failsafe monitoring thread started")

    def _failsafe_check(self):
        """Continuously monitor drone altitude for safety"""
        while self.is_running:
            try:
                if self.drone.vehicle:
                    current_altitude = self.drone.vehicle.location.global_relative_frame.alt
                    if current_altitude > self.MAX_ALTITUDE:
                        self.logbook.log_event("FAILSAFE", f"Altitude exceeded {self.MAX_ALTITUDE}m (Current: {current_altitude:.2f}m)")
                        print(f"FAILSAFE: Altitude exceeded {self.MAX_ALTITUDE}m, initiating emergency landing...")
                        
                        # Initiate landing
                        self.drone.land()
                        
                        # Monitor landing progress
                        while current_altitude > 0.3:  # Wait until near ground
                            current_altitude = self.drone.vehicle.location.global_relative_frame.alt
                            self.logbook.log_event("FAILSAFE", f"Emergency landing in progress. Altitude: {current_altitude:.2f}m")
                            time.sleep(1)
                            
                        self.logbook.log_event("FAILSAFE", "Emergency landing completed")
            except Exception as e:
                self.logbook.log_event("FAILSAFE_ERROR", f"Error in failsafe monitor: {str(e)}")
            
            time.sleep(self.FAILSAFE_CHECK_INTERVAL)

    def start(self):
        """Start all handler threads"""
        self.is_running = True
        # Start serial reading thread
        self.read_thread = threading.Thread(target=self.read_serial_commands)
        self.read_thread.daemon = True
        self.read_thread.start()
        # Start failsafe monitoring
        self.start_failsafe_monitor()
        self.logbook.log_event("SYSTEM", "All handler threads started")

    def execute_command_in_thread(self, sender, command, payload):
        """Execute a command in a separate thread"""
        try:
            response = self.handle_commands(command, payload)
            self.send_message(sender, 'RES', response)
            print(f"Executed command from {sender}: Command: {command}, Payload: {payload}")
            # Remove thread from tracking dictionary when done
            if f"{sender}_{command}" in self.command_threads:
                del self.command_threads[f"{sender}_{command}"]
        except Exception as e:
            error_msg = f"Error executing command {command}: {str(e)}"
            self.send_message(sender, 'RES', error_msg)
            print(error_msg)

    def process_received_message(self, message):
        """
        Processes a received message in the format {S:sender,C:cmd,P:payload}
        Executes commands in separate threads
        """
        print(f"Raw message received: {message}")
        if message.startswith('{') and message.endswith('}'):
            try:
                parts = message[1:-1].split(';')
                command_dict = {}
                
                for part in parts:
                    key, value = part.strip().split(':', maxsplit=1)
                    command_dict[key.strip()] = value.strip()
                
                sender = command_dict.get('S')
                command = command_dict.get('C')
                payload = command_dict.get('P')
                
                if command == 'REQ':
                    # Handle parameter requests in main thread as they're quick
                    response_data = self.drone.handle_param_request(payload)
                    self.send_message(sender, payload, response_data)
                    print(f"Sent response to {sender}: {response_data}")
                else:
                    # Create a new thread for command execution
                    thread_name = f"{sender}_{command}"
                    if thread_name not in self.command_threads:  # Only start if not already running
                        command_thread = threading.Thread(
                            target=self.execute_command_in_thread,
                            args=(sender, command, payload),
                            name=thread_name
                        )
                        self.command_threads[thread_name] = command_thread
                        command_thread.start()
                        print(f"Started thread for command {command} from {sender}")
                    else:
                        print(f"Command {command} from {sender} is already running")
                
            except Exception as e:
                print(f"Error processing received message '{message}': {e}")
                if sender:
                    self.send_message(sender, 'RES', f"Error: {str(e)}")

    def stop(self):
        """Stop all running command threads"""
        self.is_running = False
        # Wait for all command threads to finish
        for thread_name, thread in self.command_threads.items():
            if thread.is_alive():
                thread.join(timeout=2.0)  # Wait up to 2 seconds for each thread
        self.command_threads.clear()

    def send_message(self, target, command, payload):
        """
        Sends a message through the serial port in the format {T:target;C:cmd;P:payload}
        
        Args:
            command (str): The command to send
            payload (str): The payload associated with the command
            target (str): The target identifier
        """
        message = f"{{T:{target};C:{command};P:{payload}}}\n"
        self._send_raw_message(message)

    def _send_raw_message(self, message_str):
        """Sends a raw message string through the serial port."""
        try:
            if self.serial_port and self.serial_port.is_open:
                with self.write_lock:
                    self.serial_port.write(message_str.encode())
                    self.serial_port.flush()
        except Exception as e:
            print(f"Error sending message: {e}")

    def read_serial_commands(self):
        """Reads commands from the serial port and processes them."""
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode().strip()
                    self.process_received_message(line)
            except Exception as e:
                print(f"Error reading from serial port: {e}")

    def handle_commands(self, command, payload):
        """
        Handles drone commands based on the protocol
        """
        try:
            if command == "INIT":
                # Kill existing mav session if exists
                try:
                    subprocess.run(['tmux', 'kill-session', '-t', 'mav'])
                except:
                    pass  # Ignore if session doesn't exist
                # Reinitialize vehicle connection
                success = self.drone.init_vehicle()
                self.logbook.log_event("INIT", f"Vehicle reinitialization {'successful' if success else 'failed'}")
                if success:
                    return "Vehicle reinitialized and connected"
                else:
                    return "Failed to reinitialize vehicle"
                
            elif command == "SOCAT":
                if self.drone.vehicle:
                    self.drone.disconnect()
                if payload != "0" and payload != 0:
                    ip, port = payload.split(',')
                    socat_cmd = f'socat UDP4-DATAGRAM:{ip}:{port} /dev/serial0,b115200,raw,echo=0'
                else:
                    socat_cmd = 'socat UDP4-DATAGRAM:192.168.146.161:14552 /dev/serial0,b115200,raw,echo=0'
                
                try:
                    subprocess.run(['sudo', 'tmux', 'new-session', '-d', '-s', 'mav', socat_cmd])
                    self.logbook.log_event("SOCAT", f"Connection established with command: {socat_cmd}")
                    return "SOCAT connection established"
                except Exception as e:
                    self.logbook.log_event("SOCAT_ERROR", str(e))
                    return f"Error in SOCAT: {e}"
                
            elif command == "CLOSE":
                self.drone.disconnect()
                self.logbook.log_event("CLOSE", "Vehicle disconnected")
                return "Disconnected from drone"
                
            elif command == "ARM":
                mode = payload if payload else "GUIDED"
                self.drone.arm(mode)
                self.logbook.log_event("ARM", f"Armed in {mode} mode")
                return f"Armed in {mode} mode"
                
            elif command == "DISARM":
                self.drone.disarm()
                self.logbook.log_event("DISARM", "Vehicle disarmed")
                return "Disarmed"
                
            elif command == "LAUNCH":
                alt = float(payload) if payload else 2
                self.drone.takeoff(alt)
                self.logbook.log_event("LAUNCH", f"Takeoff to altitude {alt}m")
                return f"Launched to altitude {alt}m"
                
            elif command == "NED":
                x, y, z, yaw = map(float, payload.split(','))
                self.drone.send_ned_velocity(x, y, z)
                if yaw != 0:
                    self.drone.yaw(heading=yaw, relative=True)       
                self.logbook.log_event("NED", f"Velocity command x:{x} y:{y} z:{z} yaw:{yaw}")
                return f"Moving with velocity x:{x} y:{y} z:{z} yaw:{yaw}"
            elif command == "YAW":
                h, r = map(float, payload.split(','))
                self.drone.yaw(h, bool(r))
                self.logbook.log_event("YAW", f"Yaw to heading {h} {'relative' if bool(r) else 'absolute'}")
                return f"Yawing to heading {h}"
                
            elif command == "SET_MODE":
                if payload == "FLIP":
                    self.drone.set_mode("ALTHOLD")
                    while self.drone.vehicle.mode.name != "ALTHOLD":
                        time.sleep(0.1)
                    self.drone.set_mode("FLIP")
                    time.sleep(2)
                    while self.drone.vehicle.mode.name != "ALTHOLD":
                        time.sleep(0.1)
                    self.drone.set_mode("GUIDED")
                    while self.drone.vehicle.mode.name != "GUIDED":
                        time.sleep(0.1)
                    self.logbook.log_event("SET_MODE", "FLIP sequence completed")
                    return "Mode set to GUIDED after FLIP sequence"
                else:
                    self.drone.set_mode(payload)
                    while self.drone.vehicle.mode.name != payload:
                        time.sleep(0.1)
                    self.logbook.log_event("SET_MODE", f"Mode changed to {payload}")
                    return f"Mode set to {payload}"
                
            elif command == "MTL":
                dis, alt, bearing = map(float, payload.split(','))
                current_loc = self.drone.get_location()[0]
                new_loc = self.drone.new_location(current_loc, dis, bearing)
                self.drone.goto(new_loc, alt)
                self.logbook.log_event("MTL", f"Moving to location - Distance:{dis}m Alt:{alt}m Bearing:{bearing}°")
                return f"Moving to location at distance {dis}m, altitude {alt}m, bearing {bearing}°"
                
            elif command == "LAND":
                self.drone.land()
                self.logbook.log_event("LAND", "Landing initiated")
                return "Landing initiated"
                
            elif command == "POS":
                try:
                    # Parse the payload
                    coords = payload.split(',')
                    if len(coords) != 4:
                        raise ValueError("POS command requires 4 values: x,y,altitude,heading")
                    
                    x = float(coords[0])
                    y = float(coords[1])
                    altitude = float(coords[2])
                    heading = float(coords[3])

                    # Calculate distance and direction from x,y coordinates
                    distance = math.sqrt(x*x + y*y)
                    
                    # Calculate direction in degrees (0° is North, 90° is East)
                    direction = math.degrees(math.atan2(x, y))  # Changed from atan2(y,x) to atan2(x,y)
                    
                    # Normalize direction to 0-360 degrees
                    direction = (90 - direction) % 360  # Adjust to make 0° North
                    
                    # Log the calculated values
                    print(f"Calculated distance: {distance}m, direction: {direction}°")
                    
                    # Execute the movement
                    self.drone.move_to_location(distance, altitude, direction)
                    self.logbook.log_event("POS", 
                        f"Moving to position - X:{x}m Y:{y}m Alt:{altitude}m Heading:{heading}°")
                    
                    # After reaching position, adjust heading
                    time.sleep(1)  # Small delay to ensure position is reached
                    self.drone.yaw(heading)
                    
                    return (f"Moving to position X:{x}m Y:{y}m at altitude {altitude}m "
                            f"(distance: {distance:.1f}m, direction: {direction:.1f}°) "
                            f"with final heading {heading}°")
                    
                except ValueError as ve:
                    error_msg = f"Invalid POS command format: {str(ve)}"
                    self.logbook.log_event("ERROR", error_msg)
                    return error_msg
                
            else:
                self.logbook.log_event("UNKNOWN_COMMAND", f"Unknown command received: {command}")
                return f"Unknown command: {command}"
                
        except Exception as e:
            error_msg = f"Error executing {command}: {e}"
            self.logbook.log_event("ERROR", error_msg)
            return error_msg