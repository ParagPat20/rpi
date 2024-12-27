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

class DroneVehicle:
    def __init__(self, connection_string, baud=115200):
        self.vehicle = connect(connection_string, baud=baud)
        self.posalt = 2
        self.in_air = False
        self.start_time = time.time()
        self.attitude_data = {}
        
    def disconnect(self):
        self.vehicle.close()
        self.vehicle = None
        time.sleep(2)
        print("Disconnected Successfully")

    def arm(self, mode='GUIDED'):
        """Arms the vehicle in specified mode"""
        print("Arming motors")
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.armed = True
        
        while not self.vehicle.armed:
            print("Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        print("Vehicle Armed")

    def takeoff(self, alt=1):
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
                'SPEED': ['airspd', 'gndspd', 'velocity']
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
        self.gui_update_interval = 0.7  # 0.7 seconds

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

    def process_received_message(self, message):
        """
        Processes a received message in the format {S:sender,C:cmd,P:payload}
        
        Args:
            message (str): The received message string
        """
        print(f"Raw message received: {message}")  # Debugging line
        if message.startswith('{') and message.endswith('}'):
            try:
                # Remove brackets and split by comma instead of semicolon
                parts = message[1:-1].split(';')
                command_dict = {}
                
                # Parse each part
                for part in parts:
                    key, value = part.strip().split(':', maxsplit=1)
                    command_dict[key.strip()] = value.strip()
                
                # Handle the command based on the parsed data
                sender = command_dict.get('S')
                command = command_dict.get('C')
                payload = command_dict.get('P')
                                
                # Check if the command is REQ
                if command == 'REQ':
                    # Handle the attitude request and send the response back to the sender
                    response_data = self.drone.handle_param_request(payload)
                    self.send_message(sender, payload, response_data)  # Send the response back to the sender
                    print(f"Sent response to {sender}: {response_data}")  # Debugging line
                
                print(f"Received from {sender}: Command: {command}, Payload: {payload}")
                
            except Exception as e:
                print(f"Error processing received message '{message}': {e}")

    def handle_commands(self, command, payload):
        """
        Handles drone commands based on the protocol
        
        Args:
            command (str): Command to execute
            payload (str): Command parameters
        
        Returns:
            str: Response message or error
        """
        try:
            if command == "INIT":
                # Initialize connection
                return "Connected to drone"
                
            elif command == "CLOSE":
                self.drone.disconnect()
                return "Disconnected from drone"
                
            elif command == "ARM":
                mode = payload if payload else "GUIDED"
                self.drone.arm(mode)
                return f"Armed in {mode} mode"
                
            elif command == "DISARM":
                self.drone.disarm()
                return "Disarmed"
                
            elif command == "LAUNCH":
                alt = float(payload) if payload else 2
                self.drone.takeoff(alt)
                return f"Launched to altitude {alt}m"
                
            elif command == "NED":
                # Parse X,Y,Z,T from payload
                x, y, z, t = map(float, payload.split(','))
                self.drone.send_ned_velocity(x, y, z, t)
                return f"Moving with velocity x:{x} y:{y} z:{z} for {t}s"
                
            elif command == "YAW":
                # Parse heading and relative from payload
                h, r = map(float, payload.split(','))
                self.drone.yaw(h, bool(r))
                return f"Yawing to heading {h}"
                
            elif command == "SET_MODE":
                self.drone.set_mode(payload)
                return f"Mode set to {payload}"
                
            elif command == "MTL":
                # Parse distance, altitude, bearing from payload
                dis, alt, bearing = map(float, payload.split(','))
                current_loc = self.drone.get_location()[0]
                new_loc = self.drone.new_location(current_loc, dis, bearing)
                self.drone.goto(new_loc, alt)
                return f"Moving to location at distance {dis}m, altitude {alt}m, bearing {bearing}°"
                
            else:
                return f"Unknown command: {command}"
                
        except Exception as e:
            return f"Error executing {command}: {e}"