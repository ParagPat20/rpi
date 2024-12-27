from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy
import geopy.distance
from geopy.distance import great_circle
import math
import json
from threading import Lock

class DroneVehicle:
    def __init__(self, connection_string, baud=None):
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

    def get_vehicle_state(self):
        """Returns vehicle state for ESP32 communication"""
        try:
            return {
                'armed': self.vehicle.armed,
                'mode': self.vehicle.mode.name,
                'altitude': self.vehicle.location.global_relative_frame.alt,
                'heading': self.vehicle.heading,
                'groundspeed': self.vehicle.groundspeed,
                'airspeed': self.vehicle.airspeed,
                'battery_voltage': self.vehicle.battery.voltage,
                'battery_current': self.vehicle.battery.current,
                'battery_level': self.vehicle.battery.level,
                'ekf_ok': self.vehicle.ekf_ok,
                'last_heartbeat': self.vehicle.last_heartbeat,
                'system_status': self.vehicle.system_status,
                'uptime': time.time() - self.start_time
            }
        except Exception as e:
            print(f"Error getting vehicle state: {e}")
            return None

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

    def handle_attitude_request(self, drone_id):
        """
        Handles attitude data requests for a specific drone.
        Sends commands to request location, GPS, battery, and attitude data.
        
        Args:
            drone_id (str): The ID of the drone (e.g., 'MCU', 'CD1', etc.)
            
        Returns:
            dict: A dictionary containing all the requested attitude data
        """
        try:
            # Initialize data dictionary
            self.attitude_data = {}
            
            # Define the data requests based on protocol
            data_requests = {
                'LOC': ['lat', 'lon', 'alt'],
                'GPS': ['type', 'satellites'],
                'BATT': ['volt', 'amp', 'level'],
                'ATTITUDE': ['pitch', 'roll', 'yaw', 'heading'],
                'SPEED': ['airspd', 'gndspd', 'velocity']
            }
            
            # Send commands for each data type
            for data_type, params in data_requests.items():
                try:
                    # Format and send command
                    command = f"{{T:{drone_id}; C:REQ; P:{data_type}}}\n"
                    print(f"Sending command: {command}")
                    
                    # Store the response in attitude_data
                    if data_type == 'LOC':
                        self.attitude_data.update({
                            'lat': self.vehicle.location.global_relative_frame.lat,
                            'lon': self.vehicle.location.global_relative_frame.lon,
                            'alt': self.vehicle.location.global_relative_frame.alt
                        })
                    elif data_type == 'GPS':
                        self.attitude_data.update({
                            'type': self.vehicle.gps_0.fix_type,
                            'satellites': self.vehicle.gps_0.satellites_visible
                        })
                    elif data_type == 'BATT':
                        self.attitude_data.update({
                            'volt': self.vehicle.battery.voltage,
                            'amp': self.vehicle.battery.current,
                            'level': self.vehicle.battery.level
                        })
                    elif data_type == 'ATTITUDE':
                        self.attitude_data.update({
                            'pitch': self.vehicle.attitude.pitch,
                            'roll': self.vehicle.attitude.roll,
                            'yaw': self.vehicle.attitude.yaw,
                            'heading': self.vehicle.heading
                        })
                    elif data_type == 'SPEED':
                        self.attitude_data.update({
                            'airspd': self.vehicle.airspeed,
                            'gndspd': self.vehicle.groundspeed,
                            'velocity': self.vehicle.velocity
                        })
                        
                except Exception as e:
                    print(f"Error requesting {data_type} data: {e}")
                    self.attitude_data[data_type] = f"Error: {str(e)}"
                    
            return self.attitude_data
                    
        except Exception as e:
            print(f"Error in handle_attitude_request: {e}")
            return {"error": str(e)}
            
    def get_attitude_data(self):
        """
        Returns the last collected attitude data
        
        Returns:
            dict: The last collected attitude data
        """
        return self.attitude_data 

class SerialHandler:
    def __init__(self, drone_vehicle):
        self.drone = drone_vehicle
        self.serial_port = None
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
        Processes a received message in the format {S:sender;C:cmd;P:payload}
        
        Args:
            message (str): The received message string
        """
        if message.startswith('{') and message.endswith('}'):
            try:
                # Remove brackets and split by semicolon
                parts = message[1:-1].split(';')
                command_dict = {}
                
                # Parse each part
                for part in parts:
                    key, value = part.strip().split(':')
                    command_dict[key.strip()] = value.strip()
                
                # Handle the command based on the parsed data
                sender = command_dict.get('S')
                command = command_dict.get('C')
                payload = command_dict.get('P')
                
                # Process the command (this is where you would add your logic)
                print(f"Received from {sender}: Command: {command}, Payload: {payload}")
                
            except Exception as e:
                print(f"Error processing received message: {e}") 