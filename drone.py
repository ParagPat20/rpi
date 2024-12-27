## Functions
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy
import geopy.distance
from geopy.distance import great_circle
import math
import threading
import zmq

context = zmq.Context()  # Create a ZeroMQ context

d1 = None
d2 = None
selected_drone = None
MCU_host = '192.168.174.173'
CD1_host = '192.168.174.149'
CD2_host = '192.168.174.177'
CD3_host = CD2_host
CD4_host = CD1_host
pc = '192.168.174.161'

hosts = {
    'MCU' : MCU_host,
    'CD1' : CD1_host,
    'CD2' : CD2_host,
    'CD3' : CD3_host,
    'CD4' : CD4_host
}
cmd_port = 12345
ctrl_port = 54321
drone_list = []
wait_for_command = True
immediate_command_str = None
missions = {}
poller = zmq.Poller
wifi_status = True
camera_running = True

def tmuxkill():
    try:
        import subprocess
        
        # Create new tmux session with socat command
        command = f"sudo tmux kill-session -t mav"
        subprocess.run(command, shell=True, check=True)
        
        log("TMUX session killed successfully")
    except Exception as e:
        log("Error executing socat command: {}".format(e))

class Drone:
    
    def __init__(self,name,connection_string, baud=None):
        self.vehicle = connect(connection_string, baud = baud)
        self.drone_user = connection_string
        self.drone_baud = baud
        self.name = name
        self.posalt = 2
        self.in_air = False
        self.no_vel_cmds = True
        self.pid_velx = {'P': 0.8, 'I': 0.0, 'D': 0.1}
        self.pid_vely = {'P': 0.8, 'I': 0.0, 'D': 0.1}
        self.pid_velz = {'P': 0.5, 'I': 0.0, 'D': 0.1}
        self.prev_error_velx = 0.0
        self.prev_error_vely = 0.0
        self.prev_error_velz = 0.0
        self.integral_velx = 0.0
        self.integral_vely = 0.0
        self.integral_velz = 0.0
        self.alt_ach = False
        self.prev_timestamp = time.time()
        self.wifi_status = True
        self.claw_state='closed'


    def is_wifi_connected(self):
        global wifi_status
        while True:
            try:
                wifi = context.socket(zmq.REQ)
                wifi.connect(f'tcp://{pc}:8888')

                wifi.send_string(self.name)
                wifi.setsockopt(zmq.RCVTIMEO, 5000)  # Set 3-second timeout for response

                try:
                    response = wifi.recv_string()
                    if response == "Connected":
                        self.wifi_status = True
                        wifi_status = self.wifi_status
                    else:
                        self.wifi_status = False
                        wifi_status = self.wifi_status
                    wifi.close()
                except zmq.Again:  # Timeout occurred
                    self.wifi_status = False
                    wifi_status = self.wifi_status
                    print("Waiting for new connection to be established")
                    wifi.close()
                    wifi = context.socket(zmq.REQ)
                    wifi.connect(f'tcp://{pc}:8888')

                    wifi.send_string("check")
                    wifi.setsockopt(zmq.RCVTIMEO, 10000)

                time.sleep(2)

            except zmq.ZMQError as e:
                print(f"ZMQ Error: {e}")
                self.wifi_status = False

            except Exception as e:
                print(f"General Error: {e}")
                self.wifi_status=False

            finally:
                if wifi:
                    wifi.close()

    def poshold_guided(self):
        while True:
            try:
                self.altitude = self.vehicle.location.global_relative_frame.alt
                velx = self.vehicle.velocity[0]
                vely = self.vehicle.velocity[1]
                velz = self.vehicle.velocity[2]
                if self.in_air:
                    if self.no_vel_cmds:
                        current_timestamp = time.time()
                        dt = current_timestamp - self.prev_timestamp
                        self.prev_timestamp = current_timestamp
                        # Use PID controllers for velx and vely
                        pid_output_velx = self.calculate_pid_output(velx, self.pid_velx, 'velx',dt)
                        pid_output_vely = self.calculate_pid_output(vely, self.pid_vely, 'vely',dt)
                        pid_output_velz = self.calculate_pid_output(velz, self.pid_velz, 'velz',dt)
                        if pid_output_velx > 2:
                            pid_output_velx = 2
                        if pid_output_vely > 2:
                            pid_output_vely = 2
                        if pid_output_velx < -2:
                            pid_output_velx = -2
                        if pid_output_vely < -2:
                            pid_output_vely = -2
                        if pid_output_velz > 2:
                            pid_output_velz = 2
                        if pid_output_velz > 2:
                            pid_output_velz = 2

                        self.send_ned_velocity_drone(pid_output_velx, pid_output_vely, pid_output_velz)
                        time.sleep(0.2)
                if not self.in_air:
                    break

                
            except Exception as e:
                log("Poshold_Guided Error: {}".format(e))

    def calculate_pid_output(self, current_value, pid_params, axis, dt):
        # Proportional term
        error = 0.0 - current_value
        proportional = pid_params['P'] * error
        
        # Integral term
        if axis == 'velx':
            self.integral_velx += error * dt  # Accumulate error over time
            integral = pid_params['I'] * self.integral_velx
        elif axis == 'vely':
            self.integral_vely += error * dt
            integral = pid_params['I'] * self.integral_vely
        elif axis == 'velz':
            self.integral_velz += error * dt
            integral = pid_params['I'] * self.integral_velz
        else:
            integral = 0.0

        if not self.no_vel_cmds:
            integral = 0.0

        # Derivative term

        if axis == 'velx':
            derivative = pid_params['D'] * ((error - self.prev_error_velx) / dt)  # dt: time difference
            self.prev_error_velx = error
        elif axis == 'vely':
            derivative = pid_params['D'] * ((error - self.prev_error_vely) / dt)
            self.prev_error_vely = error
        elif axis == 'velz':
            derivative = pid_params['D'] * ((error - self.prev_error_velz) / dt)
            self.prev_error_velz = error
        else:
            derivative = 0.0


        # Summing up all terms
        pid_output = proportional + integral + derivative

        return pid_output

    def security(self):
        self.altitude = self.vehicle.location.global_relative_frame.alt
        self.battery = self.vehicle.battery.voltage
        log(f"{self.name}'s Security checkup started!")
        threading.Thread(target=self.is_wifi_connected).start()

        while self.name != "STOP":
            try:
                self.altitude = self.vehicle.location.global_relative_frame.alt
                self.battery = self.vehicle.battery.voltage
                self.mode = self.vehicle.mode
                self.gps_type = self.vehicle.gps_0.fix_type 
                self.gps_num = self.vehicle.gps_0.satellites_visible
                
                if self.wifi_status:
                    log('sec {} PosAlt: {}m \n      Current altitude : {}m\n      Current Battery {}V\n      Alt Difference {}\n      Wifi Status {}\n{}'.format(self.name,self.posalt, self.altitude, self.battery, self.altitude - self.posalt, str(self.wifi_status), str(self.mode)))
                    coordlat = str(self.vehicle.location.global_relative_frame.lat)
                    coordlon = str(self.vehicle.location.global_relative_frame.lon)
                    log("lat {} {}".format(self.name
                                           ,coordlat))
                    log("lon {} {}".format(self.name,coordlon))
                    log("sec {} gps_type {}".format(self.name,
                                                    self.gps_type))
                    log("sec {} gps_num {}".format(self.name,
                                                   self.gps_num))
                if self.in_air:
                    if not self.wifi_status:
                        print("{} Wi-Fi connection lost! Initiating landing.".format(self.name))
                        self.land()
                        self.disarm()
                if self.altitude > 5:
                    log("sec {} Altitude greater than 5 meters! Initiating landing.".format(self.name))
                    self.land()
                if self.battery < 10.5:
                    log("sec {} Battery LOW, Landing".format(self.name))
                    self.land()
                # if abs(self.altitude - self.posalt) > 0.1:
                #     z = (self.altitude-self.posalt)*0.7
                #     log('sec {} Alt diff change given: {}m/s'.format(self.name,z))
                time.sleep(2)
            except Exception as e:
                log("sec {} Security Error : {}".format(self.name,e))
                pass

    def reconnect(self):
        try:
            self.vehicle.close()
            self = None
            time.sleep(2)
            log("Reconnected Successfully")
        except Exception as e:
            log(f"Error during reconnection: {e}")

    def arm(self, mode='GUIDED'):
        try:
            log("Arming motors")
            self.vehicle.mode = VehicleMode(mode)
            self.vehicle.armed = True
            TIMEOUT_SECONDS = 10
            start_time = time.time()
            while not self.vehicle.armed:
                log("Waiting for Arming")
                self.vehicle.armed = True
                if time.time() - start_time > TIMEOUT_SECONDS:
                    break
                time.sleep(1)

            log("Vehicle Armed")
        except Exception as e:
            log(f"Error during arming: {e}")

    def takeoff(self, alt=1):
        try:
            self.arm()
            log("Taking off!")
            self.vehicle.simple_takeoff(alt)
            start_time = time.time()
            TIMEOUT_SECONDS = 15
            self.posalt = alt
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                if current_altitude is not None:
                    log(" Altitude: {}".format(current_altitude))
                    if current_altitude >= alt * 0.9:
                        log("Reached target altitude")
                        break
                else:
                    log("Waiting for altitude information...")
                if time.time() - start_time > TIMEOUT_SECONDS:
                    break
                time.sleep(1)
            self.in_air = True

            self.alt_ach = False
        except Exception as e:
            log(f"Error during takeoff: {e}")

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
            log("Drone Velocity : {}, {}, {}".format(velocity_x,velocity_y,velocity_z))

        except Exception as e:
            log(f"Error sending velocity commands: {e}")

    def ctrl_front(self,velocity_x):
        self.ctrl_front_t(velocity_x)
        self.no_vel_cmds=True
        time.sleep(0.6)
   

    def ctrl_front_t(self, velocity_x):
        try:
            velocity_x = float(velocity_x)

            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
                0b0000111111000111, # type_mask (only speeds enabled)
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, 0, 0, # x, y, z velocity in m/s
                0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

            self.vehicle.send_mavlink(msg)
            log("Drone Front Velocity : {}, {}, {}".format(velocity_x,0,0))

        except Exception as e:
            log(f"Error sending velocity commands: {e}")

    def send_ned_velocity(self, x, y, z, duration = None):
        self.no_vel_cmds = False
        if duration:
            for i in range(0,duration):
                self.send_ned_velocity_drone(x,y,z)
                log(i)
                time.sleep(1)

            self.send_ned_velocity_drone(0,0,0)
            time.sleep(1)
            self.no_vel_cmds = True
            
        else:
            self.send_ned_velocity_drone(x,y,z)


    def yaw(self, heading):
        try:
            current_heading = self.vehicle.heading
            log("Current Heading : {}".format(current_heading))
            if heading - current_heading <= 0:
                rotation = 1
            else:
                rotation = -1
            estimatedTime = heading / 30.0 + 1

            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                0,  # confirmation
                heading,  # param 1, yaw in degrees
                0,  # param 2, yaw speed deg/s
                rotation,  # param 3, direction -1 ccw, 1 cw
                0,  # param 4, relative offset 1, absolute angle 0
                0, 0, 0)  # param 5 ~ 7 not used
            # send command to vehicle
            self.vehicle.send_mavlink(msg)

            # Wait sort of time for the command to be fully executed.
            for t in range(0, int(math.ceil(estimatedTime))):
                time.sleep(1)
        except Exception as e:
            log(f"Error during yaw command: {e}")
    
    def ctrl_yaw(self,rotation):
        move = 10
        msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                0,  # confirmation
                move,  # param 1, yaw in degrees
                0,  # param 2, yaw speed deg/s
                rotation,  # param 3, direction -1 ccw, 1 cw
                1,  # param 4, relative offset 1, absolute angle 0
                0, 0, 0)  # param 5 ~ 7 not used
            # send command to vehicle
        self.vehicle.send_mavlink(msg)
    

    def circle(self, radius=1, start_theta=0, velocity=0.5):
        T = 2 * math.pi * radius / velocity
        dt = 0.5

        for t in range(int(T / dt)):
            theta = start_theta + (2 * math.pi * t * dt / T)
            north_velocity = velocity * math.cos(theta)
            east_velocity = velocity * math.sin(theta)

            self.send_ned_velocity_drone(north_velocity, east_velocity, 0)
            time.sleep(dt)
            if not self.in_air:
                break

    def disarm(self):
        try:
            log("Disarming motors")
            self.vehicle.armed = False

            while self.vehicle.armed:
                log("Waiting for disarming...")
                self.vehicle.armed = False
                time.sleep(1)

            log("Vehicle Disarmed")
        except Exception as e:
            log(f"Error during disarming: {e}")

    def land(self):
        try:
            self.vehicle.mode = VehicleMode("LAND")
            log("Landing")
            self.in_air = False
            if self.claw_state == 'closed':
                self.servo('open')
        except Exception as e:
            log(f"Error during landing: {e}")

    def poshold(self):
        try:
            self.vehicle.mode = VehicleMode("POSHOLD")
            log("Drone currently in POSHOLD")
        except Exception as e:
            log(f"Error during POSHOLD mode setting: {e}")

    def rtl(self):
        try:
            self.vehicle.mode = VehicleMode("RTL")
            log("Drone currently in RTL")
            self.in_air = False
        except Exception as e:
            log(f"Error during RTL mode setting: {e}")

    def cu_lo(self):
        try:
            lat = self.vehicle.location.global_relative_frame.lat
            lon = self.vehicle.location.global_relative_frame.lon
            heading = self.vehicle.heading
            return (lat, lon), heading
        except Exception as e:
            log(f"Error in getting current location: {e}")
            return (0.0, 0.0), 0.0  # Returning default values in case of an error

    def get_vehicle_state(self):
        try:
            log_msg = (
                '{} - Checking current Vehicle Status:\n'
                '     Global Location: lat={}, lon={}, alt(above sea level)={}\n'
                '     Global Location (relative altitude): lat={}, lon={}, alt(relative)={}\n'
                '     Local Location(NED coordinate): north={}, east={}, down={}\n'
                '     Velocity: Vx={}, Vy={}, Vz={}\n'
                '     GPS Info: fix_type={}, num_sat={}\n'
                '     Battery: voltage={}V, current={}A, level={}%\n'
                '     Heading: {} (degrees from North)\n'
                '     Groundspeed: {} m/s\n'
                '     Airspeed: {} m/s'
            ).format(
                time.ctime(),
                self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon,
                self.vehicle.location.global_frame.alt,
                self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.alt,
                self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east,
                self.vehicle.location.local_frame.down,
                self.vehicle.velocity[0], self.vehicle.velocity[1], self.vehicle.velocity[2],
                self.vehicle.gps_0.fix_type, self.vehicle.gps_0.satellites_visible,
                self.vehicle.battery.voltage, self.vehicle.battery.current, self.vehicle.battery.level,
                self.vehicle.heading, self.vehicle.groundspeed, self.vehicle.airspeed
            )

            log(log_msg)


        except Exception as e:
            log(f"Error getting vehicle state: {e}")

    def send_gps(self):
        try:
            context = zmq.Context()
            socket = context.socket(zmq.PUSH)

            socket.bind("tcp://*:5656")
            start_time = time.time()
            while time.time-start_time < 120:
                lat = self.vehicle.location.global_relative_frame.lat
                lon = self.vehicle.location.global_relative_frame.lon
                gps = f'{lat},{lon}'
                socket.send_string(gps)
        except Exception as e:
            log('{} Error in send_gps : {}'.format(self.name,e))

    def goto(self, l, alt, groundspeed=0.7):
        try:
            log('\n')
            log('{} - Calling goto_gps_location_relative(lat={}, lon={}, alt={}, groundspeed={})'.format(
                time.ctime(), l[0], l[1], alt, groundspeed))
            destination = LocationGlobalRelative(l[0], l[1], alt)
            log('{} - Before calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
            self.get_vehicle_state()
            current_lat = self.vehicle.location.global_relative_frame.lat
            current_lon = self.vehicle.location.global_relative_frame.lon
            current_alt = self.vehicle.location.global_relative_frame.alt
            while ((self.distance_between_two_gps_coord((current_lat, current_lon), l) > 0.5) or (
                    abs(current_alt - alt) > 0.3)):
                self.vehicle.simple_goto(destination, groundspeed=groundspeed)
                time.sleep(0.5)
                current_lat = self.vehicle.location.global_relative_frame.lat
                current_lon = self.vehicle.location.global_relative_frame.lon
                current_alt = self.vehicle.location.global_relative_frame.alt
                log('{} - Horizontal distance to destination: {} m.'.format(time.ctime(),
                                                                             self.distance_between_two_gps_coord(
                                                                                 (current_lat, current_lon), l)))
                log('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(),
                                                                                 current_alt - alt))
                if self.mode == 'LAND': 
                    break
            log('{} - After calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
            self.get_vehicle_state()
        except Exception as e:
            log(f"Error during goto command: {e}")

    def distance_between_two_gps_coord(self, point1, point2):
        try:
            distance = great_circle(point1, point2).meters
            return distance
        except Exception as e:
            log(f"Error calculating distance between two GPS coordinates: {e}")

    def check_distance(self,d1,d2):
        try:
            drone1loc = request_gps(hosts[d1])
            drone2loc = request_gps(hosts[d2])
            distance = self.distance_between_two_gps_coord(drone1loc,drone2loc)
            log("Distance between those drones is {} meters".format(distance))
            
        except Exception as e:
            log(f"Error in check_distance: {e}")

def request_gps(drone):
    '''
    drone = 'MCU' or 'CD1' etc...
    '''
    send(hosts[drone],f'{drone}.send_gps()')
    time.sleep(1)
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect(f'tcp://{hosts[drone]}:5656')
    gps = socket.recv_string()
    lat,lon = gps.split(',')
    log('Requested GPS from {}, Got {}'.format(drone,gps))
    return (float(lat),float(lon))
    
connected_hosts = set()
clients = {}

import random

def send(host, immediate_command_str):
    global connected_hosts
    global clients

    try:
        if host not in connected_hosts:
            connect_and_register_socket(host)

        immediate_command_str = str(immediate_command_str)
        clients[host].send_string(immediate_command_str, zmq.NOBLOCK)  # Non-blocking send
        log("Command sent successfully to {}".format(host))

    except zmq.error.Again:  # Handle non-blocking send errors
        poller.register(clients[host], zmq.POLLOUT)  # Wait for socket readiness
        socks = dict(poller.poll(1000))
        if clients[host] in socks and socks[clients[host]] == zmq.POLLOUT:
            clients[host].send_string(immediate_command_str)  # Retry sending
        else:
            log(f"Socket not ready for {host}, reconnecting...")
            reconnect_socket(host)

    except zmq.error.ZMQError as e:
        log(f"PC Host {host}: {e}")
        reconnect_socket(host)  # Attempt reconnection

def connect_and_register_socket(host):
    socket = context.socket(zmq.PUSH)
    socket.setsockopt(zmq.SNDHWM, 1000)  # Allow up to 1000 queued messages
    socket.connect(f"tcp://{host}:12345")
    clients[host] = socket
    connected_hosts.add(host)
    log("Clients: {}".format(clients))

def reconnect_socket(host):
    socket = clients[host]
    socket.close()
    socket = context.socket(zmq.PUSH)
    socket.connect(f"tcp://{host}:12345")
    clients[host] = socket

def add_drone(string):
    try:
        global drone_list
        drone_list.append(string)
    except Exception as e:
        log(f"Error adding drone: {e}")

def remove_drone(string):
    try:
        global drone_list
        drone_list.remove(string)
    except Exception as e:
        log(f"Error removing drone: {e}")
         
def chat(string):
    try:
        print(string)

    except Exception as e:
        log(f"Error in chat function: {e}")

context = zmq.Context()
dealer_socket = context.socket(zmq.DEALER)  # Create a single DEALER socket
dealer_socket.connect(f"tcp://{pc}:5556")  # Connect to the server

def log(immediate_command_str):
    try:
        immediate_command_str = str(immediate_command_str)
        dealer_socket.send_multipart([immediate_command_str.encode()])
        if not wifi_status:
            print(immediate_command_str)

    except zmq.ZMQError as e:
        print("Error sending message: %s", e)  # Log error

def log_reset_server():
    global dealer_socket
    dealer_socket.close()
    dealer_socket = context.socket(zmq.DEALER)  # Create a single DEALER socket
    dealer_socket.connect(f"tcp://{pc}:5556")  # Connect to the server
    log("Server has been reset to {}".format(pc))
        
def file_server():
    try:
        context = zmq.Context()
        socket = context.socket(zmq.REP)

        socket.bind(f"tcp://{MCU_host}:5577")
        print("File_recieve server Started in MCU!")

        print("Waiting for file_name")
        file_name = socket.recv_string()
        socket.send_string("Completed")
        print("File Name Recieved")
        print("waiting for data")
        file_data = socket.recv()

        
        with open(f"{file_name}.txt", 'wb') as file:
            file.write(file_data)

        # Send a response back to the client (EDIT application)
        socket.send_string("File received successfully")
        time.sleep(1)
        socket.close()
        context.term()
        missions[file_name] = file
        log("MCU has {} Missons".format(str(missions)))
    except Exception as e:
        log("File_Server Error in MCU : {}".format(e))
