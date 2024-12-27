from drone import Drone
from drone import *

cmd_port = 12345
ctrl_port = 54321

MCU = None

##################################################### Initialization #####################################################

MCU_initialized = False
d1 = None

context = zmq.Context(10)  # Allow up to 10 concurrent sockets
msg_socket = context.socket(zmq.PULL)
msg_socket.bind("tcp://*:12345")
msg_socket.setsockopt(zmq.RCVHWM, 1000)  # High water mark for incoming messages

poller = zmq.Poller()
poller.register(msg_socket, zmq.POLLIN)  # Monitor for incoming messages

log('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        log(drone_list)
    except Exception as e:
        log("MCU_Host: Error in drone_list_update: {}".format(e))

def execute_command(immediate_command_str):
    if immediate_command_str.startswith("socat("):
        ipaddress = immediate_command_str.split("(")[1].strip(")")
        execute_socat_command(ipaddress)
        return
    try:
        log("Executing command: {}".format(repr(immediate_command_str)))
        exec(immediate_command_str)
        log('{} - Command executed successfully'.format(time.ctime()))
    except Exception as e:
        log('{} - Error in execute_command: {}'.format(time.ctime(), e))

def execute_socat_command(ipaddress = pc):
    try:
        import subprocess
        ipaddress = str(ipaddress)
        # Create new tmux session with socat command
        command = f"sudo tmux new-session -d -s mav 'socat UDP4-DATAGRAM:{ipaddress}:14550 /dev/serial0,b115200,raw,echo=0'"
        subprocess.run(command, shell=True, check=True)
        
        log("socat command executed successfully with IP: {}".format(ipaddress))
    except Exception as e:
        log("Error executing socat command: {}".format(e))

def run_mis(filename):
    try:
        # Open the mission file
        with open(f"{filename}.txt", 'r') as file:
            # Read each line from the file
            for line in file:
                # Skip empty lines
                if not line.strip():
                    continue
                line=str(line)

                # Execute the command
                exec(line)  # Assuming each line is a command

    except Exception as e:
        log("Error in run_mis: {}".format(e))

##########################################################################################################################

def initialize_MCU():
    try:
        global d1, MCU, MCU_initialized
        if not MCU and not MCU_initialized:
            d1_str = 'MCU'
            MCU = Drone(d1_str,'/dev/serial0', 115200)
            # MCU = Drone(d1_str,'COM6',115200)
            d1 = MCU
            log("MCU Connected")
            time.sleep(5)
            log("MCU getting ready for the params...")
            MCU.get_vehicle_state()
            threading.Thread(target=MCU.security).start()
            MCU_initialized=True
        MCU.get_vehicle_state()
        log('MCU_status')
    except Exception as e:
        log("MCU_Host: Error in initialize_MCU: {}".format(e))

def deinitialize_MCU():
    try:
        global d1, MCU, MCU_initialized
        MCU.vehicle.close()
        MCU = None
        d1 = None
        MCU_initialized = False
        
    except Exception as e:
        log("MCU_Host: Error in deinitialize_MCU: {}".format(e))

##########################################################################################################################
log("MCU Server started, have fun!")

while True:
    socks = dict(poller.poll())

    if msg_socket in socks and socks[msg_socket] == zmq.POLLIN:
        try:
            immediate_command_str = msg_socket.recv(zmq.NOBLOCK)
            immediate_command_str = immediate_command_str.decode()
            command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
            command_thread.start()

        except zmq.error.Again:  # Handle non-blocking recv errors
            pass  # Wait for next poll event

        except zmq.ZMQError as zmq_error:
            log("ZMQ Error: {}".format(zmq_error))
            msg_socket.close()  # Recreate socket on ZMQ errors
            msg_socket = context.socket(zmq.PULL)
            msg_socket.bind("tcp://*:12345")
            poller.register(msg_socket, zmq.POLLIN)

        except Exception as e:
            log("Error: {}".format(e))

if KeyboardInterrupt:
    log("KeyboardInterrupt")
    msg_socket.close()

##########################################################################################################################
