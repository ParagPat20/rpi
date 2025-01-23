from mav_functions import DroneVehicle, SerialHandler, LogBook
import time
import threading

def main():
    # Initialize the logbook
    logbook = LogBook()
    logbook.log_event("SYSTEM", "Starting drone system")
    
    connection_string = '/dev/serial0'  # Change this to your actual connection string
    logbook.log_event("CONFIG", f"Connection string: {connection_string}")
    
    drone = DroneVehicle(connection_string)
    
    # Try different serial ports until one succeeds
    ports = [f'/dev/ttyUSB{i}' for i in range(4)] + [f'/dev/ttyACM{i}' for i in range(4)]
    serial_handler = None
    
    for port in ports:
        try:
            serial_handler = SerialHandler(drone, port, 115200)
            print(f"Successfully connected to {port}")
            break
        except:
            print(f"Failed to connect to {port}")
            continue
            
    if not serial_handler:
        raise Exception("Could not connect to any serial port")
    logbook.log_event("CONFIG", "Serial handler initialized on /dev/ttyUSB0 at 115200 baud")
    print("Drone initialized")
    serial_handler.start()  # This will start both serial reading and failsafe monitoring threads
    logbook.log_event("SYSTEM", "Serial handler started with all monitoring threads")

    try:
        while True:
            if drone.vehicle:  # Only send messages if vehicle is connected
                # Send heartbeat every second
                serial_handler.send_message('GCS', 'HB', '1')
                print("Heartbeat sent")

            time.sleep(1)

    except KeyboardInterrupt:
        logbook.log_event("SYSTEM", "Received keyboard interrupt, shutting down")
        print("Shutting down...")
    except Exception as e:
        logbook.log_event("ERROR", f"Unexpected error in main loop: {str(e)}")
    finally:
        serial_handler.is_running = False  # Stop all threads
        logbook.log_event("SYSTEM", "Stopping all threads")
        drone.disconnect()  # Ensure the drone is disconnected properly
        logbook.log_event("SYSTEM", "Drone disconnected, shutdown complete")

if __name__ == "__main__":
    main() 