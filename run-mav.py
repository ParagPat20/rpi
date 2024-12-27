from mav_functions import DroneVehicle, SerialHandler
import time
import threading

def main():
    # Initialize the drone with the appropriate connection string
    connection_string = '/dev/serial0'  # Change this to your actual connection string
    drone = DroneVehicle(connection_string)
    serial_handler = SerialHandler(drone)  # Initialize SerialHandler with the drone instance
    print("Drone initialized")

    # Start the serial handler in a separate thread
    serial_handler.is_running = True  # Set the running flag
    serial_thread = threading.Thread(target=serial_handler.read_serial_commands)
    serial_thread.start()

    try:
        while True:
            # Example command to send to the drone
            print("Sending message to drone")
            time.sleep(4)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        serial_handler.is_running = False  # Stop the serial handler
        drone.disconnect()  # Ensure the drone is disconnected properly

if __name__ == "__main__":
    main() 