from mav_functions import DroneVehicle, SerialHandler
import time
import threading

def main():
    # Initialize the drone with the appropriate connection string
    connection_string = '/dev/serial0'  # Change this to your actual connection string
    drone = DroneVehicle(connection_string)
    
    serial_handler = SerialHandler(drone, '/dev/ttyUSB0', 115200)  # Initialize SerialHandler with the drone instance
    print("Drone initialized")

    # Start the serial handler in a separate thread
    serial_handler.is_running = True  # Set the running flag
    serial_thread = threading.Thread(target=serial_handler.read_serial_commands)
    serial_thread.start()

    # Define a function to check altitude in a separate thread
    def check_altitude():
        while serial_handler.is_running:
            current_altitude = drone.vehicle.location.global_relative_frame.alt
            if current_altitude > 5:
                print("Altitude exceeded 5 meters, initiating landing...")
                drone.land()
                serial_handler.is_running = False  # Stop the serial handler
                break  # Exit the loop after landing
            time.sleep(1)

    # Start the altitude checking thread
    altitude_thread = threading.Thread(target=check_altitude)
    altitude_thread.start()

    try:
        while True:
            # Example command to send to the drone
            print("Sending message to drone")
            serial_handler.send_message('GCS', 'HB', '1')

            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        serial_handler.is_running = False  # Stop the serial handler
        drone.disconnect()  # Ensure the drone is disconnected properly

if __name__ == "__main__":
    main() 