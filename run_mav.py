from mav_functions import DroneVehicle, SerialHandler, LogBook
import time
import threading

def main():
    # Initialize the logbook
    logbook = LogBook()
    logbook.log_event("SYSTEM", "Starting drone system")
    
    # Initialize the drone with the appropriate connection string
    connection_string = '/dev/serial0'  # Change this to your actual connection string
    logbook.log_event("CONFIG", f"Connection string: {connection_string}")
    
    drone = DroneVehicle(connection_string)
    
    # Function to reinitialize drone if needed
    def reinit_drone():
        logbook.log_event("REINIT", "Attempting to reinitialize drone")
        return drone.init_vehicle()
    
    serial_handler = SerialHandler(drone, '/dev/ttyUSB0', 115200)  # Initialize SerialHandler with the drone instance
    logbook.log_event("CONFIG", "Serial handler initialized on /dev/ttyUSB0 at 115200 baud")
    print("Drone initialized")

    # Start the serial handler in a separate thread
    serial_handler.is_running = True  # Set the running flag
    serial_thread = threading.Thread(target=serial_handler.read_serial_commands)
    serial_thread.start()
    logbook.log_event("THREAD", "Serial handler thread started")

    # Define a function to check altitude and log drone status in a separate thread
    def check_altitude():
        while serial_handler.is_running:
            if drone.vehicle:  # Only check if vehicle is connected
                try:
                    current_altitude = drone.vehicle.location.global_relative_frame.alt
                    current_location = drone.vehicle.location.global_relative_frame
                    battery = drone.vehicle.battery
                    mode = drone.vehicle.mode.name
                    
                    # Log comprehensive drone status every 10 seconds
                    if int(time.time()) % 10 == 0:
                        status = (
                            f"Alt: {current_altitude:.2f}m, "
                            f"Loc: ({current_location.lat:.6f}, {current_location.lon:.6f}), "
                            f"Battery: {battery.voltage:.2f}V ({battery.level}%), "
                            f"Mode: {mode}"
                        )
                        logbook.log_event("STATUS", status)
                    
                    if current_altitude > 8:
                        logbook.log_event("SAFETY", f"Altitude exceeded 8 meters (Current: {current_altitude:.2f}m)")
                        print(f"Altitude exceeded 8 meters (Current: {current_altitude:.2f}m), initiating landing...")
                        
                        # Change to LAND mode and wait for confirmation
                        drone.vehicle.mode = drone.Vehicle.LAND
                        logbook.log_event("COMMAND", "Setting mode to LAND")
                        
                        # Monitor landing
                        while current_altitude > 0.3:  # Wait until we're close to the ground
                            current_altitude = drone.vehicle.location.global_relative_frame.alt
                            logbook.log_event("STATUS", f"Landing in progress. Current altitude: {current_altitude:.2f}m")
                            time.sleep(1)
                        
                        serial_handler.is_running = False  # Stop the serial handler
                        logbook.log_event("SAFETY", "Landing completed")
                        break  # Exit the loop after landing
                except Exception as e:
                    if int(time.time()) % 30 == 0:  # Log errors every 30 seconds to avoid spam
                        logbook.log_event("ERROR", f"Error reading drone status: {str(e)}")
                    pass  # Ignore if vehicle is temporarily disconnected
            time.sleep(1)

    # Start the altitude checking thread
    altitude_thread = threading.Thread(target=check_altitude)
    altitude_thread.start()
    logbook.log_event("THREAD", "Altitude monitoring thread started")

    try:
        heartbeat_count = 0
        while True:
            if drone.vehicle:  # Only send messages if vehicle is connected
                heartbeat_count += 1
                if heartbeat_count % 60 == 0:  # Log heartbeat every 60 seconds
                    logbook.log_event("HEARTBEAT", f"Heartbeat count: {heartbeat_count}")
                print("Sending message to drone")
                serial_handler.send_message('GCS', 'HB', '1')
            time.sleep(1)

    except KeyboardInterrupt:
        logbook.log_event("SYSTEM", "Received keyboard interrupt, shutting down")
        print("Shutting down...")
    except Exception as e:
        logbook.log_event("ERROR", f"Unexpected error in main loop: {str(e)}")
    finally:
        serial_handler.is_running = False  # Stop the serial handler
        logbook.log_event("SYSTEM", "Stopping serial handler")
        drone.disconnect()  # Ensure the drone is disconnected properly
        logbook.log_event("SYSTEM", "Drone disconnected, shutdown complete")

if __name__ == "__main__":
    main() 