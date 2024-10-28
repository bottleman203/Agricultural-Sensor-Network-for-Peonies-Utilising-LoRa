import BlynkLib
import time
import sys

BLYNK_AUTH_TOKEN = 'eh63hD77pBpRccIPZjCZ1rqBTS6qM5sG'

# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)

# Function to sync data from virtual pins
@blynk.on("connected")
def blynk_connected():
    print("Connected to Blynk 2.0")
    print("Synchronizing virtual pins...")
    blynk.sync_virtual(0)  # Example to sync virtual pin 0

# Function to collect data from sensor and send it to the Blynk server
def send_sensor_data():
    try:
        # Placeholder for real sensor reading logic
        sensor_value = 10  # Replace with actual sensor data
        print(f"Sending sensor value: {sensor_value}")
        
        # Send the value to virtual pin 0
        blynk.virtual_write(0, sensor_value)
    except BrokenPipeError:
        print("Connection broken. Attempting to reconnect...")
        reconnect_blynk()
    except Exception as e:
        print(f"Error sending data to Blynk: {e}")

# Function to reconnect to Blynk
def reconnect_blynk():
    global blynk
    try:
        print("Reconnecting to Blynk...")
        blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)  # Reinitialize Blynk connection
        blynk.connect()  # Force reconnection
        print("Reconnected to Blynk!")
    except Exception as e:
        print(f"Failed to reconnect: {e}")
        time.sleep(5)  # Wait before trying to reconnect again

# Main loop
try:
    while True:
        blynk.run()
        send_sensor_data()
        time.sleep(1)  # Adjust the interval as needed
except KeyboardInterrupt:
    print("Exiting program")
    sys.exit(0)
except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
