import BlynkLib
import time

# Your Blynk Auth Token
BLYNK_AUTH = 'eh63hD77pBpRccIPZjCZ1rqBTS6qM5sG'

# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH, server="blynk.cloud")

# Function to handle Blynk connection
def blynk_connected():
    print("Connected to Blynk 2.0")
    print("Synchronizing virtual pins...")
    # You can request Blynk to sync virtual pin states when connected
    blynk.sync_virtual(0)  # Example to sync virtual pin 0

# Function to collect data from sensor and send it to the Blynk server
def send_sensor_data():
    try:
        # Placeholder for real sensor reading logic
        SM = read_sensor_value()  # Replace with actual sensor reading function
        print(f"Sending sensor value: {SM}")
        
        # Send the value to virtual pin 0
        blynk.virtual_write(0, SM)
    except Exception as e:
        print(f"Error sending data to Blynk: {e}")

# Example sensor reading function (replace with actual implementation)
def read_sensor_value():
    # Placeholder for actual sensor code
    return 11  # Replace with real sensor data

# Main loop
try:
    while True:
        # Run Blynk logic
        blynk.run()

        # Check if connected to Blynk
        if blynk.state == blynk.connect:
            blynk_connected()

        # Send sensor data
        send_sensor_data()

        time.sleep(5)  # Delay between each sensor data transmission
except KeyboardInterrupt:
    print("Exiting program")
except Exception as e:
    print(f"An error occurred: {e}")
