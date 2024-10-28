import time
from Adafruit_IO import Client, Feed, Data

# Adafruit IO key and username
ADAFRUIT_AIO_USERNAME = "angusEE"
ADAFRUIT_AIO_KEY      = "aio_llGM99YxipMI5jtBPtSUACHkla00"

# Initialize an Adafruit IO Client
aio = Client(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY)

# Define your feed name
FEED_NAME = ''

# Create the feed if it doesn't exist
try:
    feed = aio.feeds(FEED_NAME)
except Exception:
    # If the feed does not exist, create it
    feed = aio.create_feed(Feed(name=FEED_NAME))

# Function to read sensor data (replace with actual sensor code)
def read_sensor_value():
    # Placeholder: replace with code to read from your sensor
    return 25.0  # Example sensor value

# Main loop to send sensor data to Adafruit IO
try:
    while True:
        # Read the sensor value
        sensor_value = read_sensor_value()

        # Send the sensor value to the Adafruit IO feed
        print(f"Sending sensor value: {sensor_value}")
        aio.send_data(feed.key, sensor_value)

        # Wait 10 seconds before sending the next value
        time.sleep(10)
except KeyboardInterrupt:
    print("Program stopped")
