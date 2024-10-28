import os
import sys
import time
import csv
from datetime import datetime
from Adafruit_IO import Client, Feed, Data  # Import Adafruit IO
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX127x, LoRaSpi, LoRaGpio

# Adafruit IO key and username
ADAFRUIT_AIO_USERNAME = "angusEE"
ADAFRUIT_AIO_KEY      = "aio_llGM99YxipMI5jtBPtSUACHkla00"

# Initialize an Adafruit IO Client
aio = Client(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY)

# Initialize Adafruit IO feeds for each sensor and RSSI
feeds = {
    '001SM': aio.feeds('001sm'),
    '001ST': aio.feeds('001st'),
    '001AT': aio.feeds('001at'),
    '001AH': aio.feeds('001ah'),
    '001RSSI': aio.feeds('001rssi')  # RSSI Feed
}

# Initialize LoRa Radio with SPI and GPIO
spi = LoRaSpi(0, 0, 7800000)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 22)
LoRa = SX127x(spi, cs, reset)

print("Begin LoRa radio")
if not LoRa.begin():
    raise Exception("Something went wrong, can't begin LoRa radio")

# Set LoRa frequency to 433 MHz
print("Set frequency to 433 MHz")
LoRa.setFrequency(433000000)

# Set RX gain to boosted and automatic
print("Set RX gain to Boosted gain")
LoRa.setRxGain(LoRa.RX_GAIN_BOOSTED, LoRa.RX_GAIN_AUTO)

# Configure modulation parameters (Spreading Factor, Bandwidth, Coding Rate)
print("Set modulation parameters:\n\tSpreading factor = 12\n\tBandwidth = 125 kHz\n\tCoding rate = 4/8")
LoRa.setSpreadingFactor(12)
LoRa.setBandwidth(125000)
LoRa.setCodeRate(8)

# Configure packet parameters
print("Set packet parameters:\n\tImplicit header type\n\tPreamble length = 8\n\tCRC on")
LoRa.setHeaderType(LoRa.HEADER_EXPLICIT)
LoRa.setPreambleLength(8)
LoRa.setCrcEnable(True)

# Set sync word
print("Set synchronize word to 0x1424")
LoRa.setSyncWord(0x1424)

print("\n-- LoRa Receiver with Adafruit IO Integration --\n")

# Global variables for storing the received sensor data
receivedSM = 0.0
receivedST = 0.0
receivedAT = 0.0
receivedAH = 0.0
rssi_value = 0.0  # Variable to store RSSI value

def demodulate_data(message):
    """Parse the message and extract sensor data."""
    global receivedSM, receivedST, receivedAT, receivedAH

    # Split the message using ':' as delimiter (Format: $NID:SM:ST:AT:AP:AH)
    try:
        data_parts = message.split(":")
        if len(data_parts) >= 6:
            receivedSM = float(data_parts[1])  # Soil Moisture
            receivedST = float(data_parts[2])  # Soil Temperature
            receivedAT = float(data_parts[3])  # Air Temperature
            receivedAH = float(data_parts[5])  # Air Humidity
    except Exception as e:
        print(f"Error in demodulating data: {e}")

def write_data_to_csv():
    """Store the received data into a CSV file with a timestamp."""
    # Get current timestamp
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Get current month for the CSV file
    current_month = datetime.now().strftime("%Y_%m")
    file_name = f"data_{current_month}.csv"

    # Check if the file exists (to add headers if it's a new file)
    file_exists = os.path.isfile(file_name)

    # Open the CSV file in append mode
    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write the header if the file is new
        if not file_exists:
            writer.writerow(["Timestamp", "SoilMoisture", "SoilTemp", "AirTemp", "AirHumidity", "RSSI"])

        # Write the data row with a timestamp
        writer.writerow([current_time, receivedSM, receivedST, receivedAT, receivedAH, rssi_value])

def send_data_to_aio():
    """Send sensor data and RSSI to Adafruit IO."""
    try:
        # Send each sensor value to its respective feed, excluding AP and Error LED
        aio.send_data(feeds['001SM'].key, receivedSM)
        aio.send_data(feeds['001ST'].key, receivedST)
        aio.send_data(feeds['001AT'].key, receivedAT)
        aio.send_data(feeds['001AH'].key, receivedAH)

        # Send RSSI value to Adafruit IO
        aio.send_data(feeds['001RSSI'].key, rssi_value)

        print("Data sent to Adafruit IO successfully.")
    except Exception as e:
        print(f"Error sending data to Adafruit IO: {e}")

# Receive and process LoRa messages continuously
while True:
    # Request for receiving new LoRa packet
    LoRa.request()
    # Wait for incoming LoRa packet
    LoRa.wait()

    # Put received packet to message variable
    message = ""
    while LoRa.available() > 1:
        message += chr(LoRa.read())

    # Demodulate the received message
    demodulate_data(message)

    # Capture RSSI and store it in the global variable
    rssi_value = LoRa.packetRssi()

    # Write the demodulated data to CSV
    write_data_to_csv()

    # Send the demodulated data and RSSI to Adafruit IO
    send_data_to_aio()

    # Print the received message and status
    print(f"Received: {message}")
    print(f"Packet status: RSSI = {rssi_value:.2f} dBm | SNR = {LoRa.snr():.2f} dB")
