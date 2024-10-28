import os
import csv
from datetime import datetime
from Adafruit_IO import Client, Feed
from LoRaRF import SX127x, LoRaSpi, LoRaGpio

# Adafruit IO credentials
ADAFRUIT_AIO_USERNAME = "angusEE"
ADAFRUIT_AIO_KEY = "aio_llGM99YxipMI5jtBPtSUACHkla00"

# Initialize Adafruit IO client
aio = Client(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY)

# Define the feeds for both devices
feeds = {
    '001SM': aio.feeds('001sm'),
    '001ST': aio.feeds('001st'),
    '001AT': aio.feeds('001at'),
    '001AH': aio.feeds('001ah'),
    '001RSSI': aio.feeds('001rssi'),
    '002SM': aio.feeds('002sm'),
    '002ST': aio.feeds('002st'),
    '002AT': aio.feeds('002at'),
    '002AH': aio.feeds('002ah'),
    '002RSSI': aio.feeds('002rssi')
}

# Initialize LoRa module with SPI and GPIO
spi = LoRaSpi(0, 0, 7800000)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 22)
LoRa = SX127x(spi, cs, reset)

# LoRa setup
print("Begin LoRa radio")
if not LoRa.begin():
    raise Exception("Unable to start LoRa radio")

LoRa.setFrequency(433000000)  # Set frequency to 433 MHz
LoRa.setRxGain(LoRa.RX_GAIN_BOOSTED, LoRa.RX_GAIN_AUTO)
LoRa.setSpreadingFactor(12)  # SF12 for maximum range
LoRa.setBandwidth(125000)  # 125 kHz bandwidth
LoRa.setCodeRate(8)  # Coding rate 4/8
LoRa.setHeaderType(LoRa.HEADER_EXPLICIT)
LoRa.setPreambleLength(8)
LoRa.setCrcEnable(True)
LoRa.setSyncWord(0x1424)

print("\n-- LoRa Receiver with Adafruit IO Integration --\n")

# Store last received data to detect changes
last_data = {}

def demodulate_data(message):
    """Parse the message and extract sensor data based on NID."""
    try:
        parts = message.split(":")
        if len(parts) < 6:
            return None

        nid = parts[0].strip("$")  # Get the device ID
        data = {
            'SM': float(parts[1]),
            'ST': float(parts[2]),
            'AT': float(parts[3]),
            'AP': float(parts[4]),
            'AH': float(parts[5])
        }
        return nid, data
    except Exception as e:
        print(f"Error parsing message: {e}")
        return None

def send_data_to_aio(nid, data, rssi):
    """Send parsed sensor data to Adafruit IO for the specific NID if data is new."""
    global last_data
    # Only upload if the data is different from the last received data
    if last_data.get(nid) != (data, rssi):
        try:
            # Select the correct feed prefix based on NID
            prefix = f'{nid.zfill(3)}'
            aio.send_data(feeds[f'{prefix}SM'].key, data['SM'])
            aio.send_data(feeds[f'{prefix}ST'].key, data['ST'])
            aio.send_data(feeds[f'{prefix}AT'].key, data['AT'])
            aio.send_data(feeds[f'{prefix}AH'].key, data['AH'])
            aio.send_data(feeds[f'{prefix}RSSI'].key, rssi)
            print(f"Data from NID {nid} sent to Adafruit IO successfully.")
            # Update last_data to the current data
            last_data[nid] = (data, rssi)
        except Exception as e:
            print(f"Error sending data to Adafruit IO: {e}")

def write_data_to_csv(nid, data, rssi):
    """Store the received data into a CSV file with a timestamp."""
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    current_month = datetime.now().strftime("%Y_%m")
    file_name = f"data_{nid}_{current_month}.csv"
    file_exists = os.path.isfile(file_name)

    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Timestamp", "SoilMoisture", "SoilTemp", "AirTemp", "AirPressure", "AirHumidity", "RSSI"])
        writer.writerow([current_time, data['SM'], data['ST'], data['AT'], data['AP'], data['AH'], rssi])

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

    # Parse the message and get RSSI
    parsed = demodulate_data(message)
    rssi_value = LoRa.packetRssi()  # Capture RSSI of the received packet

    # Process the parsed data if it is valid
    if parsed:
        nid, data = parsed
        write_data_to_csv(nid, data, rssi_value)  # Write to CSV
        send_data_to_aio(nid, data, rssi_value)  # Send to Adafruit IO

    # Additional delay or logic for continuous listening can be added here...
