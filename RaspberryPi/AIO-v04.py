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

# Initialize Adafruit IO feeds for each network ID
feed_config = {
    '001': {
        'SM': aio.feeds('001sm'),
        'ST': aio.feeds('001st'),
        'AT': aio.feeds('001at'),
        # 'AP': aio.feeds('001ap'),
        'AH': aio.feeds('001ah'),
        'RSSI': aio.feeds('001rssi'),
        # 'ErrorLED': aio.feeds('001errorled')
    },
    '002': {
        'SM': aio.feeds('002sm'),
        'ST': aio.feeds('002st'),
        'AT': aio.feeds('002at'),
        # 'AP': aio.feeds('002ap'),
        'AH': aio.feeds('002ah'),
        'RSSI': aio.feeds('002rssi'),
        # 'ErrorLED': aio.feeds('002errorled')
    }
}

# Initialize LoRa Radio with SPI and GPIO
spi = LoRaSpi(0, 0, 7800000)
cs = LoRaGpio(0, 8)
reset = LoRaGpio(0, 22)
LoRa = SX127x(spi, cs, reset)

print("Begin LoRa radio")
if not LoRa.begin():
    raise Exception("Something went wrong, can't begin LoRa radio")

# LoRa radio setup configuration
LoRa.setFrequency(433000000)
LoRa.setRxGain(LoRa.RX_GAIN_BOOSTED, LoRa.RX_GAIN_AUTO)
LoRa.setSpreadingFactor(12)
LoRa.setBandwidth(125000)
LoRa.setCodeRate(8)
LoRa.setHeaderType(LoRa.HEADER_EXPLICIT)
LoRa.setPreambleLength(8)
LoRa.setCrcEnable(True)
LoRa.setSyncWord(0x1424)

print("\n-- LoRa Receiver with Adafruit IO Integration --\n")

# Global variables for storing the received sensor data
current_network_id = "001"
receivedSM = receivedST = receivedAT = receivedAP = receivedAH = rssi_value = 0.0

def demodulate_data(message):
    """Parse the message and extract sensor data and network ID."""
    global current_network_id, receivedSM, receivedST, receivedAT, receivedAP, receivedAH

    try:
        data_parts = message.split(":")
        if len(data_parts) >= 6:
            current_network_id = data_parts[0]
            receivedSM = float(data_parts[1])
            receivedST = float(data_parts[2])
            receivedAT = float(data_parts[3])
            receivedAP = float(data_parts[4])
            receivedAH = float(data_parts[5])
    except Exception as e:
        print(f"Error in demodulating data: {e}")

def write_data_to_csv():
    """Store the received data into a CSV file with a timestamp."""
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    current_month = datetime.now().strftime("%Y_%m")
    file_name = f"data_{current_network_id}_{current_month}.csv"
    file_exists = os.path.isfile(file_name)

    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Timestamp", "SoilMoisture", "SoilTemp", "AirTemp", "AirPressure", "AirHumidity", "RSSI"])
        writer.writerow([current_time, receivedSM, receivedST, receivedAT, receivedAP, receivedAH, rssi_value])

def send_data_to_aio():
    """Send sensor data and RSSI to Adafruit IO based on network ID."""
    try:
        feeds = feed_config.get(current_network_id)
        if feeds:
            aio.send_data(feeds['SM'].key, receivedSM)
            aio.send_data(feeds['ST'].key, receivedST)
            aio.send_data(feeds['AT'].key, receivedAT)
            # aio.send_data(feeds['AP'].key, receivedAP)
            aio.send_data(feeds['AH'].key, receivedAH)
            aio.send_data(feeds['RSSI'].key, rssi_value)
            print(f"Data sent to Adafruit IO successfully for Network ID {current_network_id}.")
        else:
            print(f"No configured feeds for Network ID {current_network_id}.")
    except Exception as e:
        print(f"Error sending data to Adafruit IO: {e}")

def handle_errors(status):
    """Handle errors and light up an LED on the dashboard."""
    try:
        feeds = feed_config.get(current_network_id)
        if feeds:
            if status == LoRa.STATUS_CRC_ERR or status == LoRa.STATUS_HEADER_ERR:
                # aio.send_data(feeds['ErrorLED'].key, 1)
                print(f"Error occurred: Lighting up LED on Adafruit IO for Network ID {current_network_id}.")
            else:
                # aio.send_data(feeds['ErrorLED'].key, 0)
                pass
    except Exception as e:
        print(f"Error handling Adafruit IO LED status: {e}")

# Receive and process LoRa messages continuously
while True:
    LoRa.request()
    LoRa.wait()
    message = ""
    while LoRa.available() > 1:
        message += chr(LoRa.read())

    demodulate_data(message)
    rssi_value = LoRa.packetRssi()
    write_data_to_csv()
    send_data_to_aio()

    print(f"Received: {message}")
    print(f"Packet status: RSSI = {rssi_value:.2f} dBm | SNR = {LoRa.snr():.2f} dB")
    status = LoRa.status()
    handle_errors(status)
