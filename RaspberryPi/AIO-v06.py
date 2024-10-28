import os
import csv
from datetime import datetime
from Adafruit_IO import Client
from LoRaRF import SX127x, LoRaSpi, LoRaGpio

# Adafruit IO key and username
ADAFRUIT_AIO_USERNAME = "angusEE"
ADAFRUIT_AIO_KEY = "aio_llGM99YxipMI5jtBPtSUACHkla00"

# Initialize an Adafruit IO Client
aio = Client(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY)

# Feeds dictionary for device 001 and device 002
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

# Initialize and configure LoRa module for receiving...
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
    """Send parsed sensor data to Adafruit IO for the specific NID."""
    try:
        # Select the correct feed prefix based on NID
        prefix = f'{nid.zfill(3)}'
        aio.send_data(feeds[f'{prefix}SM'].key, data['SM'])
        aio.send_data(feeds[f'{prefix}ST'].key, data['ST'])
        aio.send_data(feeds[f'{prefix}AT'].key, data['AT'])
        aio.send_data(feeds[f'{prefix}AH'].key, data['AH'])
        aio.send_data(feeds[f'{prefix}RSSI'].key, rssi)
        print(f"Data from NID {nid} sent to Adafruit IO successfully.")
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

# Placeholder for LoRa receiver loop
while True:
# Request for receiving new LoRa packet
    LoRa.request()
    # Wait for incoming LoRa packet
    LoRa.wait()

    # Put received packet to message variable
    message = ""
    while LoRa.available() > 1:
        message += chr(LoRa.read())


    rssi = LoRa.packetRssi()

    parsed = demodulate_data(message)
    if parsed:
        nid, data = parsed
        write_data_to_csv(nid, data, rssi)
        send_data_to_aio(nid, data, rssi)

    # Print the received message and status
    print(f"Received: {message}")
    print(f"Packet status: RSSI = {rssi:.2f} dBm | SNR = {LoRa.snr():.2f} dB")
    # Additional delay or logic for continuous listening...
