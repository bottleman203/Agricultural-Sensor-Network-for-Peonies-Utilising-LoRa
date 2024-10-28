import spidev
import RPi.GPIO as GPIO
import time
import os

# Global variables
success_count = 0
error_count = 0



# Pin configuration based on the provided pinout
NSS_PIN = 8   # Chip select pin
DIO0_PIN = 4  # RX Done interrupt
RESET_PIN = 22  # Reset pin

# SX1278 Registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0C
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_RSSI_VALUE = 0x1A
REG_PKT_SNR_VALUE = 0x19
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_SYMB_TIMEOUT_LSB = 0x1F
REG_PAYLOAD_LENGTH = 0x22
REG_FIFO_RX_CURRENT_ADDR = 0x25
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42

# SX1278 Modes
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_RX_CONTINUOUS = 0x85

# SX1278 IRQ Masks
IRQ_RX_DONE_MASK = 0x40
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20

# LoRa symbol timeout to match STM32 setting
LORA_SYMBOL_TIMEOUT = 0x05  # Symbol timeout set to 5 symbols

# SPI Configuration
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 500000  # Set SPI speed to 500kHz

def write_register(address, value):
    spi.xfer2([address | 0x80, value])

def read_register(address):
    return spi.xfer2([address & 0x7F, 0x00])[1]

def reset_lora():
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.1)

def setup_lora():
    # Reset the SX1278
    reset_lora()

    # Check the version
    version = read_register(REG_VERSION)
    if version != 0x12:
        print("SX1278 not found!")
        return False

    # Put in sleep mode
    write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)

    # Set frequency to 433 MHz (adjust according to region if needed)
    write_register(REG_FRF_MSB, 0x6C)  # MSB for 433 MHz
    write_register(REG_FRF_MID, 0x80)  # MID for 433 MHz
    write_register(REG_FRF_LSB, 0x00)  # LSB for 433 MHz

    # Set PA configuration for max power (14 dBm)
    write_register(REG_PA_CONFIG, 0x8F)  # Set transmit power to maximum (adjust per region)

    # Enable LNA Boost
    write_register(REG_LNA, 0x23)

    # Set RX base address
    write_register(REG_FIFO_RX_BASE_ADDR, 0x00)
    write_register(REG_FIFO_ADDR_PTR, 0x00)

    # Set modem configurations
    # Bandwidth = 125 kHz, Coding Rate = 4/8, Spreading Factor = 12 (SF12)
    write_register(REG_MODEM_CONFIG_1, 0x72)  # BW = 125 kHz, CR = 4/8
    write_register(REG_MODEM_CONFIG_2, 0xC4)  # SF = 12, CRC on
    write_register(REG_SYMB_TIMEOUT_LSB, LORA_SYMBOL_TIMEOUT)  # Set symbol timeout to match STM32

    # Set payload length to maximum (255 bytes)
    write_register(REG_PAYLOAD_LENGTH, 0xFF)

    # Set to continuous receive mode
    write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

    # Map DIO0 to RX done
    write_register(REG_DIO_MAPPING_1, 0x00)
    return True

def clear_fifo():
    """ Clears the FIFO by setting the FIFO address pointer to the RX base address """
    rx_base_addr = read_register(REG_FIFO_RX_BASE_ADDR)
    write_register(REG_FIFO_ADDR_PTR, rx_base_addr)

def get_csv_filename():
    """
    Generates the filename based on the current month and year.
    Format: Month_Year.csv (e.g., September_2024.csv)
    """
    current_time = time.localtime()
    month_year = time.strftime("%B_%Y", current_time)  # e.g., "September_2024"
    return f"{month_year}.csv"

def initialize_log_file():
    """
    Initializes the CSV log file with headers if it doesn't exist.
    """
    csv_file = get_csv_filename()  # Get the filename based on the current month and year
    if not os.path.exists(csv_file):
        with open(csv_file, "w") as file:
            # Write the CSV header
            file.write("Timestamp,Soil Moisture (SM),Soil Temp (ST),Air Temp (AT),Air Pressure (AP),Air Humidity (AH)\n")
    return csv_file

def log_data_to_csv(SM, ST, AT, AP, AH, rssi, snr):
    """
    Logs the sensor data along with RSSI and SNR to a CSV file with a timestamp.
    """
    csv_file = initialize_log_file()  # Get the current month's CSV file
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log_message = f"{timestamp},{SM},{ST},{AT},{AP},{AH},{rssi},{snr}\n"
    
    # Append the log message to the CSV file
    with open(csv_file, "a") as file:
        file.write(log_message)
    
    # Print to console for immediate feedback
    print(log_message)


def log_error_to_csv(error_message):
    """
    Logs an error to the CSV file with a timestamp.
    """
    csv_file = initialize_log_file()  # Get the current month's CSV file
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log_message = f"{timestamp},Error,{error_message}\n"
    
    # Append the log message to the CSV file
    with open(csv_file, "a") as file:
        file.write(log_message)
    
    # Print to console for immediate feedback
    print(log_message)

def reorder_packet(packet):
    """
    This function finds the '$' in the received data, reorders it so that the 
    data starts from '$', and then wraps around the remaining bytes.
    """
    # Convert packet to string (strip ignores non-printable chars)
    packet_str = packet.decode('utf-8', errors='ignore').strip()

    # Check if the '$' character is present in the packet
    if '$' in packet_str:
        start_idx = packet_str.find('$')  # Find the position of the '$'
        
        # Reorder the packet so that it starts at '$'
        reordered_packet = packet_str[start_idx:] + packet_str[:start_idx]
        
        # Return the reordered packet
        return reordered_packet
    else:
        # If no '$' is found, return the original packet
        return packet_str

def receive_packet():
    global success_count, error_count  # Use global counters

    # Check for RX done interrupt
    irq_flags = read_register(REG_IRQ_FLAGS)

        # Check for CRC error first
    if irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK:
        # CRC error, clear the flags and reset the buffer
        write_register(REG_IRQ_FLAGS, 0xFF)
        print("CRC error!")
        clear_fifo()
        write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)
        error_count += 1  # Increment error counter

        # Log the CRC error
        log_error_to_csv("CRC error")
        return  # Exit the function since data is corrupt
    
    if irq_flags & IRQ_RX_DONE_MASK:
        current_addr = read_register(REG_FIFO_RX_CURRENT_ADDR)
        num_bytes = read_register(REG_RX_NB_BYTES)

        # Set FIFO address to current RX address
        write_register(REG_FIFO_ADDR_PTR, current_addr)

        # Read the received packet byte by byte
        packet = bytearray()  # This creates a new empty buffer for each reception
        for i in range(num_bytes):
            packet.append(read_register(REG_FIFO))

        # Clear the IRQ flags
        write_register(REG_IRQ_FLAGS, 0xFF)

        # Retrieve RSSI and SNR
        rssi = read_register(REG_PKT_RSSI_VALUE) - 157  # Adjusted for LoRa offset
        snr = read_register(REG_PKT_SNR_VALUE)
        if snr > 127:  # SNR is a signed value
            snr = ((snr - 256) / 4.0)
        else:
            snr = (snr / 4.0)

        # Debug: Show the raw byte sequence and signal strength
        print(f"Raw packet bytes: {packet}")
        print(f"RSSI: {rssi} dBm, SNR: {snr} dB")

        # Reorder the packet if it's circular
        reordered_packet = reorder_packet(packet)

        # Debug: Display the reordered packet
        print(f"Reordered packet: {reordered_packet}")

        # After reordering, process the packet as before
        # Look for the start character '$'
        if reordered_packet.startswith('$'):
            # Remove the leading '$' for easier parsing
            data_values = reordered_packet[1:].split(':')

            # Debug: Display the parsed values
            print(f"Parsed values: {data_values}")

            if len(data_values) == 5:
                SM = data_values[0]
                ST = data_values[1]
                AT = data_values[2]
                AP = data_values[3]
                AH = data_values[4]
                parsed_data = f"SM: {SM}, ST: {ST}, AT: {AT}, AP: {AP}, AH: {AH}"
                print(f"Parsed Data - {parsed_data}")
                
                # Increment success counter
                success_count += 1
                
                # Log the success to CSV, including RSSI and SNR
                log_data_to_csv(SM, ST, AT, AP, AH, rssi, snr)
            else:
                print("Data format mismatch or incorrect length.")
                error_count += 1
                log_error_to_csv("Data format mismatch or incorrect length.")
        else:
            print("Start character '$' not found at the beginning.")
            error_count += 1
            log_error_to_csv("Start character '$' not found.")

        # Clear the FIFO after reading the received data
        clear_fifo()


    # Display current success and error counts
    print(f"Success count: {success_count}, Error count: {error_count}")


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(NSS_PIN, GPIO.OUT)
    GPIO.setup(DIO0_PIN, GPIO.IN)
    GPIO.setup(RESET_PIN, GPIO.OUT)

    if not setup_lora():
        print("Failed to initialize LoRa module")
        return

    print("LoRa Receiver Ready")

    try:
        while True:
            if GPIO.input(DIO0_PIN) == GPIO.HIGH:
                receive_packet()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        GPIO.cleanup()
        spi.close()

if __name__ == "__main__":
    main()