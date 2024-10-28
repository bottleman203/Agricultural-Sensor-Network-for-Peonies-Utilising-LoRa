import spidev
import RPi.GPIO as GPIO
import time

# Pin configuration based on the provided pinout
NSS_PIN = 8   # Chip select pin
DIO0_PIN = 4  # RX Done interrupt
RESET_PIN = 22 # Reset pin

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

# Define LORA symbol timeout to match STM32 setting
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

    # Set frequency to 433 MHz
    write_register(REG_FRF_MSB, 0x6C)
    write_register(REG_FRF_MID, 0x80)
    write_register(REG_FRF_LSB, 0x00)

    # Set PA configuration for max power (14 dBm)
    write_register(REG_PA_CONFIG, 0x8F)  # Set transmit power to maximum (adjust per region)

    # Enable LNA Boost
    write_register(REG_LNA, 0x23)

    # Set RX base address
    write_register(REG_FIFO_RX_BASE_ADDR, 0x00)
    write_register(REG_FIFO_ADDR_PTR, 0x00)

    # Set modem configurations
    # Bandwidth = 125 kHz, Coding Rate = 4/5, Spreading Factor = 12 (SF12)
    write_register(REG_MODEM_CONFIG_1, 0x72)  # BW = 125 kHz, CR = 4/5
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

def receive_packet():
    # Check for RX done interrupt
    irq_flags = read_register(REG_IRQ_FLAGS)
    if irq_flags & IRQ_RX_DONE_MASK:
        # Packet received
        current_addr = read_register(REG_FIFO_RX_CURRENT_ADDR)
        num_bytes = read_register(REG_RX_NB_BYTES)

        # Set FIFO address to current RX address
        write_register(REG_FIFO_ADDR_PTR, current_addr)

        # Read the received packet byte by byte
        packet = bytearray()
        for i in range(num_bytes):
            packet.append(read_register(REG_FIFO))

        # Clear the IRQ flags
        write_register(REG_IRQ_FLAGS, 0xFF)

        # Get RSSI value
        rssi = read_register(REG_PKT_RSSI_VALUE) - 157

        # Print the raw byte sequence for debugging
        print(f"Raw packet bytes: {packet}")

        # Decode the packet as a string to see if it starts with the expected character
        try:
            packet_str = packet.decode('utf-8', errors='ignore')
        except UnicodeDecodeError:
            packet_str = ''  # In case decoding fails, set an empty string

        # Debug: Display the packet string
        print(f"Decoded packet: {packet_str}")

        # Check if the decoded packet starts with the expected '$' character
        if packet_str.startswith('$'):
            # Remove the starting '$' and split by ':' to extract values
            data_values = packet_str[1:].split(':')

            # Debug: Display the parsed values
            print(f"Parsed values: {data_values}")

            # Ensure data length matches expected values (5)
            if len(data_values) == 5:
                SM = data_values[0]
                ST = data_values[1]
                AT = data_values[2]
                AP = data_values[3]
                AH = data_values[4]
                print(f"Parsed Data - SM: {SM}, ST: {ST}, AT: {AT}, AP: {AP}, AH: {AH}")
            else:
                print("Data format mismatch or incorrect length.")
        else:
            print("Packet does not start with the expected character.")

        # Reconfigure to continuous receive mode after handling the packet
        clear_fifo()
        write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

    elif irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK:
        # CRC error, clear the flags and reset buffer
        write_register(REG_IRQ_FLAGS, 0xFF)
        print("CRC error!")
        # Flush the FIFO to ensure no residual data
        clear_fifo()
        # Reconfigure to continuous receive mode
        write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

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
