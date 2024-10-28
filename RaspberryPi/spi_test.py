import spidev

# Open SPI bus 0, device 0
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500000

# Send a test byte
resp = spi.xfer2([0x00])
print(f"SPI Response: {resp}")
