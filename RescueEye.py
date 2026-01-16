import time
import spidev
import Adafruit_DHT
import RPi.GPIO as GPIO
from Adafruit_IO import Client

# ======================
# ADAFRUIT IO SETTINGS
# ======================
ADAFRUIT_IO_USERNAME = "YOUR_USERNAME"
ADAFRUIT_IO_KEY = "YOUR_AIO_KEY"

aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Feed names (create later)
FEED_TEMP = "temperature"
FEED_HUMIDITY = "humidity"
FEED_OXYGEN = "oxygen-level"
FEED_STATUS = "risk-status"

# ======================
# GPIO SETUP
# ======================
GPIO.setmode(GPIO.BCM)

RED_LED = 18   # PWM pin
GPIO.setup(RED_LED, GPIO.OUT)
red_pwm = GPIO.PWM(RED_LED, 1000)
red_pwm.start(0)

# ======================
# DHT11 SETUP
# ======================
DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 21

# ======================
# MCP3008 SETUP
# ======================
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI0, CE0
spi.max_speed_hz = 1350000

def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) | adc[2]
    return data  # 0 - 1023

# ======================
# MAIN LOOP
# ======================
try:
    while True:
        # Read sensors
        humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
        oxygen_sim = read_adc(1)  # Potentiometer on CH1

        if humidity is None or temperature is None:
            print("Failed to read DHT sensor")
            time.sleep(2)
            continue

        # ------------------
        # RISK LOGIC
        # ------------------
        status = "SAFE"
        pwm_level = 0

        if oxygen_sim < 300 or temperature > 45:
            status = "DANGER"
            pwm_level = 100
        elif oxygen_sim < 500 or temperature > 35:
            status = "CAUTION"
            pwm_level = 30

        red_pwm.ChangeDutyCycle(pwm_level)

        # ------------------
        # TERMINAL OUTPUT
        # ------------------
        print("Temp:", temperature, "°C")
        print("Humidity:", humidity, "%")
        print("Oxygen(sim):", oxygen_sim)
        print("STATUS:", status)
        print("-" * 30)

        # ------------------
        # SEND TO ADAFRUIT
        # ------------------
        aio.send(FEED_TEMP, temperature)
        aio.send(FEED_HUMIDITY, humidity)
        aio.send(FEED_OXYGEN, oxygen_sim)
        aio.send(FEED_STATUS, status)

        time.sleep(5)

except KeyboardInterrupt:
    print("Exiting...")
    red_pwm.stop()
    GPIO.cleanup()
    spi.close()
