import time
import board
import busio
import digitalio
import RPi.GPIO as GPIO
import adafruit_dht
from adafruit_mcp3xxx.mcp3008 import MCP3008
from adafruit_mcp3xxx.analog_in import AnalogIn
from Adafruit_IO import Client


# ADAFRUIT IO CONFIG
ADAFRUIT_IO_USERNAME = "User"
ADAFRUIT_IO_KEY = "Key"

aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Feed names
FEED_RISK = "risk-status"
FEED_TEMP = "temperature"
FEED_HUM = "humidity"
FEED_OXY = "oxygen"


# GPIO SETUP
LED_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

led_pwm = GPIO.PWM(LED_PIN, 100)
led_pwm.start(0)


# DHT11 SETUP
dht = adafruit_dht.DHT11(board.D21)


# MCP3008 ADC SETUP
spi = busio.SPI(clock=board.SCK,
                MISO=board.MISO,
                MOSI=board.MOSI)

cs = digitalio.DigitalInOut(board.D8)  # CE0
mcp = MCP3008(spi, cs)

potentiometer = AnalogIn(mcp, MCP3008.P1)  # CH1


# CLASSIFICATION FUNCTIONS
def classify_oxygen(value):
    if value <= 340:
        return "SAFE"
    elif value <= 680:
        return "CAUTION"
    else:
        return "DANGER"

def classify_temperature(temp):
    if temp <= 35:
        return "SAFE"
    elif temp <= 50:
        return "CAUTION"
    else:
        return "DANGER"

def classify_humidity(hum):
    if hum <= 70:
        return "SAFE"
    elif hum <= 85:
        return "CAUTION"
    else:
        return "DANGER"

def overall_risk(statuses):
    if "DANGER" in statuses:
        return "DANGER"
    elif "CAUTION" in statuses:
        return "CAUTION"
    else:
        return "SAFE"


# MAIN LOOP
try:
    while True:
        try:
            # Read sensors
            oxygen_value = potentiometer.value >> 6  # Scale to 0–1023
            temperature = dht.temperature
            humidity = dht.humidity

            # Classify each
            oxy_status = classify_oxygen(oxygen_value)
            temp_status = classify_temperature(temperature)
            hum_status = classify_humidity(humidity)

            final_status = overall_risk(
                [oxy_status, temp_status, hum_status]
            )

            # LED behaviour
            if final_status == "SAFE":
                led_pwm.ChangeDutyCycle(0)
            elif final_status == "CAUTION":
                led_pwm.ChangeDutyCycle(30)
            else:
                led_pwm.ChangeDutyCycle(100)

            # Terminal output
            print("----------------------------")
            print(f"Oxygen ADC : {oxygen_value} → {oxy_status}")
            print(f"Temp (°C)  : {temperature} → {temp_status}")
            print(f"Humidity % : {humidity} → {hum_status}")
            print(f"OVERALL    : {final_status}")
            print("----------------------------")

            # Send to Adafruit IO
            aio.send(FEED_OXY, oxygen_value)
            aio.send(FEED_TEMP, temperature)
            aio.send(FEED_HUM, humidity)
            aio.send(FEED_RISK, final_status)

        except RuntimeError as error:
            print("Sensor read error:", error)

        time.sleep(5)

except KeyboardInterrupt:
    print("Program stopped")

finally:
    led_pwm.stop()
    GPIO.cleanup()
