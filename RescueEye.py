import time
import board
import RPi.GPIO as GPIO
import adafruit_dht
import spidev
from Adafruit_IO import Client

# --- Adafruit IO setup ---
ADAFRUIT_IO_USERNAME = "User"
ADAFRUIT_IO_KEY = "Key"
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

FEED_RISK = "risk-status"
FEED_TEMP = "temperature"
FEED_HUM = "humidity"
FEED_OXY = "oxygen"
FEED_LDR = "light"

# --- GPIO setup ---
LED_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
led_pwm = GPIO.PWM(LED_PIN, 100)
led_pwm.start(0)

# --- DHT11 setup ---
dht = adafruit_dht.DHT11(board.D21)

# --- SPI setup for MCP3008 ---
spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0 (CE0)
spi.max_speed_hz = 1350000

def readadc(adcnum):
    """Read SPI data from MCP3008 channel (0–7)."""
    if adcnum > 7 or adcnum < 0:
        return -1
    r = spi.xfer2([1, (8 + adcnum) << 4, 0])
    data = ((r[1] & 3) << 8) + r[2]
    return data

# --- Classification functions ---
def classify_oxygen(value):
    if value <= 33:      # 0–33% safe
        return "SAFE"
    elif value <= 66:    # 34–66% caution
        return "CAUTION"
    else:                # 67–100% danger
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

# --- Main loop ---
try:
    while True:
        try:
            # Read sensors
            ldr_value = readadc(0)       # CH0 = LDR raw value
            oxygen_raw = readadc(1)      # CH1 = potentiometer raw value
            oxygen_value = round((oxygen_raw / 1023) * 100, 2)  # convert to 0–100 %

            temperature = dht.temperature
            humidity = dht.humidity
            if humidity is not None:
                humidity = min(max(humidity, 0), 100)  # clamp to 0–100

            # Classify each
            oxy_status = classify_oxygen(oxygen_value)
            temp_status = classify_temperature(temperature)
            hum_status = classify_humidity(humidity)

            final_status = overall_risk([oxy_status, temp_status, hum_status])

            # LED behaviour
            if final_status == "SAFE":
                led_pwm.ChangeDutyCycle(0)
            elif final_status == "CAUTION":
                led_pwm.ChangeDutyCycle(30)
            else:
                led_pwm.ChangeDutyCycle(100)

            # Terminal output
            print("----------------------------")
            print(f"LDR (CH0)   : {ldr_value}")
            print(f"Oxygen (CH1): {oxygen_value}% → {oxy_status}")
            print(f"Temp (°C)   : {temperature} → {temp_status}")
            print(f"Humidity %  : {humidity} → {hum_status}")
            print(f"OVERALL     : {final_status}")
            print("----------------------------")

            # Send to Adafruit IO
            aio.send(FEED_LDR, ldr_value)
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
    spi.close()

