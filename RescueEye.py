import time
import board
import RPi.GPIO as GPIO
import adafruit_dht
import spidev
from Adafruit_IO import Client, ThrottlingError

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
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
led_pwm = GPIO.PWM(LED_PIN, 100)
led_pwm.start(0)

# --- DHT11 setup ---
dht = adafruit_dht.DHT11(board.D21)

# --- SPI setup for MCP3008 ---
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def readadc(adcnum):
    if adcnum > 7 or adcnum < 0:
        return -1
    r = spi.xfer2([1, (8 + adcnum) << 4, 0])
    data = ((r[1] & 3) << 8) + r[2]
    return data

# --- UPDATED Classification functions ---

def classify_oxygen(value):
    """
    Human Safety Levels:
    Normal air is ~21%. 
    Below 19.5% is oxygen deficient.
    """
    if value < 16.0:
        return "DANGER"   # Critical low
    elif 16.0 <= value < 19.5:
        return "CAUTION"  # Low oxygen
    elif 19.5 <= value <= 23.5:
        return "SAFE"     # Normal range
    else:
        return "CAUTION"  # High oxygen (enrichment/fire risk)

def classify_temperature(temp):
    if temp is None: return "UNKNOWN"
    if temp <= 30:        # Adjusted for room comfort
        return "SAFE"
    elif temp <= 40:
        return "CAUTION"
    else:
        return "DANGER"

def classify_humidity(hum):
    if hum is None: return "UNKNOWN"
    # Normal indoor humidity is 30-60%
    if hum <= 60:
        return "SAFE"
    elif hum <= 80:
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
print("RescueEye: Human Safety Monitor Active...")

try:
    while True:
        try:
            # 1. Read Analog Sensors
            ldr_value = readadc(0)
            oxygen_raw = readadc(1)
            # Map potentiometer 0-1023 to a 0-25% oxygen scale for simulation
            oxygen_value = round((oxygen_raw / 1023) * 25, 2) 

            # 2. Read Digital Sensor (DHT11)
            try:
                temperature = dht.temperature
                raw_humidity = dht.humidity
            except RuntimeError as e:
                print(f"DHT Read Error: {e.args[0]}")
                time.sleep(2)
                continue 

            if raw_humidity is not None and temperature is not None:
                # APPLY HUMIDITY CORRECTION: Divide by 2
                humidity = round(raw_humidity / 2, 2)
                humidity = min(max(humidity, 0), 100) # Clamp 0-100

                # 3. Classification
                oxy_status = classify_oxygen(oxygen_value)
                temp_status = classify_temperature(temperature)
                hum_status = classify_humidity(humidity)
                final_status = overall_risk([oxy_status, temp_status, hum_status])

                # 4. LED Hardware Control
                if final_status == "SAFE":
                    led_pwm.ChangeDutyCycle(0)
                elif final_status == "CAUTION":
                    led_pwm.ChangeDutyCycle(30)
                else:
                    led_pwm.ChangeDutyCycle(100)

                # 5. Output
                print(f"\nO2: {oxygen_value}% [{oxy_status}] | Temp: {temperature}C | Hum: {humidity}%")
                print(f"OVERALL STATUS: {final_status}")

                # 6. Adafruit IO (12s delay to prevent Throttling)
                try:
                    aio.send(FEED_LDR, ldr_value)
                    aio.send(FEED_OXY, oxygen_value)
                    aio.send(FEED_TEMP, temperature)
                    aio.send(FEED_HUM, humidity)
                    aio.send(FEED_RISK, final_status)
                except ThrottlingError:
                    print("Cloud Warning: Rate limit hit.")
                except Exception as e:
                    print(f"Cloud Error: {e}")

            time.sleep(12)

        except Exception as e:
            print(f"Loop Error: {e}")
            time.sleep(2)

except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    led_pwm.stop()
    GPIO.cleanup()
    spi.close()
