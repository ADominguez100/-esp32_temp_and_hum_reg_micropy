"""
MicroPython IoT Weather Station Example for Wokwi.com

To view the data:

1. Go to http://www.hivemq.com/demos/websocket-client/
2. Click "Connect"
3. Under Subscriptions, click "Add New Topic Subscription"
4. In the Topic field, type "wokwi-weather" then click "Subscribe"

Now click on the DHT22 sensor in the simulation,
change the temperature/humidity, and you should see
the message appear on the MQTT Broker, in the "Messages" pane.

Copyright (C) 2022, Uri Shaked

https://wokwi.com/arduino/projects/322577683855704658
"""

"""


import network
import time
from machine import I2C, Pin
import dht
import ujson
from umqtt.simple import MQTTClient

import machine
from machine import Pin, SoftI2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep


# MQTT Server Parameters
MQTT_CLIENT_ID = "micropython-weather-demo"
MQTT_BROKER    = "broker.mqttdashboard.com"
MQTT_USER      = ""
MQTT_PASSWORD  = ""
MQTT_TOPIC     = "wokwi-weather"

sensor = dht.DHT22(Pin(15))


I2C_ADDR = 0x27
totalRows = 2
totalColumns = 16


i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)     #initializing the I2C method for ESP32
#i2c = I2C(scl=Pin(5), sda=Pin(4), freq=10000)       #initializing the I2C method for ESP8266

lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)




print("Connecting to WiFi", end="")
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('Wokwi-GUEST', '')
while not sta_if.isconnected():
  print(".", end="")
  time.sleep(0.1)
print(" Connected!")

print("Connecting to MQTT server... ", end="")
client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, user=MQTT_USER, password=MQTT_PASSWORD)
client.connect()

print("Connected!")

prev_weather = ""
while True:
  print("Measuring weather conditions... ", end="")
  sensor.measure() 
  message = ujson.dumps({
    "temp": sensor.temperature(),
    "humidity": sensor.humidity(),
  })
  if message != prev_weather:
    print("Updated!")
    print("Reporting to MQTT topic {}: {}".format(MQTT_TOPIC, message))
    client.publish(MQTT_TOPIC, message)
    prev_weather = message
  else:
    print("No change")
  

  lcd.putstr(message)
  lcd.clear()
  time.sleep(1)
"""

#все для MQTT
from machine import Pin
from umqtt.simple import MQTTClient
import ujson
import network
import utime as time
import dht

#https://www.hivemq.com/demos/websocket-client/


import machine
from machine import Pin, SoftI2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep

from machine import Pin
from time import sleep
import dht 


I2C_ADDR = 0x27
totalRows = 2
totalColumns = 16

#18-25
#20-60%

led1 = machine.Pin(2, machine.Pin.OUT)
led2 = machine.Pin(4, machine.Pin.OUT)

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)     #initializing the I2C method for ESP32
#i2c = I2C(scl=Pin(5), sda=Pin(4), freq=10000)       #initializing the I2C method for ESP8266

lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)


sensor = dht.DHT22(Pin(15))




#mqtt
# Device Setup
DEVICE_ID = "wokwi001"

# WiFi Setup
WIFI_SSID       = "Wokwi-GUEST"
WIFI_PASSWORD   = ""

# MQTT Setup
MQTT_BROKER             = "broker.mqttdashboard.com"
MQTT_CLIENT             = DEVICE_ID
MQTT_TELEMETRY_TOPIC    = "pavel"
MQTT_CONTROL_TOPIC      = "iot/control"

# DHT Sensor Setup
DHT_PIN = Pin(15)

# LED/LAMP Setup
RED_LED     = Pin(12, Pin.OUT)
BLUE_LED    = Pin(13, Pin.OUT)
FLASH_LED   = Pin(2, Pin.OUT)
RED_LED.on()
BLUE_LED.on()

# Methods
def did_recieve_callback(topic, message):
    print('\n\nData Recieved! \ntopic = {0}, message = {1}'.format(topic, message))

    # device_id/lamp/color/state
    # device_id/lamp/state
    # lamp/state
    if topic == MQTT_CONTROL_TOPIC.encode():
        if message == ('{0}/lamp/red/on'.format(DEVICE_ID)).encode():
            RED_LED.on()
        elif message == ('{0}/lamp/red/off'.format(DEVICE_ID)).encode():
            RED_LED.off()
        elif message == ('{0}/lamp/blue/on'.format(DEVICE_ID)).encode():
            BLUE_LED.on()
        elif message == ('{0}/lamp/blue/off'.format(DEVICE_ID)).encode():
            BLUE_LED.off()
        elif message == ('{0}/lamp/on'.format(DEVICE_ID)).encode() or message == b'lamp/on':
            RED_LED.on()
            BLUE_LED.on()
        elif message == ('{0}/lamp/off'.format(DEVICE_ID)).encode() or message == b'lamp/off':
            RED_LED.off()
            BLUE_LED.off()
        elif message == ('{0}/status'.format(DEVICE_ID)).encode() or message == ('status').encode():
            global telemetry_data_old
            mqtt_client_publish(MQTT_TELEMETRY_TOPIC, telemetry_data_old)
        else:
            return
        
        

def mqtt_connect():
    print("Connecting to MQTT broker ...", end="")
    mqtt_client = MQTTClient(MQTT_CLIENT, MQTT_BROKER, user="", password="")
    mqtt_client.set_callback(did_recieve_callback)
    mqtt_client.connect()
    print("Connected.")
    mqtt_client.subscribe(MQTT_CONTROL_TOPIC)
    return mqtt_client

def create_json_data(temperature, humidity):
    data = ujson.dumps({
        "temp": temperature,
        "humidity": humidity
    })
    return data

def mqtt_client_publish(topic, data):
    print("\nUpdating MQTT Broker...")
    mqtt_client.publish(topic, data)
    print(data)



# Application Logic

# Connect to WiFi
wifi_client = network.WLAN(network.STA_IF)
wifi_client.active(True)
print("Connecting device to WiFi")
wifi_client.connect(WIFI_SSID, WIFI_PASSWORD)

# Wait until WiFi is Connected
while not wifi_client.isconnected():
    print("Connecting")
    time.sleep(0.1)
print("WiFi Connected!")
print(wifi_client.ifconfig())

# Connect to MQTT
mqtt_client = mqtt_connect()
RED_LED.off()
BLUE_LED.off()
mqtt_client_publish(MQTT_CONTROL_TOPIC, 'lamp/off')
dht_sensor = dht.DHT22(DHT_PIN)
telemetry_data_old = ""
    













while True:
  try:
    sleep(2)
    sensor.measure()
    temp = sensor.temperature()
    hum = sensor.humidity()
    temp_f = temp * (9/5) + 32.0
    print('Temperature: %3.1f C' %temp)
    print('Humidity: %3.1f %%' %hum)
  except OSError as e:
    print('Failed to read sensor.')

  if temp < 18 or temp > 25:
    led1.on()
  else:
    led1.off()

  if hum < 20 or hum > 60:
    led2.on()
  else:
    led2.off()

  lcd.clear()
  lcd.putstr(f'Temp: {temp} \nHumidity: {hum}')
  sleep(1)

  
  mqtt_client.check_msg()
  print(". ", end="")

  try:
    dht_sensor.measure()
  except:
    pass


  telemetry_data_new = create_json_data(dht_sensor.temperature(), dht_sensor.humidity())

  if telemetry_data_new != telemetry_data_old:
      mqtt_client_publish(MQTT_TELEMETRY_TOPIC, telemetry_data_new)
      telemetry_data_old = telemetry_data_new

  time.sleep(0.1)