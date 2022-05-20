# basic sensors integration
import serial
import time
import RPi.GPIO as GPIO
from time import sleep
import sys
import Adafruit_DHT

# Define pins and Ports
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
gpgga_info = "$GPGGA,"
GPGGA_buffer = 0
NMEA_buff = 0
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "“%.4f" % (position)
    return position


def get_tempdata():

    humidity, temperature = Adafruit_DHT.read_retry(11, 4)

    return humidity, temperature


def get_gpsdata():
    # NMEA = (str)(ser.readline())  # read NMEA string received
    # return NMEA
    received_data = (str)(ser.readline())  # read NMEA string received
    GPGGA_data_available = received_data.find(
        gpgga_info)  # check for NMEA GPGGA string
    if (GPGGA_data_available > 0):
        # store data coming after “$GPGGA,” string
        GPGGA_buffer = received_data.split("$GPGGA,", 1)[1]
        NMEA_buff = (GPGGA_buffer.split(','))
        nmea_time = []
        nmea_latitude = []
        nmea_longitude = []
        nmea_time = NMEA_buff[0]  # extract time from GPGGA string
        nmea_latitude = NMEA_buff[1]  # extract latitude from GPGGA string
        nmea_longitude = NMEA_buff[3]  # extract longitude from GPGGA string
        print("NMEA Time: ", nmea_time, "\n")
        lat = (float)(nmea_latitude)
        lat = convert_to_degrees(lat)
        longi = (float)(nmea_longitude)
        longi = convert_to_degrees(longi)

    return lat, longi


def get_distancedata():

    GPIO.output(GPIO_TRIGGER, True)  # set Trigger to HIGH
    time.sleep(0.00001)  # set Trigger after 0.01ms to LOW
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    while GPIO.input(GPIO_ECHO) == 0:  # save StartTime
        StartTime = time.time()

    while GPIO.input(GPIO_ECHO) == 1:  # save time of arrival
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime   # time difference between start and arrival
    # multiply with the sonic speed (34300 cm/s) and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


if __name__ == '__main__':
    try:
        while True:
            distance = get_distancedata()
            lat,longi = get_gpsdata()
            temperature, humidity = get_tempdata()
            print("Your Location =", lat , longi)
            print("Measured Distance = %.1f cm" % distance)
            print("Temperature = ", temperature)
            print("Humidity = ", humidity)
            time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Software Stopped!")
        GPIO.cleanup()
