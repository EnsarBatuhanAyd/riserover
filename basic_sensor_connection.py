# basic sensors integration
from email.errors import ObsoleteHeaderDefect
from xmlrpc.client import boolean
import pyrebase
# from firebase_admin import db
# from pyrebase import credentials
# from pyrebase import firestore
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
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

servoPin = 11
GPIO.setup(servoPin, GPIO.OUT)
servo = GPIO.PWM(servoPin, 50)
servo.start(7)

TRIG = 12
ECHO = 18
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
targets = {}


class Target:
    angle = -1
    objectex = 0
    distance = -1
    time = -1.0
    color = ()
  
    # initalization
    def __init__(self, angle, distance ,objectex):
        self.angle = angle
        self.distance = distance
        self.objectex = objectex
        self.time = time.time()
      
def ultrasonicRead(GPIO, TRIG, ECHO):
    
    # settling the sensor
    GPIO.output(TRIG, False)
    # time.sleep(0.01)
    # send a signal
    GPIO.output(TRIG, True)
    time.sleep(0.0001)
    GPIO.output(TRIG, False)
    start_time = time.time()

    # catch a signal
    error = 0
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        error += 1
        if error > 1000:
            break
    if error > 1000:
        return -1

    end_time = time.time()
    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    # calculate the distance 
    total_time = end_time - start_time
    distance = (34300 * total_time) / 2
    distance = round(distance, 2)
    # change the condition if the range is changed
    if distance <= 50:
        return distance
    else:
        return -1

def radar():
   
    for angle in range(0, 180):
            
        distance = ultrasonicRead(GPIO, TRIG, ECHO)
        
        # change the condition if the range is changed
        if distance != -1 and distance <= 50:
            targets[angle] = Target(angle, distance , Target.objectex) 
            Target.objectex = 1
            

            
        print(angle, distance , Target.objectex)
        angle = 180 - angle
        dc = 1.0 / 18.0 * angle + 2
        servo.ChangeDutyCycle(dc)
        time.sleep(0.001)
            

        # rotate from 180 to 0
    for angle in range(180, 0, -1):
            
        distance = ultrasonicRead(GPIO, TRIG, ECHO)
        
        # change the condition if the range is changed
        if distance != -1 and distance <= 50:
            targets[angle] = Target(angle, distance)
            Target.objectex = 1
            
           
        
        print( angle, distance)

        angle = 180 - angle
        dc = 1.0 / 18.0 * angle + 2
        servo.ChangeDutyCycle(dc)

        time.sleep(0.001)
    return angle , distance , Target.objectex
        # detect if close is pressed to stop the program
        
    
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "“%.4f" % (position)
    return position


def database_maindata():
    config = {
            "apiKey": "ky1tzNYylmxZw4rWV4zm50ZjMnMStcNn1yf6DIxA",
            "authDomain": "rover-maindata.firebaseapp.com",
            "databaseURL": "https://rover-maindata-default-rtdb.firebaseio.com/",
            "storageBucket": "rover-maindata.appspot.com"
    }


    # cred = credentials.ApplicationDefault()

    # cred = credentials.Certificate(
    # "./FirebaseCertificates/rover-maindata-firebase-adminsdk-poln2-9ee560d0a8.json")

    firebase= pyrebase.initialize_app(config)
    db = firebase.database() 
                                   
    data={
            "Latitude": lat,
            "Longitude": longi,
            "Humidity": humidity,
            "Temperature": temperature,
          }

    db.child("datas").child("1-set").set(data)
    db.child("datas").child("2-push").push(data)
    

    #     #Firestore Database Sending 
    # rf=db.childcollection("RoverMainData")
    # rf.add(data)                           
    return 

def database_radar():
    config = {
            "apiKey": "hTGQFFqFFfCT65ppUzA6MEJbYQf9GwKDKCwdUcrc",
            "authDomain": "rover-radardata.firebaseapp.com",
            "databaseURL": "https://rover-radardata-default-rtdb.firebaseio.com/",
            "storageBucket": "rover-radardata.appspot.com"
    }


    # cred = credentials.ApplicationDefault()

    # cred = credentials.Certificate(
    # "./FirebaseCertificates/rover-maindata-firebase-adminsdk-poln2-9ee560d0a8.json")

    firebase= pyrebase.initialize_app(config)
    db = firebase.database() 
                                   
    data={
            "Angle": angle,
            "Distance": distance,
            "ObjectEx": Target.objectex,
           
          }

    db.child("datas").child("1-set").set(data)
    db.child("datas").child("2-push").push(data)
    

    #     #Firestore Database Sending 
    # rf=db.childcollection("RoverMainData")
    # rf.add(data)                           
    return 


def get_tempdata():

    humidity, temperature = Adafruit_DHT.read_retry(11, 26)

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
    #gülüm buraya elsede ne yapılıcak bi onu giricen arkaya boş data yollama
    else:
        lat =0
        longi=0
    return lat, longi


# def get_distancedata():

#     GPIO.output(TRIG, True)  # set Trigger to HIGH
#     time.sleep(0.00001)  # set Trigger after 0.01ms to LOW
#     GPIO.output(TRIG, False)

#     StartTime = time.time()
#     StopTime = time.time()

#     while GPIO.input(ECHO) == 0:  # save StartTime
#         StartTime = time.time()

#     while GPIO.input(ECHO) == 1:  # save time of arrival
#         StopTime = time.time()

#     TimeElapsed = StopTime - StartTime   # time difference between start and arrival
#     # multiply with the sonic speed (34300 cm/s) and divide by 2, because there and back
#     distance = (TimeElapsed * 34300) / 2

#     return distance


if __name__ == '__main__':
    try:
        while True:
            # distance = get_distancedata()
            lat,longi = get_gpsdata()
            temperature, humidity = get_tempdata()
            angle , distance , objectex = radar()
            # Check all class 

            print("Your Location =", lat , longi)
            print("Temperature = ", temperature)
            print("Humidity = ", humidity)
            print("Angle = ", angle ,"Measured Distance = %.1f cm" % distance ,"Existence = ", objectex )
            # send main data database
            database_maindata()
            database_radar()
            # print("Radar = ", radarvalues) 
            # angle , distance , objex = radar()
            # print ("angle :" , angle ,"distance :" , distance ,"objex :" , objex)
            # # send radar database they will merge with image processing
            # database_radar(angle,distance,objex)

            time.sleep(1)
           

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        # servo.stop()
        GPIO.cleanup()
        print("Software Stopped!")
        
