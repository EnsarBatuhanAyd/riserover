import serial
import time
import Adafruit_DHT


def get_tempdata():

    humidity, temperature = Adafruit_DHT.read_retry(11, 4)

    return humidity, temperature


if __name__ == '__main__':
    try:
        while True:
            temperature, humidity = get_tempdata()
           
            print("Temperature = ", temperature)
            print("Humidity = ", humidity)
            time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Software Stopped!")
 
