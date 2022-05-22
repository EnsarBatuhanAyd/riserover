import serial
import time
import RPi.GPIO as GPIO

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


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

            print("Measured Distance = %.1f cm" % distance)

            time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Software Stopped!")
        GPIO.cleanup()
