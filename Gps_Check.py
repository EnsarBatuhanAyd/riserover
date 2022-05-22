import serial
import time


ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
gpgga_info = "$GPGGA,"
GPGGA_buffer = 0
NMEA_buff = 0

def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "“%.4f" % (position)
    return position

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


if __name__ == '__main__':
    try:
        while True:
            lat,longi = get_gpsdata()
            print("Your Location =", lat , longi)
            time.sleep(1)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Software Stopped!")
     
