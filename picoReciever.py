import time
from rpi_ws281x import *
import numpy as np
import serial

### LED Strip Configuration ###

LED_COUNT = 8 # Number of LED pixels.
LED_PIN = 18 # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000 # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10 # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 20 # Set to 0 for darkest and 255 for brightest
LED_INVERT = False # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0 # set to '1' for GPIOs 13, 19, 41, 45 or 53

##Serial port configuration##

SERIAL_PORT = '/dev/tnt1' #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0. TNT0 is test virtual serial port
SERIAL_BAUDRATE = 115200
SERIAL_PARITY = serial.PARITY_NONE
SERIAL_STOPBITS = serial.STOPBITS_ONE
SERIAL_BYTESIZE = serial.EIGHTBITS
SERIAL_TIMEOUT = 1



##Color buffer global array##

colorsBuffer = np.array([Color(0,0,0)] * LED_COUNT, dtype=np.uint32)

serialString = ""                           # Used to hold data coming over UART

serial1 = serial.Serial(
    port= SERIAL_PORT, 
    baudrate = SERIAL_BAUDRATE,
    parity=SERIAL_PARITY,
    stopbits=SERIAL_STOPBITS,
    bytesize=SERIAL_BYTESIZE,
    timeout=SERIAL_TIMEOUT
)   

def bytesToInt(byteValues, intValue):
    intValue = int.from_bytes(byteValues, "big")

def inputStringParse(inputString, colorsArray):
    for i in range(LED_COUNT):   
        colorsArray[i] = Color((inputString[i * 3 + 0]), (inputString[i * 3 + 1]), (inputString[i * 3 + 2])) 

def setNeopixelData(colorsArray, strip, pixelShift):
    for i in range(LED_COUNT):           
        strip.setPixelColor(i + pixelShift, colorsArray[i])   


while(1):
 
    # Wait until there is data waiting in the serial buffer
    if(serial1.in_waiting > 0):

        # Read data out of the buffer until a carraige return / new line is found
        serialString = serial1.readline()

        inputStringParse(serialString, colorsBuffer)

        