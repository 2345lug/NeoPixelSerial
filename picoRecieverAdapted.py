import time
from neopixel import Neopixel
import os
import machine

### LED Strip Configuration ###

LED_COUNT = 8 # Number of LED pixels.
##LED_PIN = 18 # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
FIRST_STRIP_SHIFT = 0
##Serial port configuration##

SERIAL_BAUDRATE = 115200
##Color buffer global array##

colorsBuffer = ([0,0,0] * LED_COUNT)
pixels = Neopixel(LED_COUNT, 0, 22, "RGB")
pixels.brightness(30)

serialString = bytearray(b'')                           # Used to hold data coming over UART

serial1 = machine.UART(0, SERIAL_BAUDRATE)

def bytesToInt(byteValues, intValue):
    intValue = int.from_bytes(byteValues, "big")

def inputStringParse(inputString, colorsArray):
    for i in range(LED_COUNT):   
        colorsArray[i] = ((inputString[i * 3 + 1]), (inputString[i * 3 + 0]), (inputString[i * 3 + 2])) 

def setNeopixelData(colorsArray, strip, pixelShift):
    for i in range(LED_COUNT):
        strip.set_pixel(i + pixelShift, colorsArray[i])
    strip.show()
        #strip.setPixelColor(i + pixelShift, colorsArray[i])   


while(1):
 
    # Wait until there is data waiting in the serial buffer

    if(serial1.any()):
        # Read data out of the buffer until a carraige return / new line is found
        readedByte = serial1.read(1)
        # print(readedByte)
        serialString += readedByte
        
        if (readedByte == b'\n' and serialString[len(serialString) - 2] == 13 ):
            if (len(serialString) >= LED_COUNT*3):
                #print ("".join("\\x%02x" % i for i in serialString))
                inputStringParse(serialString, colorsBuffer)
                setNeopixelData(colorsBuffer, pixels, FIRST_STRIP_SHIFT)
            serialString = bytearray(b'')

     