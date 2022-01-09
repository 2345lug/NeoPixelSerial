import time
from neopixel import Neopixel
import os
import machine

### LED Strip Configuration ###

LED_COUNT = 8 # Number of LED pixels.
##LED_PIN = 18 # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
FIRST_STRIP_SHIFT = 0
SECOND_STRIP_SHIFT = 8
STRIPS_COUNT = 2
##Serial port configuration##

SERIAL_BAUDRATE = 115200
##Color buffer global array##

colorsBuffer1 = ([0,0,0] * LED_COUNT)
colorsBuffer2 = ([0,0,0] * LED_COUNT)

pixels = Neopixel(LED_COUNT * STRIPS_COUNT, 0, 22, "RGB")
pixels.brightness(30)

serialString1 = bytearray(b'')                           # Used to hold data coming over UART
serialString2 = bytearray(b'') 

serial1 = machine.UART(0, SERIAL_BAUDRATE)
serial2 = machine.UART(1, SERIAL_BAUDRATE)

serial1DataStatus = 0
serial2DataStatus = 0

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

    if (time.ticks_ms() % 500):
        if (serial1DataStatus == 0):
            pixels.set_pixel(LED_COUNT - 1, [0, 0, 255])
        if (serial2DataStatus == 0):
            pixels.set_pixel(LED_COUNT * 2 - 1, [0, 0, 255])

    if (time.ticks_ms() % 1000):
        if (serial1DataStatus == 0):
            pixels.set_pixel(LED_COUNT - 1, [0, 0, 0])
        if (serial2DataStatus == 0):
            pixels.set_pixel(LED_COUNT * 2 - 1, [0, 0, 0])
    
    if (serial1DataStatus == 0 or serial2DataStatus == 0):
        pixels.show()

    if(serial1.any()):
        readedByte1 = serial1.read(1)
        serialString1 += readedByte1
        serial1DataStatus = 1
        if (readedByte1 == b'\n' and serialString1[len(serialString1) - 2] == 13 ):
            if (len(serialString1) >= LED_COUNT*3):
                #print ("".join("\\x%02x" % i for i in serialString1))
                inputStringParse(serialString1, colorsBuffer1)
                setNeopixelData(colorsBuffer1, pixels, FIRST_STRIP_SHIFT)
            serialString1 = bytearray(b'')

    if(serial2.any()):
        readedByte2 = serial2.read(1)
        serialString2 += readedByte2
        serial2DataStatus = 1
        if (readedByte2 == b'\n' and serialString2[len(serialString2) - 2] == 13 ):
            if (len(serialString2) >= LED_COUNT*3):
                #print ("".join("\\x%02x" % i for i in serialString2))
                inputStringParse(serialString2, colorsBuffer2)
                setNeopixelData(colorsBuffer2, pixels, SECOND_STRIP_SHIFT)
            serialString2 = bytearray(b'')





     