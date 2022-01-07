import time
from neopixel import Neopixel
import os
import machine

### LED Strip Configuration ###

LED_COUNT = 8 # Number of LED pixels for one strip
OVERALL_LED_COUNT = 16
##LED_PIN = 18 # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
FIRST_STRIP_SHIFT = 0
SECOND_STRIP_SHIFT = 8
##Serial port configuration##

SERIAL_BAUDRATE = 115200
##Color buffer global array##

colorsBuffer1 = ([0,0,0] * LED_COUNT)
colorsBuffer2 = ([0,0,0] * LED_COUNT)

statusState1 = 0
statusState2 = 0

incomingStatus1 = 0
incomingStatus2 = 0

changeFlag1 = 0
changeFlag2 = 0

pixels = Neopixel(OVERALL_LED_COUNT, 0, 22, "RGB")
pixels.brightness(30)

serial1String = bytearray(b'')                           # Used to hold data coming over UART
serial2String = bytearray(b'')   

serial1 = machine.UART(0, SERIAL_BAUDRATE)
serial2 = machine.UART(0, SERIAL_BAUDRATE)

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

def doSerialRead (serialInstance, inputBuffer, incomingStatus, colorsBuffer, stripShift):
    if(serialInstance.any()):
        incomingStatus = 1
        # Read data out of the buffer until a carraige return / new line is found
        readedByte = serialInstance.read(1)
        # print(readedByte)
        inputBuffer += readedByte
        if (len(inputBuffer) > 255):
            inputBuffer = bytearray(b'')
        if (readedByte == b'\n' and inputBuffer[len(inputBuffer) - 2] == 13 ):
            if (len(inputBuffer) >= LED_COUNT*3):
                #print ("".join("\\x%02x" % i for i in serialString))
                inputStringParse(inputBuffer, colorsBuffer)
                setNeopixelData(colorsBuffer, pixels, stripShift)
            inputBuffer = bytearray(b'')

def waitIncomingData (incomingStatus, statusState, changeFlag, strip, stripShift):
    if (incomingStatus == 0):
        if (time.ticks_ms() % 500 == 0 and changeFlag == 0):
            if (statusState == 0):
                strip.set_pixel(stripShift + LED_COUNT - 1, [0,128,0])
                statusState = 1
                
            else:
                statusState = 0
                strip.set_pixel(stripShift + LED_COUNT - 1, [0,0,0])
            changeFlag = 1
        else: 
            changeFlag = 0

while(1):
 
    # Wait until there is data waiting in the serial buffer
    waitIncomingData(incomingStatus1, statusState1, changeFlag1, pixels, FIRST_STRIP_SHIFT)
    waitIncomingData(incomingStatus2, statusState2, changeFlag2, pixels, SECOND_STRIP_SHIFT)

    doSerialRead(serial1, serial1String, incomingStatus1, colorsBuffer1, FIRST_STRIP_SHIFT)
    doSerialRead(serial2, serial2String, incomingStatus2, colorsBuffer2, SECOND_STRIP_SHIFT)
    



     