import time
from rpi_ws281x import *
from gpiozero import Button
import struct
import serial
import numpy as np
import math
import subprocess
import threading


### LED Strip Configuration ###

LED_COUNT = 8 # Number of LED pixels.
LED_PIN = 18 # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000 # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10 # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 20 # Set to 0 for darkest and 255 for brightest
LED_INVERT = False # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0 # set to '1' for GPIOs 13, 19, 41, 45 or 53

### SD Card Monitor Configuration ###

STATUS_PIN = 13
#SD_STATUS = Button(STATUS_PIN)  #Commented for simulation purpose!
SD_LIGHT = 0

### JACK Audio Metering Configuration ###

MIN_DB = -40
#JACK_METER_PATH = "../jb/jackmeter-0.4/jack_meter"
JACK_METER_PATH = "/root/Livefeed-Encoder-Core/frontpanel/jack_meter"
JACK_METER_PARAMS = ["-n", "-f25"]
DEVICE_INPUT_LEFT = "system:capture_1"
DEVICE_INPUT_RIGHT = "system:capture_2"
DEVICE_OUTPUT_LEFT = "system:capture_1"
DEVICE_OUTPUT_RIGHT = "system:capture_2"
lastlevel = 0
leveldelta = 0
lastsignal = 0

### Program Running / Power Indicator ###

program_running = True
not_running = True

### Global Brightness Threshold ###

MAX_BRIGHTNESS_THRESHOLD = 1

### Ethernet Link Monitor Configuration ###
deltaTx = 0
last_valueTx = 0
currTx = 0
deltaRx = 0
last_valueRx = 0
currRx = 0
UPPER_THRESHOLD = 10

## LED MAPS and COLOURS Configuration ##

LED_MAP = {
    "program_running": 0,
    "ffmpeg": 1,
    "ch1_ip": 2,
    "ch2_ip": 3,
    "ch1_op": 4,
    "ch2_op": 5,
    "EthI": 6,
    "EthO": 7
}

LED_COLOR = {
    "program_running": [[150, 100, 100]],
    "ffmpeg": [[150, 100, 100]],
    "ch1_ip": [[150, 0, 100]],
    "ch2_ip": [[150, 0, 100]],
    "ch1_op": [[150, 0, 100]],
    "ch2_op": [[150, 0, 100]],
    "EthI": [[234, 100, 88]],
    "EthO": [[150, 100, 100]]
}

##Serial port configuration##

SERIAL_PORT = '/dev/tnt0' #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0. TNT0 is test virtual serial port
SERIAL_BAUDRATE = 115200
SERIAL_PARITY = serial.PARITY_NONE
SERIAL_STOPBITS = serial.STOPBITS_ONE
SERIAL_BYTESIZE = serial.EIGHTBITS
SERIAL_TIMEOUT = 1

serial1 = serial.Serial(
        port= SERIAL_PORT, 
        baudrate = SERIAL_BAUDRATE,
        parity=SERIAL_PARITY,
        stopbits=SERIAL_STOPBITS,
        bytesize=SERIAL_BYTESIZE,
        timeout=SERIAL_TIMEOUT
)

##Color buffer global array##

colorsBuffer = []

## Functions ##

def get_network_bytes(interface):
    for line in open('/proc/net/dev', 'r'):
        if interface in line:
            data = line.split('%s:' % interface)[1].split()
            rx_bytes, tx_bytes = (data[0], data[8])
            return (int(rx_bytes), int(tx_bytes))


def printstats():
    global last_valueTx
    global deltaTx
    global currTx
    global last_valueRx
    global deltaRx
    global currRx

    rx_bytes, tx_bytes = get_network_bytes('eth0')
    currTx = (tx_bytes)
    currRx = (rx_bytes)
    deltaTx = currTx - last_valueTx
    deltaRx = currRx - last_valueRx
    last_valueTx = currTx
    last_valueRx = currRx
    if deltaTx < 111:
     deltaTx = 0
    if deltaRx < 67:
     deltaRx = 0

def colorWipe(strip, color, wait_ms=200):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/2000.0)

def clear_strip(strip):
    for i in range(LED_COUNT):
        strip.setPixelColor(i, 0)
    strip.show()


def check_ffmpeg():
    result = subprocess.run(['sh', '/root/Livefeed-Encoder-Core/frontpanel/check_ff.sh'], stdout=subprocess.PIPE)
    st = result.stdout.decode('utf-8')
    return ("not_running" in st)

def disconnect_outputs():
    subprocess.run(['sh', '/root/Livefeed-Encoder-Core/frontpanel/disconnect_outputs.sh'])

def hsv_to_rgb(hue, sat, value):
    h = hue / 360
    s = sat / 100
    v = value / 100
    if s == 0.0:
        v *= 255
        return (v, v, v)
    i = int(h*6.)
    f = (h*6.)-i
    p, q, t = int(255*(v*(1.-s))), int(255*(v*(1.-s*f))
                                       ), int(255*(v*(1.-s*(1.-f))))
    v *= 255
    i %= 6
    if i == 0:
        return (v, t, p)
    if i == 1:
        return (q, v, p)
    if i == 2:
        return (p, v, t)
    if i == 3:
        return (p, q, v)
    if i == 4:
        return (t, p, v)
    if i == 5:
        return (v, p, q)


def set_power_led(strip):
    [hue, sat, val] = LED_COLOR["program_running"][0]
    (h, s, b) = hsv_to_rgb(hue, sat, val)
    strip.setPixelColor(LED_MAP["program_running"],
                        Color(int(h), int(s), int(b)))

def set_lan_statusTx(strip):
    [hue, sat, val] = LED_COLOR["EthO"][0]
    br = (deltaTx / UPPER_THRESHOLD)
    #print(br)
    br = br if br < 100 else 100
    #if deltaTx > 100:
    # hue = 0
    # sat = 0
    # br = 100
    #print(br)
    (h, s, b) = hsv_to_rgb(hue, sat, br)
    strip.setPixelColor(LED_MAP["EthO"],
    Color(int(h), int(s), int(b)))

def set_lan_statusRx(strip):
    [hue, sat, val] = LED_COLOR["EthI"][0]
    br = (deltaRx / UPPER_THRESHOLD)
    br = br if br < 100 else 100
    if deltaRx > 0 < 100:
     hue = 234
     sat = 100
     br = 88
    (h, s, b) = hsv_to_rgb(hue, sat, br)
    strip.setPixelColor(LED_MAP["EthI"],
    Color(int(h), int(s), int(b)))

def parse_line(b):
    try:
        return float(b)
    except ValueError:
        return 0


def map_level_to_pwm(level):
    try:
        signal = round((1 - (level / MIN_DB)) * 100)
        return min([max([signal, 0]), 100])
    except:
        return 0

def ffmpeg_thread(strip):

    global program_running
    #print("FFMPEG THREAD")
    #print((strip))
    [hue, sat, bri] = LED_COLOR["ffmpeg"][0]
    while program_running:
        not_running = check_ffmpeg()
        if not_running:
            strip.setPixelColor(LED_MAP["ffmpeg"], Color(255, 0, 0))
            #time.sleep(1)
        else:
            for i in range(0, 100, 4):
                (rL, gL, bL) = hsv_to_rgb(hue, sat, i)
                strip.setPixelColor(LED_MAP["ffmpeg"], Color(int(rL), int(gL), int(bL)))
                time.sleep(0.04)
            for i in range(100, 0, -4):
                (rL, gL, bL) = hsv_to_rgb(hue, sat, i)
                strip.setPixelColor(LED_MAP["ffmpeg"], Color(int(rL), int(gL), int(bL)))
                time.sleep(0.04)

def process_feed_audio(path, map_name, strip):
    time.sleep(1)
    t_list = [JACK_METER_PATH, path] + JACK_METER_PARAMS
    print(t_list)
    output = subprocess.Popen(t_list, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    lastsignal = 0
    for line in output.stdout:
        #print(line)
        level = parse_line(line)
        #print(level)
        signal = map_level_to_pwm(level)
        #signaldelta = signal - lastsignal
        #lastsignal = signal
        #if signaldelta < 10:
        # signal = 0
        #time.sleep(0.04)
        #print(signal)
        [hue, sat, bri] = LED_COLOR[map_name][0]
        #if 50.00 < signal < 51.00:
        # print("Signal is between 60 and 61")
        if signal < 33.2: ## Show green up to -20db
         hue = 150
         sat = 100
        #if signal > 33.3: ## Show orange between -20 and -10db
        # hue = 25
        # sat = 100
        #if signal > 66.6: ## Show Red above -10db
        # hue = 0
        # sat = 100

        (rL, gL, bL) = hsv_to_rgb(hue, sat, signal)
        #print(gL)
        strip.setPixelColor(LED_MAP[map_name], Color(int(rL), int(gL), int(bL)))
        time.sleep(0.01)


#### Main Program ####

if __name__ == '__main__':
    strip = Adafruit_NeoPixel(
        LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    #strip.begin() #STRIP removing from code
    print('Press Ctrl-C to quit.')

    clear_strip(strip)
    #set_power_led(strip)
    colorWipe(strip, Color(0, 255, 128))
    x = threading.Thread(target=ffmpeg_thread, args=(strip,))

    x_ip1 = threading.Thread(target=process_feed_audio, args=(DEVICE_INPUT_LEFT,"ch1_ip", strip, ))
    x_ip2 = threading.Thread(target=process_feed_audio, args=(DEVICE_INPUT_RIGHT,"ch2_ip", strip, ))
    x_op1 = threading.Thread(target=process_feed_audio, args=(DEVICE_OUTPUT_LEFT,"ch1_op", strip, ))
    x_op2 = threading.Thread(target=process_feed_audio, args=(DEVICE_OUTPUT_RIGHT,"ch2_op", strip, ))

    x_ip1.start()
    time.sleep(0.2)
    x_ip2.start()
    time.sleep(0.2)
    x_op1.start()
    time.sleep(0.2)
    x_op2.start()
    time.sleep(0.2)

    x.start()

    try:
        #colorWipe(strip, Color(255, 255, 255))
        time.sleep(1)
        disconnect_outputs()

        while True:
            set_power_led(strip)
            set_lan_statusTx(strip)
            set_lan_statusRx(strip)
            printstats()
            strip.show()
            time.sleep(0.02)

    except KeyboardInterrupt:
        program_running = False
        x_ip1.join()
        x_ip2.join()
        x_op1.join()
        x_op2.join()
        x.join()
        print("Break Command Received! Clearing LEDs")
        clear_strip(strip)
