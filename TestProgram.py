from termios import TIOCPKT_DATA
import RPi.GPIO as GPIO
import time
import Adafruit_ADS1x15
import adafruit_ssd1306
import busio
from mpu6050 import mpu6050
from time import sleep
import board
import adafruit_bme280.advanced as adafruit_bme280
import serial


HC12 = serial.Serial(
    port = '/dev/ttyS0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

#ADS1115
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2
if GAIN == 2/3 : conv = 0.0001875   # +/- 6.144
if GAIN == 1 : conv = 0.000125      # +/-4.096V
if GAIN == 2 : conv = 0.0000625     # +/-2.048V
if GAIN == 4 : conv = 0.00003125    # +/-1.024V
if GAIN == 8 : conv = 0.000015625   # +/-0.512V
if GAIN == 16 : conv = 0.000007812  # +/-0.256V


#BME280
i2c = board.I2C()  # uses board.SCL and board.SDA
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

#OLED
from board import SCL, SDA
from PIL import Image, ImageDraw, ImageFont

#Display configuration
i2c = busio.I2C(SCL, SDA)
disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)
font = ImageFont.load_default()
width = disp.width
height = disp.height
image = Image.new("1", (width, height))
draw = ImageDraw.Draw(image)

#Display border settings
padding = -3                    #the space between top and 1st str
top = padding                   
bottom = height - padding       #auto adjust bottom border

#GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Pin declaration
sensor = mpu6050(0x68)

#Input
pb1 = 10
pb2 = 9
pb3 = 11
toggle = 26

#Output
red = 13
yellow = 6
green = 5
buzzer = 19

GPIO.setup(toggle,GPIO.IN)
GPIO.setup(pb1,GPIO.IN)
GPIO.setup(pb2,GPIO.IN)
GPIO.setup(pb3,GPIO.IN)

GPIO.setup(red,GPIO.OUT)
GPIO.setup(yellow,GPIO.OUT)
GPIO.setup(green,GPIO.OUT)
GPIO.setup(buzzer,GPIO.OUT)

#Variable Initialization
pb1_counter = pb2_counter = pb3_counter = 0
old_input_pb1 = old_input_pb2 = old_input_pb3 = 0
input_toggle = 0
stateRed = stateYellow = stateGreen = stateBuzzer = 0
LED_counter = 0
accel_data = gyro_data = temp = 0
state = 0
x = 1
interval = 0.3
counter = 0

#Clears the writeText
def cleardisplay():
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

#Displays up to 20 characters per row
def writeText(pos,align,string):
    global x

    #determine position of string
    if pos == 0 : position = 0
    if pos == 1 : position = 8
    if pos == 2 : position = 16
    if pos == 3 : position = 25
    
    if type(string) == int:
        if string == 0 : string = "Off"
        elif string == 1 : string = "On"

    #1 = centre, 2 = right, else left
    #Alignment
    string_post = str(string)
    numchar = len(string_post)*5
    if align == 1 : 
        dis = (128-numchar)/2 - 5

    elif align == 2 :
        dis = (128-numchar) - 10

    else: 
        dis = 0

    draw.text((dis, top + position), str(string), font=font, fill=255)

def display() :
    disp.image(image)
    disp.show()

def checkpbevent() :
    global pb1_counter, pb2_counter, pb3_counter
    global old_input_pb1, old_input_pb2, old_input_pb3
    global input_toggle
    
    input_toggle = GPIO.input(toggle)
    input_pb1 = GPIO.input(pb1)
    input_pb2 = GPIO.input(pb2)
    input_pb3 = GPIO.input(pb3)

    if input_pb1 == 1 and old_input_pb1 == 0 :
        pb1_counter += 1
        time.sleep(0.3)
        return 1

    if input_pb2 == 1 and old_input_pb2 == 0 :
        pb2_counter += 1
        time.sleep(0.3)
        return 2

    if input_pb3 == 1 and old_input_pb3 == 0 :
        pb3_counter += 1
        time.sleep(0.3)
        return 3

    old_input_pb1 = input_pb1
    old_input_pb2 = input_pb2
    old_input_pb3 = input_pb3


def statemenu(state) :
    global stateRed, stateYellow, stateGreen, stateBuzzer, LED_counter, x
    global accel_data, gyro_data, temp
    global interval
    global input_toggle
    input = checkpbevent()

    text = "{:.3f}"
    text2 = "{:.2f}"

    #LED
    if state == 1 :
        cleardisplay()

        writeText(0,1,"LED")
        writeText(1,0,"Red"); writeText(1,1,"Yellow"); writeText(1,2,"Green")
        writeText(2,0,stateRed); writeText(2,1,stateYellow); writeText(2,2,stateGreen)
        writeText(3,0,"Back"); writeText(3,1,"Select"); writeText(3,2,"Next")

        if input == 2 :
            LED_counter += 1

        #FSM
        if LED_counter == 0 : 
            stateRed = stateYellow = stateGreen = 0
        elif LED_counter == 1 :
            stateRed = 1
            stateYellow = stateGreen = 0
        elif LED_counter == 2 :
            stateRed = stateYellow = 1
            stateGreen = 0
        elif LED_counter == 3 :
            stateRed = stateYellow = stateGreen = 1
        #Overflow
        if LED_counter == 4 : LED_counter = 0

        #FSM output
        GPIO.output(red,GPIO.HIGH) if stateRed == 1 else GPIO.output(red,GPIO.LOW)
        GPIO.output(yellow,GPIO.HIGH) if stateYellow == 1 else GPIO.output(yellow,GPIO.LOW)
        GPIO.output(green,GPIO.HIGH) if stateGreen == 1 else GPIO.output(green,GPIO.LOW)

        display()
        
    #Buzzer
    if state == 2 :
        cleardisplay()

        writeText(0,1,"Buzzer")
        writeText(1,1,stateBuzzer)
        writeText(3,0,"Back"); writeText(3,1,"Select"); writeText(3,2,"Next")

        if input == 2 and stateBuzzer == 0 :
            stateBuzzer = 1
        elif input == 2 and stateBuzzer == 1 :
            stateBuzzer = 0

        display()

    # #MPU6050
    if state == 3 :
        cleardisplay()

        writeText(0,0,"Accel")
        writeText(0,1,"Temp")
        writeText(0,2,"Gyro")

        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()
        temp = sensor.get_temp()

        accel_x = accel_data['x']
        accel_y = accel_data['y']
        accel_z = accel_data['z']

        gyro_x = gyro_data['x']
        gyro_y = gyro_data['y']
        gyro_z = gyro_data['z']

        writeText(1,1,text2.format(temp))

        writeText(1,0,text.format(accel_x))
        writeText(2,0,text.format(accel_y))
        writeText(3,0,text.format(accel_z))

        writeText(1,2,text.format(gyro_x))
        writeText(2,2,text.format(gyro_y))
        writeText(3,2,text.format(gyro_z))

        display()

    # #BME280
    if state == 4 :
        cleardisplay()

        writeText(0,0,"BME280")

        bme280.sea_level_pressure = 1008.1
        bme280.mode = adafruit_bme280.MODE_NORMAL
        # bme280.standby_period = adafruit_bme280.STANDBY_TC_500
        bme280.iir_filter = adafruit_bme280.IIR_FILTER_X16
        bme280.overscan_pressure = adafruit_bme280.OVERSCAN_X16
        bme280.overscan_humidity = adafruit_bme280.OVERSCAN_X1
        bme280.overscan_temperature = adafruit_bme280.OVERSCAN_X2
        
        bmetemp = round(bme280.temperature,3)
        humidity = round(bme280.relative_humidity,3)
        pressure = round(bme280.pressure,3)
        altitude = round(bme280.altitude,3)

        writeText(0,2,text.format(bmetemp) + "  °C")
        writeText(1,2,text.format(humidity) + " %RH")
        writeText(2,2,text.format(pressure) + " hPa")
        writeText(3,2,text.format(altitude) + "   m")

        display()

    #MQ3
    if state == 5 :
        stateBuzzer = 0
        cleardisplay()

        writeText(0,1,"MQ3")

        value1 = adc.read_adc(1, gain=GAIN) + 1
        voltage1 = (value1) * conv
        actual_voltage1 = voltage1/0.4110718492

        if actual_voltage1 >= 2.5 :
            trigger = "Alarm"
            stateBuzzer = 1
        else :
            trigger = "No Gas"
            stateBuzzer = 0

        writeText(1,1,text2.format(actual_voltage1) + " V")
        writeText(2,1,trigger)
        writeText(3,0,"Back")
        writeText(3,2,"Next")

        display()

    #HC12
    #send mode or receive mode
    if state == 6 :
        cleardisplay()
        global counter

        text = ("Hello World " + str(counter) + "\n")
        entext = text.encode(encoding='UTF-8')
        HC12output = ""
        if input_toggle == 1 :
            mode = "Transmitting"
            HC12.write(entext)
            HC12output = text
            counter += 1
        elif input_toggle == 0 :
            mode = "Receiving"
            if HC12.inWaiting() > 0:
                input_read = HC12.readline()
                detext = input_read.decode(encoding='UTF-8')
                HC12output = detext

        writeText(0,1,"HC12")
        writeText(1,1,mode)
        writeText(2,1,HC12output)
        writeText(3,0,"Back")
        writeText(3,1,"Toggle SW")
        writeText(3,2,"Next")

        display()

    #Battery
    """
    battery range is from 6.4v to 8.4v (2s cell)
    at 6.4v output is 1.772v (min)
    at 8.4v output is 2.326v (max)
    r1 = 10k
    r2 = 3.83k

    test input is 1.5~1.6v
    expected output : 5.778v
    """
    if state == 7 :
        cleardisplay()
        writeText(0,1,"Battery")

        value0 = adc.read_adc(0,gain=GAIN) + 1
        voltage0 = (value0)*conv                    #convert from bit to V
        actual_voltage0 = voltage0/0.276934         #convert from reading to actual

        capacity = (actual_voltage0/8.4)*100

        writeText (1,1,text2.format(actual_voltage0) + " V")
        writeText (2,1,text2.format(capacity) + " %")
        writeText(3,0,"Back")
        writeText(3,2,"Next")

        display()

    #Move to next state
    if input == 3 :
        x += 1

    elif input == 1 :
        x -= 1

    #State overflow
    if x >= 8 : x = 1
    if x <= 0 : x = 7

    #Annoying buzzer
    if state != 5 and state != 2 : stateBuzzer = 0
    GPIO.output(buzzer,GPIO.HIGH) if stateBuzzer == 1 else GPIO.output(buzzer,GPIO.LOW)

while True :
    statemenu(x)
    time.sleep(interval)
