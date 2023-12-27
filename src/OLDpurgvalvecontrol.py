import time
#import _thread
from pimoroni_i2c import PimoroniI2C
from breakout_potentiometer import BreakoutPotentiometer

import sh1107
import machine


###
### FOR CHEMISTS!
###
#

#IN AMPS, NOT MILLIAMPS
SET_CURRENT_VARIABLE_GLOBAL = 70

#IT IS IN SECONDS, NOT MILLISECONDS
SET_PURGE_STARTUP_GLOBAL_VALVE_OPEN_TIME = 2
SET_PURGE_CONTINUOUS_GLOBAL_VALVE_OPEN_TIME = 0.5

#SET BOOLEAN TO EITHER 'True' or 'False' TO SETUP THE INITIAL VALVE OPENING TEST
SET_PRESSURE_CHECK_UP_INITIAL_OPEN_TEST_BOOLEAN = False
#SECONDS TO CHECK WHEN YOU CAN SET UP FOR EXTRA TIME TO SET THE PRESSURE FOR YOU ALL
SET_OPEN_VALVE_PERIOD_PRESSURE_CHECK_TIME = 60

###
###
###
###

#display
spi1 = machine.SPI(0, baudrate=10_000_000, sck=machine.Pin(6), mosi=machine.Pin(7), miso=machine.Pin(4))
display = sh1107.SH1107_SPI(128, 128, spi1, machine.Pin(8), machine.Pin(15), machine.Pin(5))
display.sleep(False)
display.fill(0)
display.show()

#relay
relay = machine.Pin(2,machine.Pin.OUT)
delayVal = 1

#pot
PINS_BREAKOUT_GARDEN = {"sda": 16, "scl": 17}
i2c = PimoroniI2C(**PINS_BREAKOUT_GARDEN)
pot = BreakoutPotentiometer(i2c)
pot.set_brightness(1.0)
#pot.set_direction(BreakoutPotentiometer.DIRECTION_CCW)     # Uncomment this to flip the direction

#Multithreading
#sLock = _thread.allocate_lock()

#else
timeIncrement = int(0)
row = []


# From CPython Lib/colorsys.py
def hsv_to_rgb(h, s, v):
    if s == 0.0:
        return v, v, v
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i = i % 6
    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    if i == 5:
        return v, p, q


val = 0

timeIncrement = 0

def currentPurgeCalculation(desiredCurrent):
    if (desiredCurrent <= 75):
        timeDelay = 2300/desiredCurrent
        return timeDelay
    else:
        print("Current is too high")

def potentiometerRead(baseTime):
    val = pot.read()
    h = val * 240.0     # Colour range from red to blue

    r, g, b = [int(255 * c) for c in hsv_to_rgb(h / 360.0, 1.0, 1.0)]  # rainbow magic

    print("Percent: ", int(val * 100), "%", sep="")
    print("Pot val: ", val)
    pot.set_led(r, g, b)
    delayVal = 1.2*val*baseTime
    print("Delay val: ", delayVal)
    return delayVal

def relayControl(delay):
    relay.value(1)
    time.sleep(delay)
    relay.value(0)
    time.sleep(delay)

def displayControl(textString0, textString1, textString2):
    display.contrast(200)
    textHead='-Time period-'
    textFoot='-Milliseconds-'
    textBottom='Valve:'
    textString1=int(textString1 * 100)
    textInsert2=(textBottom+str(textString2))
    textInsert1=(str(textString1*1.2)+'%')
    textInsert0=(str(textString0*1000))
    display.fill_rect(0,48,128,96,0)
    display.text(textHead, 0, 48, 1)
    display.text(textInsert0, 0, 64, 1)
    display.text(textFoot, 0, 80, 1)
    display.text(textInsert1, 0, 96, 1)
    display.text(textInsert2, 0, 112, 1)
    display.show()

def fileWrite(value0, value1):
    #print(value0),
    text = value0, value1
    row.append(text)
    #print(text)
    with open("data.csv", "w") as f:
        for val in row:
            #print(' %f, %f\n' % val) # write the data to the file with commas between the data and a 'return' at the end
            f.write(' %f, %f\n' % val) # write the data to the file with commas between the data and a 'return' at the end
    #f.close() # close the file

def fileAppend(value0, value1):
    #print(value0),
    text = value0, value1
    #print(text)
    with open("data.csv", "a") as f:
        #print(' %f, %f\n' % val) # write the data to the file with commas between the data and a 'return' at the end
        f.write(' %f, %f\n' % text) # write the data to the file with commas between the data and a 'return' at the end
    #f.close() # close the file

def uartCall():
    #UART
    if uart.any():
        uart.write(b'\01\03\01\26\00\04\3e\a4')
        b = uart.read(8)
        print(type(b))
        print(b)
        print(int.from_bytes(b,'big'))

#while timeIncrement<5:
def openLoop():
    while True:
        relay.value(0)
        display.contrast(200)
        textHead='OPE'
        display.fill_rect(0,48,128,96,0)
        display.large_text(textHead, 0, 48, 5)
        display.show()


#def displayTask():
#    while True:
#        #print("Display Thread")
#        if (relay.value() == 0):
#            displayControl(delayVal, pot.read(),"Open")
#        if (relay.value() == 1):
#            displayControl(delayVal, pot.read(),"Closed")
#        else:
#            displayControl(delayVal, pot.read(),"Closed")



def purgevalveTask():
    print("Purge Valve Thread")
    timeIncrement = 0
    while True:
        if SET_PRESSURE_CHECK_UP_INITIAL_OPEN_TEST_BOOLEAN == True:
            sLock.acquire()
            print("Start Pressure Setup")
            print(str(SET_OPEN_VALVE_PERIOD_PRESSURE_CHECK_TIME) + " seconds open")
            relay.value(0)
            display.contrast(200)
            textHead='on'
            display.fill_rect(0,48,128,96,0)
            display.large_text(textHead, 0, 48, 5)
            display.show()
            time.sleep(SET_OPEN_VALVE_PERIOD_PRESSURE_CHECK_TIME)
            sLock.release()

        print("Start Purge Startup Valve")
        relay.value(0)
        print(relay.value())
        delayVal = SET_PURGE_STARTUP_GLOBAL_VALVE_OPEN_TIME
        print("Open")
        print(str(delayVal) + " seconds open")


        time.sleep(SET_PURGE_STARTUP_GLOBAL_VALVE_OPEN_TIME)
        timeIncrement = timeIncrement+SET_PURGE_STARTUP_GLOBAL_VALVE_OPEN_TIME
        fileAppend(timeIncrement,SET_PURGE_STARTUP_GLOBAL_VALVE_OPEN_TIME)

        purgeTime = currentPurgeCalculation(SET_CURRENT_VARIABLE_GLOBAL)

        print("Start Purge Valve Loop")
        #_thread.start_new_thread(displayTask, ())

        while True:

            #sLock.acquire()
            relay.value(1)
            print("Closed")
            delayVal = potentiometerRead(purgeTime)
            displayControl(delayVal, pot.read(),"Closed")
            #sLock.release()

            time.sleep(delayVal)

            timeIncrement = timeIncrement+purgeTime
            fileAppend(timeIncrement,purgeTime)



            #sLock.acquire()
            relay.value(0)
            print("Open")
            delayVal = SET_PURGE_CONTINUOUS_GLOBAL_VALVE_OPEN_TIME
            displayControl(delayVal, pot.read(),"Open")
            #sLock.release()

            time.sleep(SET_PURGE_CONTINUOUS_GLOBAL_VALVE_OPEN_TIME)

            timeIncrement = timeIncrement+SET_PURGE_CONTINUOUS_GLOBAL_VALVE_OPEN_TIME
            fileAppend(timeIncrement,SET_PURGE_CONTINUOUS_GLOBAL_VALVE_OPEN_TIME)

            #sLock.release()

        ####

##INIT
purgevalveTask()





