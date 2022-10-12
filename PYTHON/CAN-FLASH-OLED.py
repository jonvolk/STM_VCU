# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 09:49:19 2019

@author: frainville

library requirements
pip install python-can
pip install intelhex
pip install crcmod

"""
#!/usr/bin/env python

import can

import os
import copy

from intelhex import IntelHex
import crcmod

import time
import math

from threading import Timer, Thread, Event
from datetime import datetime
import psutil

#SCREEN SETUP-----------------------------------------------------------------
from pathlib import Path
#from demo_opts import get_device
from luma.core.virtual import terminal
from PIL import ImageFont

from luma.core.interface.serial import i2c, spi, pcf8574
from luma.core.interface.parallel import bitbang_6800
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106, ws0010
from luma.core.virtual import terminal



serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, rotate=0)
font = ImageFont.load_default()
term = terminal(device, font, animate=False)

# SETUP SECTION ---------------------------------------------------------------
# Board name indicate which board we want to programm
# 'APP' for demo application

#BOARD_NAME = 'APP'


# HEX file to load
#HEX_FILE = '/home/jon/CAN_bootloader/F105_APP/build/F105_APP.hex'
BUILD_NAME = os.path.realpath(__file__).split(os.sep)[-2]
BOARD_NAME = '{}'.format(BUILD_NAME)
#CRC_FILE = '{}_CRC.hex'.format(BOARD_NAME)
CRC_FILE = './CAN_FLASH/{}/{}_CRC.hex'.format(BUILD_NAME,BOARD_NAME)
#HEX_FILE = '{}.hex'.format(BOARD_NAME)
HEX_FILE = './CAN_FLASH/{}/{}.hex'.format(BUILD_NAME,BOARD_NAME)


# CANID needs to be used with a OR with CAN_SA
RX_HEARTBEAT_CANID = 0x00FF00 
RX_FEEDBACK_CANID = 0x00FE00
TX_CMD_CANID = 0x00FF00




#--------------------------------------------------------------- SETUP SECTION 

# SCREEN FUNCTIONS------------------------------------------------------------



# FUNCTIONS -------------------------------------------------------------------

def CleanExit():
    global connTimer, SendTimeout, microSendTimeout
    global notifier
    global bus
    
    connTimer.cancel()
    SendTimeout.cancel()
    microSendTimeout.cancel()
 
    notifier.stop()
    bus.shutdown()
    os.system(CAN_KILL)
    print("ended")
    term.println('ended')
    time.sleep(1.00)
    os.system('python3 menu.py')
    exit()
    os._exit(os.EX_OK)

# Load the Hex file, convert it to Bin
def LoadHex(file):
    
    try:
        # Load the hex file
        ih=IntelHex()
        ih.fromfile(file,format='hex')

        bf = ih.tobinarray() #convert to bin array

    except ValueError:
        print(f'Error {ValueError} in LoadHex function with argument : {file}')
        term.println(f'Error {ValueError} in LoadHex : {file}')       
        CleanExit()
    
    return bf

def GetCrc():
    try:
        ih=IntelHex()
        ih.fromfile(CRC_FILE,format='hex')
        bf = ih.tobinarray(start=CRC_ADDRESS, size=4) #convert to bin array
        #bf = ih.tobinarray(start=0x801EFFC, size=4) #convert to bin array
        #print(bf)
        checksum_val = int.from_bytes(bf, byteorder='little', signed=False)
        #checksum_val =f'0x{checksum_val:X}'
        checksum_val =f'0x{checksum_val:0{8}X}'
        #print(checksum_val)
    except ValueError:
        print(f'Error {ValueError} in GetCrc function')
        term.println(f'Error {ValueError} in GetCrc function')
        CleanExit()
    
    return checksum_val

# CAN FUNCTIONS ---------------------------------------------------------------
def can_send(id,senddata):
    global bus
    msg = can.Message(arbitration_id=id,
                      data=senddata)#,
                      
                      #extended_id=True)
    try:
        bus.send(msg)
    except can.CanError:
        print(f'Message {"0x%X" % id} NOT sent')
        term.println(f'Mess. {"0x%X" % id} NOT sent')
   

def can_send_cmd(cmd):
    global TX_CAN_SA
    
    can_send(TX_CMD_CANID | TX_CAN_SA,[cmd])

def can_send_word(index):
    global BinaryFile,TX_CAN_SA
    global SendTimeout
    
    if (index + 3)<len(BinaryFile):
        D4=BinaryFile[index+3]
    else:
        D4=0xFF
    
    i2=math.floor(index)/4

    D5 = math.floor(i2) & 0xFF
    D6 = (math.floor(i2) & 0xFF00)>>8
    
    can_send(TX_CMD_CANID | TX_CAN_SA,[0xF6,BinaryFile[index],BinaryFile[index+1],BinaryFile[index+2],D4,D5,D6])
    
    SendTimeout.start()
    
    if((index%1024)==0) or (index>=len(BinaryFile)):
        print(f'{math.floor(index/1024)}/{math.floor(len(BinaryFile)/1024)} kB')
        term.println(f'{math.floor(index/1024)}/{math.floor(len(BinaryFile)/1024)} kB')
    #Start timeout

class CanListener(can.Listener):

    
    def on_message_received(self, msg):
        global RX_CAN_SA, TX_CAN_SA
        global connTimer, SendTimeout, microSendTimeout
        global SPAM_BIT
        global BOOT_CRC_REQUESTED_BIT
        global WAIT_FLASH_ERASE_BIT
        global WRITING_FLASH_BIT
        global WAITING_CRC_BIT
        global currentTimeout,flawcount, SPAM_BIT
        global BinaryFile
        global currentIndex
        global CRC_VAL
        global BOOT_CRC_VAL
    
        global msgqty


        # manage low level messages here
        
        # Receive CRC value
        if ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and len(msg.data)==5 and msg.data[0]==0xF4):
            CRC_VAL = f'0x{msg.data[1]:02X}{msg.data[2]:02X}{msg.data[3]:02X}{msg.data[4]:02X}'
            WAITING_CRC_BIT=0
        
        #Receive Heartbeat
        elif ((msg.arbitration_id == (RX_HEARTBEAT_CANID | RX_CAN_SA)) and msg.data[0]==0xFF):
            SPAM_BIT = 0
            connTimer.cancel() #reset connTimer
            #connTimer._interval=currentTimeout
            connTimer.start()
        
        #Flash erase success
        elif ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and msg.data[0]==0xF0):
            print('Flash Erase Success')
            WAIT_FLASH_ERASE_BIT=0
            currentTimeout=0.500
            connTimer._interval=currentTimeout
            connTimer.cancel() #reset connTimer
            connTimer.start()
            
        #Flash erase failed
        elif ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and msg.data[0]==0xF1):
            print('Flash Erase Failed')
            term.println('Flash Erase Failed')
        
        #write started
        elif ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and msg.data[0]==0xF5):
            print('write started')
            term.println('write started')
            
            currentIndex = 0
            
            can_send_word(currentIndex)
            
            currentIndex = currentIndex + 4
        
        #Next word
        elif ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and msg.data[0]==0xF6 and msg.data[1]==0xFF):
            SendTimeout.cancel()
            
            index = (msg.data[2] + (msg.data[3]<<8))*4

            if (index == currentIndex):
                if not(currentIndex >= len(BinaryFile)):
                     microSendTimeout.cancel()
                     flawcount = 0
                     can_send_word(currentIndex)

                     currentIndex += 4

                     microSendTimeout.start()
                else:
                     can_send_cmd(0xF7)
        
        elif ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and msg.data[0]==0xF6 and msg.data[1]==0x00):
            SendTimeout.cancel()
            print('Flash write failed, have you erased flash first?')
            term.println('Flash write failed')
        
        #Flash done
        elif ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and msg.data[0]==0xF7 and msg.data[1]==0xFF):
            SendTimeout.cancel()
            microSendTimeout.cancel()
            WRITING_FLASH_BIT=0
            print("Flash succeeded please check checksum")
            term.println('Flash succeeded')
        
        #read Bootloader CRC
        if ((msg.arbitration_id == (RX_FEEDBACK_CANID | RX_CAN_SA)) and len(msg.data)==5 and msg.data[0]==0x0F):
            BOOT_CRC_VAL = f'0x{msg.data[1]:02X}{msg.data[2]:02X}{msg.data[3]:02X}{msg.data[4]:02X}'
            BOOT_CRC_REQUESTED_BIT=0

#--------------------------------------------------------------- CAN FUNCTIONS 

# Continuous Timer 
class _Timer(Timer):
  def run(self):
    while not self.finished.is_set():
      self.finished.wait(self.interval)
      self.function(*self.args, **self.kwargs)
    self.finished.set()

# Repeatable Timer
class RepeatTimer(object):
  def __init__(self, interval, function, args=[], kwargs={}):
    self._interval = interval
    self._function = function
    self._args = args
    self._kwargs = kwargs
    self.t = Timer(self._interval, self._function, *self._args, **self._kwargs)
  def start(self):
    self.t = Timer(self._interval, self._function, *self._args, **self._kwargs)
    self.t.start()
  def cancel(self):
    self.t.cancel()
  def is_alive(self):
    return self.t.is_alive()


# Timer Tasks -----------------------------------------------------------------

def connTimer_Tick():
    global SPAM_BIT, RUN_REQUESTED, connTimer
    
    connTimer.cancel()
    
    SPAM_BIT = 0
    
    if RUN_REQUESTED==0:    
        print('Lost Connection')
        term.println('Lost Connection')
        CleanExit()
    else:
        RUN_REQUESTED=0

def SendTimeout_Tick():
    global SendTimeout,microSendTimeout
    
    SendTimeout.cancel()
    microSendTimeout.cancel()

    print('Send Timeout')
    CleanExit()
    
def microSendTimeout_Tick():
    global microSendTimeout, currentIndex, flawcount
    
    microSendTimeout.cancel()
    
    can_send_word(currentIndex-4)
    
    if flawcount<100: #was 100
        microSendTimeout.start()

    

#----------------------------------------------------------------- Timer Tasks 

#------------------------------------------------------------------- FUNCTIONS 

# BSP -------------------------------------------------------------------------
# Board support configurations
if BOARD_NAME=="VCU":
    # 'APP' for demo application
    TX_CAN_SA = 0x01
    RX_CAN_SA = 0x02
    TARGET_APP_SA = 0x10
    CRC_ADDRESS = 0x801EFFC
elif BOARD_NAME=='PDM_REAR':
    # 'OTHER_APP' for other applications
    TX_CAN_SA = 0x21
    RX_CAN_SA = 0x22
    TARGET_APP_SA = 0x20
    CRC_ADDRESS = 0x800EFFC
elif BOARD_NAME=='BMS':
    # 'OTHER_APP' for other applications
    TX_CAN_SA = 0x31
    RX_CAN_SA = 0x32
    TARGET_APP_SA = 0x30
else:
    # error BOARD_NAME not supported
    print(f'no support for {BOARD_NAME}')
    CleanExit()
#------------------------------------------------------------------------- BSP

# Main Program ----------------------------------------------------------------
def main():
    global SPAM_BIT
    global BOOT_CRC_REQUESTED_BIT
    global WAIT_FLASH_ERASE_BIT
    global WRITING_FLASH_BIT
    global WAITING_CRC_BIT
    global RUN_REQUESTED,currentIndex,flawcount,currentTimeout,connTimer,SendTimeout,microSendTimeout, bus,BinaryFile,RxDb
    global CRC_VAL
    global BOOT_CRC_VAL

    global notifier
    # Hex file load and verifications ---------------------------------------------

    this_proc = os.getpid() 

    for proc in psutil.process_iter():
        procd = proc.as_dict(attrs=['pid', 'name'])
        if "python" in str(procd['name']) and procd['pid'] != this_proc:
            proc.kill()


    BinaryFile=LoadHex(HEX_FILE)    # Load Hex file and convert to binary array
    
    
    # Retrieve Boardname from the Hex file (see *FLASH.ld in STM32 application)
    try:
        HexBoardName="".join(map(chr,BinaryFile[0x1E4:(0x1E4+10)])) #  0A818800    0A81E400
        HexBoardName=HexBoardName[0:HexBoardName.find("\x00")]
        print(f'Checking HEX compatibility with Target')
        term.println("Validating HEX file")
        

    except ValueError:
        print(f'Error finding BOARD NAME in {HEX_FILE}')
        term.println(f'Error finding BOARD NAME in {HEX_FILE}')
        exit()
    
    if HexBoardName!=BOARD_NAME:
        print(f'{BOARD_NAME} doesn\'t match {HexBoardName} from {HEX_FILE} Maybe this HEX file don\'t belong in this board?')
        term.println(f'{BOARD_NAME} doesn\'t match {HexBoardName} from {HEX_FILE}')
        CleanExit()
    
    HEX_CRC=GetCrc()   # CRC to compare after download
    
    #--------------------------------------------- Hex file load and verifications 
    
    # Programming sequence --------------------------------------------------------
    
    ## On timeouts, print a warning message
    
    
    # begin to SPAM enter bootloader command
    SPAM_BIT=1
    currentTimeout = 5
    connTimer._interval = currentTimeout
    connTimer.cancel()
    connTimer.start()
    
    # Send reboot command (need to be implemented)
    
    # Wait to detect bootloader or timeout then stop SPAM
    while(SPAM_BIT==1):
        can_send_cmd(0xF0)
        time.sleep(0.001)

    #Request BOOT CRC
    BOOT_CRC_REQUESTED_BIT = 1
    currentTimeout = 5
    connTimer._interval = currentTimeout
    connTimer.cancel()
    connTimer.start()
    can_send_cmd(0x0F)

    while(BOOT_CRC_REQUESTED_BIT==1):
        time.sleep(0.001)

    print(f'BOOT CRC32 : {BOOT_CRC_VAL}')
    print(f'HEX CRC32 : {HEX_CRC}')
    term.println(f'BOOT CRC32 : {BOOT_CRC_VAL}')
    term.println(f'HEX CRC32 : {HEX_CRC}')
    
    # Erase flash
    currentTimeout = 5
    connTimer._interval = currentTimeout
    connTimer.cancel()
    connTimer.start()
    WAIT_FLASH_ERASE_BIT=1;
    can_send_cmd(0xF1)
   
    
    # Wait for confirmation flash erased or timeout
    while(WAIT_FLASH_ERASE_BIT==1):
        time.sleep(0.001)
    
    # Start Flash write
    currentTimeout = .5 #was .5
    connTimer._interval = currentTimeout
    connTimer.cancel()
    connTimer.start()
    WRITING_FLASH_BIT=1
    can_send_cmd(0xF5)
    # Wait for Flash Write finish or timeout
    while(WRITING_FLASH_BIT==1):
        time.sleep(0.001)

    # request CRC from STM32
    WAITING_CRC_BIT=1
    can_send_cmd(0xF4)
    # wait for CRC or timeout
    while(WAITING_CRC_BIT==1):
        time.sleep(0.001)
        
    # compare CRC from stm32 and from HEX file
    # if CRC match, SUCCESS and Run Application, if CRC don't match, WARNING
    print(f'HEX CRC32 is : {HEX_CRC}\nMCU CRC32 is : {CRC_VAL}')
    if (HEX_CRC==CRC_VAL):
        print(f'SUCCESS! CRC MATCH')
        print(f'SENDING RUN COMMAND')
        term.println(f'SUCCESS! CRC MATCH')
        term.println(f'SENDING RUN COMMAND')
        can_send_cmd(0xF2)
        os.system(CAN_KILL)
        time.sleep(3.0)
        os.system('python3 menu.py')
        exit()
 
    else:
        print(f'ERROR! CRC DOESNT\'T MATCH')
        term.println(f'ERROR! CRC DOESNT MATCH')
        os.system(CAN_KILL)
        time.sleep(3.0)
        os.system('python3 menu.py')
        exit()

    #CleanExit()

# Global Vars -----------------------------------------------------------------
SPAM_BIT = 0
BOOT_CRC_REQUESTED_BIT = 0
WAIT_FLASH_ERASE_BIT = 0
WRITING_FLASH_BIT = 0
WAITING_CRC_BIT = 0
RUN_REQUESTED = 0
CRC_VAL = 0x00000000
CAN_CONFIG = 'sudo slcand -o -f -s6 -S500000 /dev/ttyUSB0 can0'
CAN_TX_QL = 'sudo ifconfig can0 txqueuelen 20000'
CAN_START = 'sudo ifconfig can0 up'
CAN_KILL = 'sudo pkill slcand'

currentIndex = 0
flawcount = 0

currentTimeout= .5 #was.5

connTimer = RepeatTimer(0.200,connTimer_Tick)
SendTimeout = RepeatTimer(250.500,SendTimeout_Tick)
microSendTimeout = RepeatTimer(.10,microSendTimeout_Tick)


#----------------------------------------------------------------- Global Vars 
# configure and Connect to CAN

os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')
bus = can.interface.Bus(channel = 'can0', bustype = 'socketcan')# socketcan_native


#bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
#bus = can.interface.Bus(bustype='slcan',channel='/dev/ttyUSB0',ttyBaudrate = 500000,bitrate = 500000,sleep_after_open=5)
# bus = can.interface.Bus(bustype='ixxat', channel=0, bitrate=500000)
# bus = can.interface.Bus(bustype='vector', app_name='CANalyzer', channel=0, bitrate=500000)
    
# Start the CAN receiver
a_listener = CanListener()
notifier = can.Notifier(bus, [a_listener])
    
main()


#-------------------------------------------------------- Programming sequence 
