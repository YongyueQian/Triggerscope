
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 30 11:56:32 2021

@author: Yongyue
"""
import serial
import time
import sys
import glob
import numpy as np


#Searches for serial ports and provides a list to the user to choose from
if sys.platform.startswith('win'):
    ports = ['COM%s' % (i + 1) for i in range(256)]
elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
    # this excludes your current terminal "/dev/tty"
    ports = glob.glob('/dev/tty[A-Za-z]*')
elif sys.platform.startswith('darwin'):
    ports = glob.glob('/dev/tty.*')
else:
    raise EnvironmentError('Unsupported platform')

result = []
for port in ports:
    try:
        s = serial.Serial(port)
        s.close()
        result.append(port)

    except (OSError, serial.SerialException):
        pass

print("\n")
print("Listing Available Serial Ports....\n")
x = 1
for n in result:
    print("Enter " + str(x) + " for " + n)
    x=x+1
print()
pNum = input("Select Com port # from list above to open.....:")

#open the port...
tgCom = result[int(pNum) - 1]
tgS = serial.Serial()
tgS.port = tgCom
tgS.baudrate = 115200
tgS.bytesize = serial.EIGHTBITS  # number of bits per bytes
tgS.parity = serial.PARITY_NONE  # set parity check: no parity
tgS.stopbits = serial.STOPBITS_ONE  # number of stop bits
# tgS.timeout = None          #block read
tgS.timeout = 0.5  # non-block read
tgS.xonxoff = False  # disable software flow control
tgS.rtscts = False  # disable hardware (RTS/CTS) flow control
tgS.dsrdtr = False  # disable hardware (DSR/DTR) flow control
tgS.writeTimeout = 0  # timeout for write

try:
    print("Activating Triggerscope...")
    tgS.open()
except Exception as e:
    print("ERROR: Triggerscope Com port NOT OPEN: " + str(e))
    exit()
if tgS.isOpen():
    try:
        tgS.flushInput()  # flush input buffer, discarding all its contents
        tgS.flushOutput()  # flush output buffer, aborting current output
        op = "*"
        tgS.write(op.encode() + "\n".encode('ascii'))  # send an ack to tgs to make sure it's up
        time.sleep(0.2)  # give the serial port sometime to receive the data
        print("Rx: " + tgS.readline().decode())
    except Exception as e1:
        print(" serial communication error...: " + str(e1))

else:
    print("cannot open tg cell  port ")


#functions below each perform a sample experiment as documented in the command syntax documentation

#this command performs a write
def writetgs(tgin):
    '''send a serial command to the triggerscope...
    Args:
        tgin: input string to send. Note the command terminator should be included in the string.
    Returns:
        char string of whatever comes back on the serial line.
    Raises:
        none.
    '''
    tgS.flushInput()  # flush input buffer, discarding all its contents
    tgS.flushOutput()  # flush output buffer, aborting current output
    tgS.write(tgin.encode())  # send command
    time.sleep(0.01)  # give the serial port sometime to receive the data 50ms works well...
    bufa = ""
    # bufa = tgS.readline()
    return bufa

writetgs("RESET\n")
#this uses the "STAT" command to report the current status of the triggerscope
def readStat():
    tgS.flushInput()  # flush input buffer, discarding all its contents
    tgS.flushOutput()  # flush output buffer, aborting current output
    tgS.write("STAT?\n".encode())  # send command
    time.sleep(0.02)  # give the serial port sometime to receive the data 50ms works well...
    for n in range(100):
        time.sleep(0.2)  # give the serial port sometime to receive the data 50ms works well...
        bufa = ""
        bufa = tgS.readline()
        print(bufa);
        if(len(bufa) < 5):
            break 
def send_signal_random(steps,freq,cycle):
    writetgs("PROG_WAVE," + str(1)+ "," + str(1)+ ","+ str(steps)+"," +str(0)+ "," +str(freq)+ "," +str(cycle)+ "\n")
   # tgS.write("PROG_WAVE," + str(dac)+ ","+ str(steps)+"," +str(trig)+ "\n")  # send command
    time.sleep(0.1) #quick delay needed?
    dacarray = np.random.randint(65535,size=steps)
    ttlarray = np.random.randint(0,2,steps)
    for x in range(steps):
        time.sleep(0.01)
        tosend = str(dacarray[x])+ "," + str(ttlarray[x]) + "\n"
        tgS.write(tosend.encode())
    print(ttlarray);
    print(dacarray);
    # for n in range(steps+1):
    #     time.sleep(0.01)  # give the serial port sometime to receive the data 50ms works well...
    #     bufa = ""
    #     bufa = tgS.readline().decode()
    #     print(bufa);
    tosend = "STARTWAVE\n"
    tgS.write(tosend.encode())
    #("STARTWAVE\n")
    # input("Enter to close...")
    # tgS.write("\n".encode())
    
def send_signal_manually(dac,ttl,startV,endV,steps,freq,cycle):
    startV,endV = int((startV+5)/10*65535),int((endV+5)/10*65535)
    time.sleep(0.1) 
    dacarray=np.ceil(np.linspace(startV,endV,steps))
    ttlarray=np.ones(steps, dtype=int)
    writetgs("PROG_WAVE," + str(dac)+ "," + str(ttl)+ ","+ str(steps)+"," +str(0)+ "," +str(freq)+ "," +str(cycle)+ "\n")
    for x in range(steps):
        time.sleep(0.01)
        tosend = str(int(dacarray[x]))+ "," + str(ttlarray[x]) + "\n"
        tgS.write(tosend.encode())
    #print(ttlarray);
    print(dacarray);
    # for n in range(steps+1):
    #     time.sleep(0.01)  # give the serial port sometime to receive the data 50ms works well...
    #     bufa = ""
    #     bufa = tgS.readline().decode()
    #     print(bufa);
    tosend = "STARTWAVE\n"
    tgS.write(tosend.encode())
    #("STARTWAVE\n")
    #input("Enter to close...")
    #tgS.write("\n".encode())
    #tosend = "STARTWAVE\n"
# def set_dac(num,volt,step):
#     pass
#     input("Enter to close...")
#     tgS.write("\n".encode())
    
import sys
from PyQt5 import QtWidgets, uic

qtcreator_file  = "signal.ui" # Enter file here.
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtcreator_file)

class TriggerSignal(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.start_button.clicked.connect(self.runscan_random)
        self.start_button1.clicked.connect(self.runscan_manually) 
    def runscan_random(self):
        st = int(self.step.toPlainText())
        fr = int(self.freq.toPlainText())
        cy = int(self.cycle.value())
        send_signal_random(st,fr,cy)
    def runscan_manually(self):
        dn = 1
        tn = 1
        sv = int(self.startV.toPlainText())
        en = int(self.endV.toPlainText())
        st = int(self.step1.toPlainText())
        fr = int(self.freq1.toPlainText())
        cy = int(self.cycle1.value())
        send_signal_manually(dn,tn,sv,en,st,fr,cy)
    # def runscan_setdac(self):
    #     dn = int(self.DAC2.value())#dac number
    #     fv = float(self.final1.value())#
    #     st = float(self.step2.toPlainText())
    #     self.current_V = float(self.cv.toPlainText())
    #     self.cv.setText(str(round(fv,3)))
    #     step_size = int(np.ceil(np.abs(fv - self.current_V)/st))
    #     trig,freq,cycle,ttl=0,0,1,1

    #     dacarray=np.linspace(self.current_V, fv, step_size)
    #     print(dacarray)
    #     vs=len(dacarray)
    #     print(vs)
    #     ttlarray=np.ones(vs, dtype=int)
    #     print(writetgs("PROG_WAVE," + str(dn)+ "," + str(ttl)+ ","+ str(vs)+"," +str(trig)+ "," +str(freq)+ "," +str(cycle)+ "\n"))
    #     tosend = ((dacarray+5)/10)*65535
    #     tosend = tosend.astype(int)
    #     if min(tosend) < 0 or max(tosend) > 65535:
    #         print('ERROR: Voltage is outside range')
    #     for x in range(vs):
    #         time.sleep(0.001)
    #         str_tosend = str(tosend[x]) + "," + str(ttlarray[x]) + "\n"
    #         tgS.write(str_tosend.encode())      
    #     tosend = "STARTWAVE\n"
    #     tgS.write(tosend.encode())

    # def closeport(self):
    #     self.destroy()
    #     tgS.close()
    #     sys.exit()
    def closeEvent(self, event):
        super(TriggerSignal, self).closeEvent(event)
        tgS.close()
        #self.serial_port.close()
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = TriggerSignal()
    window.show()
    sys.exit(app.exec_())
    
