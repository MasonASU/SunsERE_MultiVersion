"""
The MIT License (MIT)
Copyright (c) 2014 Ankit Aggarwal <ankitaggarwal011@gmail.com>
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


#!/usr/bin/env python

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import tkinter
import tkinter as tk
import pylab
from sr830_serial import SR830
import serial
import sys
import glob
import TLabs_Motor
import clr
import time
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import nidaqmx

# -*- coding: utf-8 -*-
import Thorlabs.TLDC2200_64.Interop
# Create virtual DC2200 object
led = Thorlabs.TLDC2200_64.Interop.TLDC2200("USB::4883::32968::M00639228",False,True)
sys.path.append(r"C:\Program Files\Thorlabs\Kinesis")

# add .net reference and import so python can see .net
clr.AddReference("Thorlabs.MotionControl.Controls")
import Thorlabs.MotionControl.Controls

# print(Thorlabs.MotionControl.Controls.__doc__)

# Add references so Python can see .Net
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericMotorCLI")
clr.AddReference("Thorlabs.MotionControl.IntegratedStepperMotorsCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.DCServo")
from Thorlabs.MotionControl.DeviceManagerCLI import *
# from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.IntegratedStepperMotorsCLI import *

def extrap1d(interpolator):
    xs = interpolator.x
    ys = interpolator.y

    def pointwise(x):
        if x < xs[0]:
            return ys[0]+(x-xs[0])*(ys[1]-ys[0])/(xs[1]-xs[0])
        elif x > xs[-1]:
            return ys[-1]+(x-xs[-1])*(ys[-1]-ys[-2])/(xs[-1]-xs[-2])
        else:
            return interpolator(x)

    def ufunclike(xs):
        return np.array(list(map(pointwise, np.array(xs))))

    return ufunclike

nidaq = nidaqmx.Task()
nidaq.ai_channels.add_ai_voltage_chan("Dev1/ai0")

# Fundamental constants
Q = 1.6021e-19
hc = 1239.8

# Spectralon thickness in mm
spectralonThickness = 11.8
# Maximum downward position in mm
positionMax = 25
# Number of positions to optimize for
measurementPositionAmount = 20

# Field key names
BANDGAP_FIELD = 0
THICKNESS_FIELD = 1
MAX_LED_CURRENT_FIELD = 2
SUNS_ERE_DIRECTION = 3
TWO_PERCENT_FIELD = 4
NINETY_NINE_PERCENT_FIELD = 5
SAMPLE_SIGNAL_FIELD = 6
SAMPLE_ERE_FIELD = 7
SAMPLE_NAME_FIELD = 8
OPTIMIZE_FIELD = 9

# Number of these fields
NUM_FIELDS = 10

# Text label associated with each field
FIELD_NAMES = ['Bandgap in eV','Thickness in mm','LED Current in mA','SunsERE Measurement Direction (up, down, random)',
               '2% Signal','99% Signal','Measured Signal','Measured ERE','Sample Name','Optimize Stage Height (yes,no)']

# Storage of optical data, including the sensor's responsivity
response = np.genfromtxt('Responsivity.csv',delimiter=',')
lp_785 = np.genfromtxt('Semrock_BLP01-785R_Transmission.csv',delimiter=',')
lp_685 = np.genfromtxt('FF01-685.txt',delimiter='\t')
felh0700 = np.genfromtxt('Thorlabs_FELH0700_Transmission.csv',delimiter=',')
felh0800 = np.genfromtxt('Thorlabs_FELH0800_Transmission.csv',delimiter=',')
ff735_di02 = np.genfromtxt('FF735-Di02_Transmission.csv',delimiter=',')
di02_r635 = np.genfromtxt('Di02-R635.txt',delimiter='\t')
bsn10 = np.genfromtxt('BSN10_400-700nm_T90-R10_BiasLightSampling.csv',delimiter=',')
bsw11 = np.genfromtxt('BSW11_700-1100nm_T50-R50_LaserBeamBending.csv',delimiter=',')
bswxx = np.genfromtxt('BSWxx_600-1700nm_T50-R50_LaserBeamBending.csv',delimiter=',')
bsn11 = np.genfromtxt('BSN11_700-1100nm_T90-R10_LaserBeamBending.csv',delimiter=',')
lp_808 = np.genfromtxt('LP_808.txt',delimiter='\t')
lens_75 = np.genfromtxt('75_Lens_Both_B_Models.csv',delimiter=',')

# Storage of all of the optical data
combined = np.array([response,lp_785,lp_685,felh0700,felh0800,ff735_di02,di02_r635,
                     bsn10,bsw11,bswxx,bsn11,lp_808,lens_75])

# Amount of each optic, NOT including the responsivity (starts at lp_785)
optics_amount = [0,0,0,0,0,0,0,1,1,0,0,2]

# Number of optics
NUM_OPTICS = 12

# Location of responsivity data in 'combined'
RESPONSE = 0

# Interpolation of the responsivity data to a given range
interpolated_arrays = np.zeros_like(combined)
start_wavelength = 500.0
stop_wavelength = 1700.0
num_points = 6000
wavelength_step = (stop_wavelength-start_wavelength)/num_points
wavelengths = np.zeros([num_points,1])
wavelengths[0] = start_wavelength
for index in range(1,len(wavelengths)):
    wavelengths[index] = wavelengths[index-1] + wavelength_step
for index in range(len(combined)):
    interped = interp1d(combined[index][:,0],combined[index][:,1])
    extraped = extrap1d(interped)
    interpolated_arrays[index] = extraped(wavelengths)
c = np.concatenate([wavelengths,interpolated_arrays[RESPONSE]],axis=1)

# Calculating the fraction of light absorbed by the optics at the laser's wavelength
laser_wavelength = float(670)
laser_bandgap = hc/laser_wavelength # in eV

laser_optics_factor = 1
for optic in range(NUM_OPTICS):
    mult = float(optics_amount[optic])
    laser_optics_factor = laser_optics_factor*np.power(interpolated_arrays[optic+1][np.argmin(np.absolute(wavelengths-laser_wavelength))],mult)
sample_optics_factor = laser_optics_factor
laser_optics_factor = (laser_optics_factor*interpolated_arrays[RESPONSE][np.argmin(np.absolute(wavelengths-laser_wavelength))]*(laser_bandgap*Q))
laser_optics_factor = laser_optics_factor[0]

# Function to find serial_ports for troubleshooting purposes
def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
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
    return result

# The class which manages the hardware and GUI
class Plotter:
    
  def __init__(self):
    print("Initializing the PiScope...")
    return
#   
# The helper function which takes in signal data from the lock-in amplifier

# Return is null, modifies self.signal_max variable
# Commented out code is to move the motor downward from 
# a given position to find the maximum signal. Do not enable during operation
# of SunsERE functionality.
  def measure(self, start_pos=0, move = False):   
    self.signal_max = 0
    if (self.ents[OPTIMIZE_FIELD].get() == 'yes'):
        move = True
    if move:
        time_constant = self.task.get_time_constant()
        for pos in np.linspace(start_pos,positionMax,measurementPositionAmount):
            self.motor.openDevice()
            self.motor.moveToPosition_mm(pos)
            self.motor.closeDevice()
            time.sleep(5*time_constant)
            signal = self.task.get_x()
            if(signal > self.task.get_sensitivity()*1e-6):
                self.task.quick_range()
            if(signal>self.signal_max):
                self.signal_max = signal
                self.position_max = pos
        self.motor.openDevice()
        self.motor.moveToPosition_mm(self.position_max)
        self.motor.closeDevice()
    else:
        self.signal_max = self.task.get_x()
    
# Function for taking in a signal and storing it in the 2% field
  def measure_2(self,ents):
    self.measure(spectralonThickness)
    self.ents[TWO_PERCENT_FIELD].delete(0,50)
    self.ents[TWO_PERCENT_FIELD].insert(0,self.signal_max)

# Function for taking in a signal and storing it in the 99% field
  def measure_99(self,ents):
    self.measure(spectralonThickness)
    self.ents[NINETY_NINE_PERCENT_FIELD].delete(0,50)
    self.ents[NINETY_NINE_PERCENT_FIELD].insert(0,self.signal_max)

# Takes a signle measurement of the sample. Requires bandgap, 2%, and 99% data.
  def measure_sample(self,ents):
    two_percent_photons = float(self.ents[TWO_PERCENT_FIELD].get())/laser_optics_factor
    nine_nine_percent_photons = float(self.ents[NINETY_NINE_PERCENT_FIELD].get())/laser_optics_factor
    zero_photons = 0
    one_hundred_photons = zero_photons + 100*((nine_nine_percent_photons - two_percent_photons)/(99.0-2.0))
    print(zero_photons, one_hundred_photons)
    global interpolation_function
    interpolation_function = interp1d([zero_photons,one_hundred_photons],[0,100])
    start_thick = float(self.ents[THICKNESS_FIELD].get())
    self.measure(start_thick)
    self.ents[SAMPLE_SIGNAL_FIELD].delete(0,50)
    self.ents[SAMPLE_SIGNAL_FIELD].insert(0,self.signal_max)
    bandgap = float(self.ents[BANDGAP_FIELD].get())
    wavelength_sample = (hc/bandgap) # in nm
    sample_optics_factor = 1
    optics_amount[1] = 1 # Add in the longpass filter to the calculation
    for optic in range(NUM_OPTICS):
        mult = float(optics_amount[optic])
        sample_optics_factor = sample_optics_factor*np.power(interpolated_arrays[optic+1][np.argmin(np.absolute(wavelengths-wavelength_sample))],mult)
    sample_optics_factor = (sample_optics_factor*interpolated_arrays[RESPONSE][np.argmin(np.absolute(wavelengths-wavelength_sample))]*(bandgap*Q))
    sample_optics_factor = sample_optics_factor[0]
    sample_photons = (self.signal_max)/(sample_optics_factor)
    print(sample_photons)
    ERE_OUT = interpolation_function(sample_photons)
    self.ents[SAMPLE_ERE_FIELD].delete(0,50)
    self.ents[SAMPLE_ERE_FIELD].insert(0,ERE_OUT)
    
# Takes an 100 point SunsERE dataset. Requires 2%, 99%, samplename and bandgap data.
# Will measure from darkest to brightest "up" as the default
# By typing 'down' or 'random' into the SunsERE direction field, the measurement
# will go brightest to darkest, or random order of brightnesses within the range.
  def measure_suns_ere(self,ents):
    start_thick = float(self.ents[THICKNESS_FIELD].get())
    bandgap = float(self.ents[BANDGAP_FIELD].get())
    wavelength_sample = (hc/bandgap) # in nm
    sample_optics_factor = 1
    optics_amount[1] = 1 # change the longpass filter
    for optic in range(NUM_OPTICS):
        mult = float(optics_amount[optic])
        sample_optics_factor = sample_optics_factor*np.power(interpolated_arrays[optic+1][np.argmin(np.absolute(wavelengths-wavelength_sample))],mult)
    sample_optics_factor = (sample_optics_factor*interpolated_arrays[RESPONSE][np.argmin(np.absolute(wavelengths-wavelength_sample))]*(bandgap*Q))
    sample_optics_factor = sample_optics_factor[0]
    two_percent_photons = float(self.ents[TWO_PERCENT_FIELD].get())/laser_optics_factor
    nine_nine_percent_photons = float(self.ents[NINETY_NINE_PERCENT_FIELD].get())/laser_optics_factor
    zero_photons = 0
    one_hundred_photons = zero_photons + 100*((nine_nine_percent_photons - two_percent_photons)/(99.0-2.0))
    print(zero_photons, one_hundred_photons)
    global interpolation_function
    interpolation_function = interp1d([zero_photons,one_hundred_photons],[0,100])
    self.task.set_time_constant(9)
    led.setLedOnOff(True)
    ledCurrents = np.logspace(1,4,100)
    if (self.ents[SUNS_ERE_DIRECTION].get() == 'down'):
        ledCurrents = np.flip(ledCurrents)
    if (self.ents[SUNS_ERE_DIRECTION].get() == 'random'):
        np.random.shuffle(ledCurrents)
    suns_ere_data = np.zeros((len(ledCurrents),5))
    suns_ere_data[:,0] = ledCurrents
    i = 0
    for current in [x * 0.001 for x in ledCurrents]:
        led.setConstCurrent(float(current))
        time.sleep(1.5)
        all_measurements = 0
        for placeholder in range(10):
            self.measure(start_thick)
            all_measurements = all_measurements + self.signal_max
            time.sleep(0.05)
        average_measurement = all_measurements/10.0
        suns_ere_data[i][1] = average_measurement
        print(current,suns_ere_data[i][1])
        sample_photons = average_measurement/(sample_optics_factor)
        try: 
            suns_ere_data[i][2]=sample_photons
            suns_ere_data[i][3]=interpolation_function(sample_photons)
            suns_ere_data[i][4]=nidaq.read()
            print(suns_ere_data[i])
        except:   
            print(sample_photons,0.0000000000)
        i = i + 1
    led.setLedOnOff(False)
    np.savetxt(self.ents[SAMPLE_NAME_FIELD].get()+"SunsERE.csv",suns_ere_data,fmt='%s',delimiter=',',header='LED Current (mA),Signal (A),Photons,ERE (%)',comments="")
    self.ents[SAMPLE_SIGNAL_FIELD].delete(0,50)
    self.ents[SAMPLE_SIGNAL_FIELD].insert(0,self.signal_max)
    sample_photons = (self.signal_max)/(sample_optics_factor)
    print(sample_photons)
    ERE_OUT = interpolation_function(sample_photons)
    self.ents[SAMPLE_ERE_FIELD].delete(0,50)
    self.ents[SAMPLE_ERE_FIELD].insert(0,ERE_OUT)
    self.ax2.cla()
    self.ax2.plot(suns_ere_data[:,0],suns_ere_data[:,3])
  def zoom_in(self):
      self.task.set_sensitivity(self.task.get_sensitivity_int()-1)
  def light_on(self):
      led.setConstCurrent(float(self.ents[MAX_LED_CURRENT_FIELD].get())/1000.0)
      led.setLedOnOff(True)
  def light_off(self):
      led.setLedOnOff(False)

  def setup(self, channels):
    self.signal_max = 0
    self.position_max = 0
    self.channels = channels
    self.motor = TLabs_Motor.TLabs_Motor(27,'27257842')
    self.motor.openDevice()
    self.motor.homeDevice()
    self.motor.moveToPosition_mm(11.8)
    self.motor.closeDevice()
    self.ents = 0
    print("Connecting to lock-in...")
    self.task = SR830(serial_ports()[0])
    # Setup oscilloscope window
    self.root = tkinter.Tk()
    self.var = tkinter.IntVar(self.root)
    self.var.set(5)
    self.scale_factor = 1
    self.root.wm_title("PiScope")
    if len(self.channels) == 1:
      # Create x and y axis
      xAchse = pylab.arange(0, 4000, 1)
      yAchse = pylab.array([0]*4000)
      # Create the plot
      fig = pylab.figure(1)
      self.ax = fig.add_subplot(121)
      self.ax2 = fig.add_subplot(122)
      self.ax2.set_title("SunsERE Curve")
      self.ax2.set_xlabel("LED Current")
      self.ax2.set_ylabel("ERE")
      self.ax.set_title("Amplifier Signal")
      self.ax.set_xlabel("Time")
      self.ax.set_ylabel("Amps")
      self.ax.axis([0, 4000, 0, self.task.get_sensitivity()])
    elif len(self.channels) == 2:
      # Create x and y axis
      xAchse = pylab.array([0]*4000)
      yAchse = pylab.array([0]*4000)
      # Create the plot
      fig = pylab.figure(1)
      self.ax = fig.add_subplot(111)
      self.ax.set_title("X-Y Plotter")
      self.ax.set_xlabel("Channel " + str(self.channels[0]))
      self.ax.set_ylabel("Channel " + str(self.channels[1]))
      self.ax.axis([0, 3.5, 0, 3.5])
    self.ax.grid(True)
    self.line1 = self.ax.plot(xAchse, yAchse, '-')
    # Integrate plot on oscilloscope window
    self.drawing = FigureCanvasTkAgg(fig, master=self.root)
    self.drawing.draw()
    self.drawing.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)
    # Setup navigation tools
    tool = NavigationToolbar2Tk(self.drawing, self.root)
    tool.update()
    self.drawing._tkcanvas.pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)
    return
   
  def plot(self):
    if len(self.channels) < 1 or len(self.channels) > 2:
      print("The device can either operate as oscilloscope (1 channel) or x-y plotter (2 channels). Please operate accordingly.")
      self._quit()
    else:
      # print("Plotting will start in a new window...")
      try:
        # Setup Quit button
        button = tkinter.Button(master=self.root, text='Quit', command=self._quit)
        button.pack(side=tkinter.BOTTOM)
        # Setup speed and width
        self.scale1 = tkinter.Scale(master=self.root,label="View Width:", from_=3, to=1000, sliderlength=30, length=self.ax.get_window_extent().width, orient=tkinter.HORIZONTAL)
        self.scale2 = tkinter.Scale(master=self.root,label="Generation Speed:", from_=1, to=200, sliderlength=30, length=self.ax.get_window_extent().width, orient=tkinter.HORIZONTAL)
        self.scale2.pack(side=tkinter.BOTTOM)
        self.scale1.pack(side=tkinter.BOTTOM)
        self.scale1.set(4000)
        self.scale2.set(self.scale2['to']-10)
        self.ents = []                             # create an empty list
        for index in range(NUM_FIELDS):          # for each of the fields to create
           row = tk.Frame(self.root)                     # get the row and create the label
           lab = tk.Label(row, width=50,text=FIELD_NAMES[index]+": ", anchor='w')
        
           ent = tk.Entry(row)                      # create the entry and init to 0
           ent.insert(0,"0")
        
           # fill allows the widget to take extra space: X, Y, BOTH, default=NONE
           # expand allows the widget to use up sapce in the parent widget
           row.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)   # place it in the GUI
           lab.pack(side=tk.LEFT)
           ent.pack(side=tk.RIGHT, expand=tk.YES, fill=tk.X)
           self.ents.append(ent)                   # add it to the list
        b4 = tkinter.Button(self.root,text='Measure 2%',command=(lambda e=self.ents: self.measure_2(self.ents)))
        b4.pack(side=tk.LEFT, padx=5, pady=5)
        b4 = tk.Button(self.root,text='Measure 99%',command=(lambda e=self.ents: self.measure_99(self.ents)))
        b4.pack(side=tk.LEFT, padx=5, pady=5)
        b4 = tk.Button(self.root,text='Measure Sample',command=(lambda e=self.ents: self.measure_sample(self.ents)))
        b4.pack(side=tk.LEFT, padx=5, pady=5)
        b4 = tk.Button(self.root,text='Measure SunsERE',command=(lambda e=self.ents: self.measure_suns_ere(self.ents)))
        b4.pack(side=tk.LEFT, padx=5, pady=5)
        b5 = tk.Button(self.root,text='Zoom In',command=(lambda e=self: self.zoom_in()))
        b5.pack(side=tk.LEFT, padx=5, pady=5)
        b6 = tk.Button(self.root,text='Light On/Change Brightness',command=(lambda e=self: self.light_on()))
        b6.pack(side=tk.LEFT, padx=5, pady=5)
        b7 = tk.Button(self.root,text='Light Off',command=(lambda e=self: self.light_off()))
        b7.pack(side=tk.LEFT, padx=5, pady=5)
        self.root.protocol("WM_DELETE_WINDOW", self._quit)
        if len(self.channels) == 1:
          self.values = []
        else:
          self.valuesx = [0 for x in range(4000)]
          self.valuesy = [0 for y in range(4000)]
        self.root.after(4000, self.draw)
        tkinter.mainloop()
      except:
        print("Error. Try again.")
        self._quit()
    return

  def read(self):
    # print("Reading channels...")
    # print(task.read())
    # Read channels in single-ended mode (1 bit = 3mV)
    if len(self.channels) == 1:
      self.values.append(self.task.get_x())
      if(self.values[-1] > self.task.get_sensitivity()*1e-6):
          self.task.quick_range()
    elif len(self.channels) == 2:
      self.valuesx.append(self.task.read()[0])
      self.valuesy.append(self.task.read()[1])
    return self.values[-1]

  def draw(self):
    self.read() #read current values
    #self.read_test() #testing reading
    # print("Plotting...")
    if len(self.channels) == 1:
      NumberSamples = min(len(self.values), self.scale1.get())
      CurrentXAxis = pylab.arange(len(self.values) - NumberSamples, len(self.values), 1)
      self.line1[0].set_data(CurrentXAxis, pylab.array(self.values[-NumberSamples:]))
      self.ax.axis([CurrentXAxis.min(), CurrentXAxis.max(), -self.task.get_sensitivity()*1e-6, self.task.get_sensitivity()*1e-6])
    elif len(self.channels) == 2:
      NumberSamplesx = min(len(self.valuesx), self.scale1.get())
      NumberSamplesy = min(len(self.valuesy), self.scale1.get())
      self.line1[0].set_data(pylab.array(self.valuesx[-NumberSamplesx:]), pylab.array(self.valuesy[-NumberSamplesy:]))
    self.drawing.draw()
    self.root.after(25, self.draw)
    return
      

  def _quit(self):
      print("Quitting program.")
      self.task.close()
      self.root.quit()
      self.root.destroy()
