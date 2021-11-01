# SunsERE
Code for operation of a SunsERE tool. Version 1.0
# Install Instructions
1. Install Thorlabs DC2200 LED Driver. This should also install NI-VISA with it.
2. Install Thorlabs Kinesis 64 bit for 64 bit Windows.
3. Install NI-DAQMx, files for .NET support, C support
4. pip install serial
5. pip install pythonnet
6. pip install pymeasure
7. pip install nidaqmx
8. Bring over copies the files "Thorlabs.MotionControl.KCube.DCServo.dll", "Thorlabs.TLDC2200_64.Interop.dll", and "Thorlabs.TLDC2200_64.Interop.xml", into the directory where your code will be operating from.
9. Download the files from Github into the directory you will be working from.
# Operation Instructions
To use in your code:
from PyScope_SR830_Clean import Plotter
piscope = Plotter()
piscope.setup([0])
piscope.plot()
