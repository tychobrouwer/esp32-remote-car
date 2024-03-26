import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class AnimationPlot:
    def animate(self, _, dataList, ser):
        ser.write(b'g')
        arduinoData_string = ser.readline()

        try:
            arduinoData_array = arduinoData_string.decode('ascii').strip().split(',')

            dataList[0].append(float(arduinoData_array[0]))
            dataList[1].append(float(arduinoData_array[1]))
            dataList[2].append(float(arduinoData_array[2]))
            dataList[3].append(float(arduinoData_array[3]))
            
        except:     
            print("Failed to read Serial Data from Arduino")                     
            pass
                
        self.getPlotFormat()

        for i, line in enumerate(lines):
            line.set_data(range(len(dataList[i])), dataList[i])        
        
    def getPlotFormat(self):
        ax.set_ylim([0, 1])
        ax.set_xlim([0, len(dataList[0])])
        ax.set_title("Arduino Data")
        ax.set_ylabel("Value")

dataList = [[], [], [], []]
                                                        
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to main fig window

lines = [ax.plot([], [], label=f'Channel {i+1}')[0] for i in range(4)] # Create lines for each channel

realTimePlot = AnimationPlot()

ser = serial.Serial("COM3", 9600)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, realTimePlot.animate, fargs=(dataList, ser), interval=1) 

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()                                             # Close Serial connection when plot is closed