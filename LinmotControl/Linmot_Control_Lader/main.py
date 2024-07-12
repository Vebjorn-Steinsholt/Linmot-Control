import tkinter as tk
from LinmotController import *
import time
from tkinter import messagebox
import math
import threading
import sys
import trace
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (
     FigureCanvasTkAgg)
fields = ('Acceleration[m/s^2]', 'Frequency[hertz]', 'Phase[1]', 'Length of time series[s]','Amplitude[mm]')



class thread_trace(threading.Thread):
  def __init__(self, *args, **keywords):
    threading.Thread.__init__(self, *args, **keywords)
    self.killed = False
 
  def start(self):
    self.__run_backup = self.run
    self.run = self.__run      
    threading.Thread.start(self)
 
  def __run(self):
    sys.settrace(self.globaltrace)
    self.__run_backup()
    self.run = self.__run_backup
 
  def globaltrace(self, frame, event, arg):
    if event == 'call':
      return self.localtrace
    else:
      return None
 
  def localtrace(self, frame, event, arg):
    if self.killed:
      if event == 'line':
        raise SystemExit()
    return self.localtrace
 
  def kill(self):
    self.killed = True

def startup():
    print('Turning on...')
    messagehandler(1,0,0,0,0)
    time.sleep(2)
    print('Homing...')
    messagehandler(2,0,0,0,0)
    time.sleep(5)
    print("Start up complete")

def run(entries):
    global t
    t = thread_trace(target = operation, args=(entries,))
    t.start()

def stop():
    t.kill()
    t.join()
    if not t.is_alive():
        print('Thread killed')


def check(entries):
    #intialize figure 
    fig, (ax1,ax2) = plt.subplots(1, 2, sharey=False)
    fig.tight_layout(pad = 3.0)
    #create canvas
    canvas = FigureCanvasTkAgg(fig, master = root)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    #plot the data

    time = int(entries['Length of time series[s]'].get())
    acceleration = float(entries['Acceleration[m/s^2]'].get())
    frequency = float(entries['Frequency[hertz]'].get())
    amplitude = 1000*acceleration / ((2 * math.pi * frequency)**2)
    omega = float(2 * math.pi * frequency)
    t = np.arange(0, time, .01)
    phase =  float(entries['Phase[1]'].get())

    #Position 
    ax1.plot(t, amplitude*np.sin(omega*t+phase))
    ax1.set_ylabel('x(t) [mm]')
    ax1.set_title('Position')
    ax1.legend(['x(t)'])
    ax1.set_xlabel('t [s]')
    ax1.hlines(y=95, xmin=0, xmax=time, linewidth=2, color='r', label = "Upper Limit")
    ax1.hlines(y=-95, xmin=0, xmax=time, linewidth=2, color='r', label = "Lower Limit")

    #Velocity
    ax2.plot(t, (amplitude/1000)*omega*np.cos(omega*t+phase), color = 'y')
    ax2.set_ylabel('v(t) [m/s]')
    ax2.set_title('velocity')
    ax2.legend(['v(t)'])
    ax2.set_xlabel('t [s]')
    ax2.hlines(y=2, xmin=0, xmax=time, linewidth=2, color='r', label = "Upper Limit")
    ax2.hlines(y=-2, xmin=0, xmax=time, linewidth=2, color='r', label = "Lower Limit")

    canvas.draw()
 



def operation(entries):
    acceleration = float(entries['Acceleration[m/s^2]'].get())
    if acceleration  > 9.81:
        messagebox.showerror(title = 'Invalid input', message='Acceleration  must be smaller than 9.81 m/s^2')
        raise Exception("Stopping program")
    frequency = float(entries['Frequency[hertz]'].get())
    if frequency  > 5:
        messagebox.showerror(title = 'Invalid input', message='Frequency  must be smaller than 5 hertz')
        raise Exception("Stopping program")
    amplitude = 1000*acceleration / ((2 * math.pi * frequency)**2)
    if amplitude > 95:
        messagebox.showerror(title='Invalid input', message='Given acceleration and frequency gives amplitude out of bounds')
        raise Exception("Stopping program")
    if (amplitude/1000)*frequency*2*math.pi >= 2:
        messagebox.showerror(title='Invalid input', message='Given acceleration and frequency gives velocity out of bounds')
        raise Exception("Stopping program")
    entries['Amplitude[mm]'].config(state= "normal")
    A = ("%6.2f" % amplitude).strip()
    entries['Amplitude[mm]'].delete(0, tk.END)
    entries['Amplitude[mm]'].insert(0, A)
    root.update()
    phase =  float(entries['Phase[1]'].get())
    time = float(entries['Length of time series[s]'].get())
    calc_sine(amplitude,frequency,phase,time)
    entries['Amplitude[mm]'].config(state= "readonly")
    
def makeform(root, fields):
    entries = {}
    for field in fields:
        row = tk.Frame(root)
        lab = tk.Label(row, width=22, text=field+": ", anchor='w')
        ent = tk.Entry(row)
        ent.insert(0, "0")
        row.pack(side=tk.TOP,
                 fill=tk.X,
                 padx=5,
                 pady=5)
        lab.pack(side=tk.LEFT)
        ent.pack(side=tk.RIGHT,
                 expand=tk.YES,
                 fill=tk.X)
        entries[field] = ent
    entries['Amplitude[mm]'].delete(0, tk.END)
    entries['Amplitude[mm]'].insert(0, "---")
    entries['Amplitude[mm]'].config(state= "disabled")
    entries['Acceleration[m/s^2]'].delete(0, tk.END)
    entries['Acceleration[m/s^2]'].insert(0, "input [0,9.81]")
    entries['Frequency[hertz]'].delete(0, tk.END)
    entries['Frequency[hertz]'].insert(0, "input [0,5]")
    return entries

def start_gui():
    global root
    root = tk.Tk()
    root.title("Control of motor")
    ents = makeform(root, fields)
    b1 = tk.Button(root, text='Init',
           command=(lambda: startup()))
    b1.pack(side=tk.LEFT, padx=5, pady=5)
    b2 = tk.Button(root, text='Start',bg = 'green', fg = 'white',
           command=(lambda e=ents:run(e)))
    b2.pack(side=tk.LEFT, padx=5, pady=5)
    b3 = tk.Button(root, text='Quit', command=root.quit)
    b3.pack(side=tk.LEFT, padx=5, pady=5)
    b4 = tk.Button(root, text='Stop',bg = 'red', fg = 'white', command=refresh)
    b4.pack(side=tk.LEFT, padx=5, pady=5)
    b5 = tk.Button(root, text='Check Values',
           command=(lambda e=ents:check(e)))
    b5.pack(side=tk.LEFT, padx=5, pady=5)
    b6 = tk.Button(root, text='Clear Graph(s)', command=refresh_gui)
    b6.pack(side=tk.LEFT, padx=5, pady=5)

    root.mainloop()

if __name__ == '__main__':
    def refresh():
        stop()
        root.destroy()
        start_gui()
    def refresh_gui():
        root.destroy()
        start_gui()
    start_gui()
    