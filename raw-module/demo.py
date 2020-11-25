import rawcopter
import tkinter as tk
import time

output_pending = False
output_m1 = 0
output_m2 = 0
output_m3 = 0
output_m4 = 0

val1 = None
val2 = None
val3 = None
val4 = None
valMaster = None

valPitch = None
valRoll = None
valYaw = None
valAlt = None

def scale_change(event):
    global val1, val2, val3, val4
    global output_m1, output_m2, output_m3, output_m4, output_pending
    output_m1 = val1.get()
    output_m2 = val2.get()
    output_m3 = val3.get()
    output_m4 = val4.get()
    output_pending = True

def master_change(event):
    global val1, val2, val3, val4, valMaster
    global output_m1, output_m2, output_m3, output_m4, output_pending
    val1.set(valMaster.get())
    val2.set(valMaster.get())
    val3.set(valMaster.get())
    val4.set(valMaster.get())
    output_m1 = output_m2 = output_m3 = output_m4 = valMaster.get()
    output_pending = True

def create_window():
    window = tk.Tk()
    frameOut = tk.Frame(window)
    frameIn = tk.Frame(window)
    frameIn.pack(side = tk.TOP)
    frameOut.pack(side = tk.TOP)
    
    global val1, val2, val3, val4, valMaster
    val1 = tk.IntVar()
    val2 = tk.IntVar()
    val3 = tk.IntVar()
    val4 = tk.IntVar()
    valMaster = tk.IntVar()

    global valm1, valm2, valm3, valm4
    scale1 = tk.Scale(frameOut, showvalue = False, from_ = 2000, to = 0, variable = val1, command = scale_change)
    scale2 = tk.Scale(frameOut, showvalue = False, from_ = 2000, to = 0, variable = val2, command = scale_change)
    scale3 = tk.Scale(frameOut, showvalue = False, from_ = 2000, to = 0, variable = val3, command = scale_change)
    scale4 = tk.Scale(frameOut, showvalue = False, from_ = 2000, to = 0, variable = val4, command = scale_change)
    scaleMaster = tk.Scale(frameOut, showvalue = False, from_ = 2000, to = 0, variable = valMaster, command = master_change)
    lblm1 = tk.Label(frameOut, text = "Motor 1")
    lblm2 = tk.Label(frameOut, text = "Motor 2")
    lblm3 = tk.Label(frameOut, text = "Motor 3")
    lblm4 = tk.Label(frameOut, text = "Motor 4")
    lblMaster = tk.Label(frameOut, text = "Master")
    valm1 = tk.Label(frameOut, text = "0")
    valm2 = tk.Label(frameOut, text = "0")
    valm3 = tk.Label(frameOut, text = "0")
    valm4 = tk.Label(frameOut, text = "0")
    scale1.grid(row = 0, column = 0)
    scale2.grid(row = 0, column = 1)
    scale3.grid(row = 0, column = 2)
    scale4.grid(row = 0, column = 3)
    scaleMaster.grid(row = 0, column = 4)
    lblm1.grid(row = 1, column = 0)
    lblm2.grid(row = 1, column = 1)
    lblm3.grid(row = 1, column = 2)
    lblm4.grid(row = 1, column = 3)
    lblMaster.grid(row = 1, column = 4)
    valm1.grid(row = 2, column = 0)
    valm2.grid(row = 2, column = 1)
    valm3.grid(row = 2, column = 2)
    valm4.grid(row = 2, column = 3)
    
    global valPitch, valRoll, valYaw, valAlt
    lblPitch = tk.Label(frameIn, text = "pitch")
    lblRoll = tk.Label(frameIn, text = "roll")
    lblYaw = tk.Label(frameIn, text = "yaw")
    lblAlt = tk.Label(frameIn, text = "altitude")
    valPitch = tk.Label(frameIn, text = "0.0", width = 10)
    valRoll = tk.Label(frameIn, text = "0.0", width = 10)
    valYaw = tk.Label(frameIn, text = "0.0", width = 10)
    valAlt = tk.Label(frameIn, text = "0.0", width = 10)

    lblPitch.grid(row = 0, column = 0)
    lblRoll.grid(row = 0, column = 1)
    lblYaw.grid(row = 0, column = 2)
    lblAlt.grid(row = 0, column = 3)
    valPitch.grid(row = 1, column = 0)
    valRoll.grid(row = 1, column = 1)
    valYaw.grid(row = 1, column = 2)
    valAlt.grid(row = 1, column = 3)

    window.after(100, synchronize, window)
    return window

def synchronize(window):
    global output_m1, output_m2, output_m3, output_m4, output_pending
    global valm1, valm2, valm3, valm4
    global valPitch, valRoll, valYaw, valAlt

    window.after(100, synchronize, window)
    data = rawcopter.input()
    if (data != None):
        (pitch, roll, yaw, altitude) = data
        valPitch['text'] = '{:.4f}'.format(pitch)
        valRoll['text'] = '{:.4f}'.format(roll)
        valYaw['text'] = '{:.4f}'.format(yaw)
        valAlt['text'] = '{:.4f}'.format(altitude)

    if output_pending:
        rawcopter.output(output_m1, output_m2, output_m3, output_m4)
        valm1['text'] = str(output_m1)
        valm2['text'] = str(output_m2)
        valm3['text'] = str(output_m3)
        valm4['text'] = str(output_m4)
        output_pending = False

def main():
    if not rawcopter.connect():
        print("MAVLink with Arducopter is not available!")
        exit()

    window = create_window()
    window.mainloop()
    rawcopter.disconnect()

if __name__ == '__main__':
    main()