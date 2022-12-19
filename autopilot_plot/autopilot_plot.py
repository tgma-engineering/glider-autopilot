"""
Autopilot Log Plotting Tool
Call from the Autopilot Log Plotting Tool's directory with
$ python3 autopilot_plot.py path/to/autopilot.log start_time end_time
Time format is YYYY-MM-DD-HHMMSSCC (Second and Centisecond are optional)
Plots will be saved to ./plot/
Variable data_blocks stores how the data is formatted in the log file lines.
"""

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sys
import datetime

# Example times (Hours in GMT)
# 2022-10-30-1115
# 2022-10-30-11220000
def str_to_time(str):
    # Read time
    year, month, day, t = str.split('-')
    # Work around data logging bug
    if (int(month) < 1 or int(month) > 12):
        return None
    if (int(day) < 1 or int(day) > 31):
        return None
    hours = int(t[0:2])
    minutes = int(t[2:4])
    if (len(t) > 4):
        seconds = int(t[4:6])
    else:
        seconds = 0
    if (len(t) > 6):   
        centiseconds = int(t[6:8])
    else:
        centiseconds = 0
    return datetime.datetime(int(year), int(month), int(day), hours, minutes, seconds, centiseconds * 1000)

mpl.rcParams['font.size'] = 8

argc = len(sys.argv)
if (argc > 1):
    path = sys.argv[1]
else:
    path = 'autopilot.log'

if (argc > 3):
    start_time = str_to_time(sys.argv[2])
    end_time = str_to_time(sys.argv[3])
else:
    start_time = None
    end_time = None

save_path = 'plots/'

def update_mean(new_val, old_mean, old_num):
    return (new_val + old_num * old_mean) / (old_num + 1)

class PositionData:
    def __init__(self, name='position'):
        self.length = 3
        self.name = name
        self.x = []
        self.y = []
        self.z = []

    def read(self, data):
        self.x.append(float(data[0]))
        self.y.append(float(data[1]))
        self.z.append(float(data[2]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(3, 1, sharex='col', figsize=(6, 8))
        figure.suptitle('Position', fontsize=12)
        axis[0].plot(time_list, self.x, 'k-', lw=1)
        axis[0].set_ylabel('x / m')
        axis[0].grid(True)
        axis[1].plot(time_list, self.y, 'k-', lw=1)
        axis[1].set_ylabel('y / m')
        axis[1].grid(True)
        axis[2].plot(time_list, self.z, 'k-', lw=1)
        axis[2].set_ylabel('z / m')
        axis[2].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class VelocityData:
    def __init__(self, name='velocity'):
        self.length = 3
        self.name = name
        self.vx = []
        self.vy = []
        self.vz = []

    def read(self, data):
        self.vx.append(float(data[0]))
        self.vy.append(float(data[1]))
        self.vz.append(float(data[2]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(3, 1, sharex='col', figsize=(6, 8))
        figure.suptitle('Velocity', fontsize=12)
        axis[0].plot(time_list, self.vx, 'k-', lw=1)
        axis[0].set_ylabel('x / m/s')
        axis[0].grid(True)
        axis[1].plot(time_list, self.vy, 'k-', lw=1)
        axis[1].set_ylabel('y / m/s')
        axis[1].grid(True)
        axis[2].plot(time_list, self.vz, 'k-', lw=1)
        axis[2].set_ylabel('z / m/s')
        axis[2].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class TranslationData:  # Position and velocity
    def __init__(self, name='translation'):
        self.length = 6
        self.name = name
        self.x = []
        self.y = []
        self.z = []
        self.vx = []
        self.vy = []
        self.vz = []

    def read(self, data):
        self.x.append(float(data[0]))
        self.y.append(float(data[1]))
        self.z.append(float(data[2]))
        self.vx.append(float(data[3]))
        self.vy.append(float(data[4]))
        self.vz.append(float(data[5]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(4, 2, sharex='col', figsize=(12, 11))
        figure.suptitle('Translation', fontsize=12)
        axis[0, 0].plot(time_list, self.x, 'k-', lw=1)
        axis[0, 0].set_title('Position', fontsize=10)
        axis[0, 0].set_ylabel('x / m')
        axis[0, 0].grid(True)
        axis[1, 0].plot(time_list, self.y, 'k-', lw=1)
        axis[1, 0].set_ylabel('y / m')
        axis[1, 0].grid(True)
        axis[2, 0].plot(time_list, self.z, 'k-', lw=1)
        axis[2, 0].set_ylabel('z / m')
        axis[2, 0].grid(True)
        axis[3, 0].plot(time_list, [np.sqrt(self.x[i]**2 + self.y[i]**2 + self.z[i]**2) for i in range(len(self.x))], 'k-', lw=1)
        axis[3, 0].set_title('Distance from Home', fontsize=10)
        axis[3, 0].set_ylabel('d / m')
        axis[3, 0].grid(True)

        axis[0, 1].plot(time_list, self.vx, 'k-', lw=1)
        axis[0, 1].set_title('Velocity', fontsize=10)
        axis[0, 1].set_ylabel('x / m/s')
        axis[0, 1].grid(True)
        axis[1, 1].plot(time_list, self.vy, 'k-', lw=1)
        axis[1, 1].set_ylabel('y / m/s')
        axis[1, 1].grid(True)
        axis[2, 1].plot(time_list, self.vz, 'k-', lw=1)
        axis[2, 1].set_ylabel('z / m/s')
        axis[2, 1].grid(True)
        axis[3, 1].plot(time_list, [np.sqrt(self.vx[i]**2 + self.vy[i]**2 + self.vz[i]**2) for i in range(len(self.vx))], 'k-', lw=1)
        axis[3, 1].set_title('Speed', fontsize=10)
        axis[3, 1].set_ylabel('v / m/s')
        axis[3, 1].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class AttitudeData:
    def __init__(self, name='attitude'):
        self.length = 4
        self.name = name
        self.w = []
        self.x = []
        self.y = []
        self.z = []

    def read(self, data):
        self.w.append(float(data[0]))
        self.x.append(float(data[1]))
        self.y.append(float(data[2]))
        self.z.append(float(data[3]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(4, 1, sharex='col', figsize=(6, 8))
        figure.suptitle('Attitude', fontsize=12)
        axis[0].plot(time_list, self.w, 'k-', lw=1)
        axis[0].set_ylabel('w')
        axis[0].grid(True)
        axis[1].plot(time_list, self.x, 'k-', lw=1)
        axis[1].set_ylabel('x')
        axis[1].grid(True)
        axis[2].plot(time_list, self.y, 'k-', lw=1)
        axis[2].set_ylabel('y')
        axis[2].grid(True)
        axis[3].plot(time_list, self.z, 'k-', lw=1)
        axis[3].set_ylabel('z')
        axis[3].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class WindData:
    def __init__(self, name='wind'):
        self.length = 3
        self.name = name
        self.wx = []
        self.wy = []
        self.wz = []
        self.mean_dir = 0.0
        self.mean_vel = 0.0

    def read(self, data):
        self.wx.append(float(data[0]))
        self.wy.append(float(data[1]))
        self.wz.append(float(data[2]))
        # Update means
        direction = np.rad2deg(np.arctan2(float(data[0]), float(data[1])))
        self.mean_dir = update_mean(direction, self.mean_dir, len(self.wx)-1)
        velocity = np.sqrt(float(data[0])**2 + float(data[1])**2)
        self.mean_vel = update_mean(velocity, self.mean_vel, len(self.wx)-1)

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(3, 1, sharex='col', figsize=(6, 8))
        figure.suptitle('Wind', fontsize=12)
        axis[0].plot(time_list, [np.rad2deg(np.arctan2(self.wx[i], self.wy[i])) for i in range(len(self.wx))], 'k-', lw=1)
        axis[0].set_title(f'Direction (Mean: {self.mean_dir} deg)', fontsize=10)
        axis[0].set_ylabel('dir / deg')
        axis[0].grid(True)
        axis[1].plot(time_list, [np.sqrt(self.wx[i]**2 + self.wy[i]**2) for i in range(len(self.wx))], 'k-', lw=1)
        axis[1].set_title(f'Horizontal Speed (Mean: {self.mean_vel} m/s)', fontsize=10)
        axis[1].set_ylabel('v_h / m/s')
        axis[1].grid(True)
        axis[2].plot(time_list, self.wz, 'k-', lw=1)
        axis[2].set_title('Vertical Speed', fontsize=10)
        axis[2].set_ylabel('v_v / m/s')
        axis[2].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class ParameterData:
    def __init__(self, name='parameter'):
        self.length = 2
        self.name = name
        self.cw = []
        self.cm = []
        self.mean_cw = 0.0
        self.mean_cm = 0.0

    def read(self, data):
        self.cw.append(float(data[0]))
        self.cm.append(float(data[1]))
        # Update means
        self.mean_cw = update_mean(float(data[0]), self.mean_cw, len(self.cw)-1)
        self.mean_cm = update_mean(float(data[1]), self.mean_cm, len(self.cm)-1)

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(2, 1, sharex='col', figsize=(6, 5))
        figure.suptitle('Parameter', fontsize=12)
        axis[0].plot(time_list, self.cw, 'k-', lw=1)
        axis[0].set_title(f'Drag Coefficient (Mean: {self.mean_cw} kg/m)', fontsize=10)
        axis[0].set_ylabel('cw / kg/m')
        axis[0].grid(True)
        axis[1].plot(time_list, self.cm, 'k-', lw=1)
        axis[1].set_title(f'Motor Coefficient (Mean: {self.mean_cm} N/kg)', fontsize=10)
        axis[1].set_ylabel('cm / N/kg')
        axis[1].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class ControlSurfaceData:  # Control Surface parameters and angular drift
    def __init__(self, name='control_surface'):
        self.length = 6
        self.name = name
        self.crp = []  # Control Surface Constant for Pitch
        self.crr = []  # Roll
        self.cry = []  # Yaw
        self.w0p = []  # Angular Drift for Pitch
        self.w0r = []  # Roll
        self.w0y = []  # Yaw

    def read(self, data):
        self.crp.append(float(data[0]))
        self.crr.append(float(data[1]))
        self.cry.append(float(data[2]))
        self.w0p.append(float(data[3]))
        self.w0r.append(float(data[4]))
        self.w0y.append(float(data[5]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(3, 2, sharex='col', figsize=(12, 11))
        figure.suptitle('Parameters', fontsize=12)
        axis[0, 0].plot(time_list, self.crp, 'k-', lw=1)
        axis[0, 0].set_title('Position', fontsize=10)
        axis[0, 0].set_ylabel('Pitch / rad/m')
        axis[0, 0].grid(True)
        axis[1, 0].plot(time_list, self.crr, 'k-', lw=1)
        axis[1, 0].set_ylabel('Roll / rad/m')
        axis[1, 0].grid(True)
        axis[2, 0].plot(time_list, self.cry, 'k-', lw=1)
        axis[2, 0].set_ylabel('Yaw / rad/m')
        axis[2, 0].grid(True)

        axis[0, 1].plot(time_list, self.w0p, 'k-', lw=1)
        axis[0, 1].set_title('Drift', fontsize=10)
        axis[0, 1].set_ylabel('Pitch / rad/s')
        axis[0, 1].grid(True)
        axis[1, 1].plot(time_list, self.w0r, 'k-', lw=1)
        axis[1, 1].set_ylabel('Roll / rad/s')
        axis[1, 1].grid(True)
        axis[2, 1].plot(time_list, self.w0y, 'k-', lw=1)
        axis[2, 1].set_ylabel('Yaw / rad/s')
        axis[2, 1].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class SatelliteData:
    def __init__(self, name='satellites'):
        self.length = 1
        self.name = name
        self.satellites = []

    def read(self, data):
        self.satellites.append(float(data[0]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(1, 1, sharex='col', figsize=(6, 3))
        figure.suptitle('Received Number of Satellites', fontsize=12)
        axis[0].plot(time_list, self.satellites, 'k-', lw=1)
        axis[0].set_ylabel('s')
        axis[0].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

class TimeData:
    def __init__(self, name='dt'):
        self.length = 1
        self.name = name
        self.dt = []

    def read(self, data):
        self.dt.append(float(data[0]))

    def plot(self, time_list, path=''):
        figure, axis = plt.subplots(1, 1, sharex='col', figsize=(6, 3))
        figure.suptitle('Average Tick Time', fontsize=12)
        axis[0].plot(time_list, self.dt, 'k-', lw=1)
        axis[0].set_ylabel('dt / mys')
        axis[0].grid(True)
        figure.tight_layout()
        figure.savefig(path + self.name + '.png', dpi = 500)

# Specify what data the log consists of per line
data_blocks = (TranslationData(), AttitudeData(), WindData(), ParameterData(), ControlSurfaceData(), SatelliteData(), TimeData())

time_list = []

with open(path, 'r') as file:
    for line in file:
        time_str, data = line.split(':')

        # Read time
        time = str_to_time(time_str)
        if (time is None or (not start_time is None and time < start_time)):
            continue
        elif (not end_time is None and time > end_time):
            break
        time_list.append(time)

        # Read data
        words = data.split(';')
        for block in data_blocks:
            block.read([words.pop(0) for i in range(block.length)])

# If there are multiple equal times in time_list, spread them out linearly
ref_time = time_list[0]
cnt = 0  # Counts how many 'clones' of ref_time there are after it
for i, time in enumerate(time_list):
    if (i == 0): continue  # Skip first element
    # If this element has the same time as the ones before: Increment count
    if (time == ref_time):
        cnt += 1
    else:
        if (cnt == 0):  # No clones, everything fine
            continue
        else:  # There are cnt+1 elements with the same time
            part_diff = (time - ref_time) / (cnt+1)
            for j in range(1, cnt+1):
                time_list[i-(cnt+1)+j] += part_diff * j
            ref_time = time
            cnt = 0

for block in data_blocks:
    block.plot(time_list, save_path)