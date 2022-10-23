import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sys
import datetime

mpl.rcParams['font.size'] = 8

argc = len(sys.argv)
if (argc > 1):
    path = sys.argv[1]
    if (argc > 2):
        save_path = sys.argv[2]
    else:
        save_path = 'plots/'
else:
    path = 'autopilot.log'
    save_path = 'plots/'

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
        axis[0].set_ylabel('vx / m/s')
        axis[0].grid(True)
        axis[1].plot(time_list, self.vy, 'k-', lw=1)
        axis[1].set_ylabel('vy / m/s')
        axis[1].grid(True)
        axis[2].plot(time_list, self.vz, 'k-', lw=1)
        axis[2].set_ylabel('vz / m/s')
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
        figure, axis = plt.subplots(3, 2, sharex='col', figsize=(12, 8))
        figure.suptitle('Translation', fontsize=12)
        axis[0, 0].plot(time_list, self.x, 'k-', lw=1)
        axis[0, 0].set_title('Position', fontsize=10)
        axis[0, 0].set_ylabel('x / m')
        axis[0, 0].tick_params(axis='y')
        axis[0, 0].grid(True)
        axis[1, 0].plot(time_list, self.y, 'k-', lw=1)
        axis[1, 0].set_ylabel('y / m')
        axis[1, 0].tick_params(axis='y')
        axis[1, 0].grid(True)
        axis[2, 0].plot(time_list, self.z, 'k-', lw=1)
        axis[2, 0].set_ylabel('z / m')
        axis[2, 0].tick_params(axis='x')
        axis[2, 0].tick_params(axis='y')
        axis[2, 0].grid(True)
        axis[0, 1].plot(time_list, self.vx, 'k-', lw=1)
        axis[0, 1].set_title('Velocity', fontsize=10)
        axis[0, 1].set_ylabel('x / m/s')
        axis[0, 1].tick_params(axis='y')
        axis[0, 1].grid(True)
        axis[1, 1].plot(time_list, self.vy, 'k-', lw=1)
        axis[1, 1].set_ylabel('y / m/s')
        axis[1, 1].tick_params(axis='y')
        axis[1, 1].grid(True)
        axis[2, 1].plot(time_list, self.vz, 'k-', lw=1)
        axis[2, 1].set_ylabel('z / m/s')
        axis[2, 1].tick_params(axis='x')
        axis[2, 1].tick_params(axis='y')
        axis[2, 1].grid(True)
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

# Specify what data the log consists of per line
data_blocks = (TranslationData(), AttitudeData())

time_list = []

with open(path, 'r') as file:
    for line in file:
        time, data = line.split(':')

        # Read time
        year, month, day, t = time.split('-')
        if (int(month) == 0):  # Work around data logging bug
            continue
        hours = int(t[0:2])
        minutes = int(t[2:4])
        seconds = int(t[4:6])
        centiseconds = int(t[6:8])
        time_list.append(datetime.datetime(int(year), int(month), int(day), hours, minutes, seconds, centiseconds * 1000))

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