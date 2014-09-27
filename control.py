import os
import glob
import time
import RPi.GPIO as io

from pid import PID

relay_pin = 23

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

io.setmode(io.BCM)
io.setup(relay_pin, io.OUT)

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c

controller = PID(0.1, 0.001, 1)
controller.setPoint(54)

while True:
	temperature = read_temp()

	control_signal = controller.update(temperature)

	if control_signal > 0:
		io.output(relay_pin, True)
		status = '1'
	else:
		io.output(relay_pin, False)
		status = '0'

	message = str(time.time()) + '\t' + str(status) + '\t' + str(temperature) + '\t' + str(control_signal)

	log = open('log', 'a')
	log.write(message + '\n')
	log.close()

	print message

	time.sleep(30)
