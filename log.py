import os
import glob
import time
import RPi.GPIO as io

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

while True:
	print("Power On")
	io.output(relay_pin, True)
	count = 0
	while count < 900:
		temperature = str(read_temp())
		log = open('log', 'a')
		log.write(str(time.time()) + '\t1\t' + temperature + '\n')
		log.close()
		time.sleep(1)
		count = count + 1
	print("Power Off")
	io.output(relay_pin, False)
	count = 0
	while count < 900:
		temperature = str(read_temp())
		log = open('log', 'a')
		log.write(str(time.time()) + '\t0\t' + temperature + '\n')
		log.close()
		time.sleep(1)
		count = count + 1
