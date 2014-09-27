import time
import RPi.GPIO as io
io.setmode(io.BCM)

io.setup(23, io.OUT)

while True:
	print("POWER ON")
	io.output(23, True)
	time.sleep(10);
	print("POWER OFF")
	io.output(23, False)
	time.sleep(10)
