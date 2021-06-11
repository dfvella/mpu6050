BOARD = arduino:avr:uno
PORT = /dev/ttyACM0

# if USB port is found, then set flags so the board is flashed
FLAGS = --board $(BOARD)
FOUND = $(shell find /dev -wholename $(PORT))
FLAGS += $(if $(FOUND),--port $(PORT) --upload,--verify)

mpu6050_angles:
	cp mpu6050.* $@/
	arduino $(FLAGS) $@/$@.ino
	rm $@/mpu6050.*

mpu6050_calibrate:
	cp mpu6050.* $@/
	arduino $(FLAGS) $@/$@.ino
	rm $@/mpu6050.*

mpu6050_raw:
	cp mpu6050.* $@/
	arduino $(FLAGS) $@/$@.ino
	rm $@/mpu6050.*

.PHONY: mpu6050_angles mpu6050_calibrate mpu6050_raw
