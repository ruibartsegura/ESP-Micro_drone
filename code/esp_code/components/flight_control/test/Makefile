# Tests unitarios en host (sin ESP-IDF):   make test
# Estructura esperada:  components/flight_control/test/   (este directorio)
COMP    ?= ../..
CONTROLLER_SRC ?= ../attitude_controller.c

INCS := -Imocks -Iunity -I.. -I../include \
        -I$(COMP)/drivers/imu/include \
        -I$(COMP)/drivers/height/include \
        -I$(COMP)/drivers/motors/include \
        -I$(COMP)/ros_coordinator/include \
        -I$(COMP)/system/include \
        -I$(COMP)/configurations/parameters/include

CC      ?= gcc
CFLAGS  := -std=gnu11 -O0 -g -Wall -Wextra -Wno-unused-parameter $(INCS) \
           -DCONTROLLER_SRC='"$(CONTROLLER_SRC)"'
BIN     := build/test_attitude_controller

test: $(BIN)
	./$(BIN)

$(BIN): test_attitude_controller.c $(CONTROLLER_SRC) unity/unity.c
	@mkdir -p build
	$(CC) $(CFLAGS) test_attitude_controller.c unity/unity.c -o $@ -lm

clean:
	rm -rf build
.PHONY: test clean
