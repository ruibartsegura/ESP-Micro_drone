# ==========================================================================
#  Host unit tests (GCC + Unity, no ESP-IDF and no hardware)
#
#  make test               build and run every test, and print a summary
#  make run T=motors_sim   build and run one test with the full Unity output
#  make list               list the available tests
#  make clean
#
#  Options:
#    SAN=1                  build with AddressSanitizer + UBSan
#    MODULE_SRC=path/to.c   (with "run") test another version of the module
#    STATE= PARAMS= BMP= MPU=   other versions of the real modules that are linked
#
#  Every module is built in hardware mode (*_hw) and, if it has code for
#  simulation, also with CONFIG_SIMULATION_ON (*_sim).
# ==========================================================================
COMP  := ../components
BUILD := build
CC    ?= gcc

INCS := -I. -Imocks -Iunity \
        -I$(COMP)/drivers/leds/include \
        -I$(COMP)/drivers/motors/include \
        -I$(COMP)/drivers/imu/include \
        -I$(COMP)/drivers/height/include \
        -I$(COMP)/flight_control/include \
        -I$(COMP)/global_data/state/include \
        -I$(COMP)/global_data/parameters/include \
        -I$(COMP)/ros_coordinator/include \
        -I$(COMP)/system/include

CFLAGS := -std=gnu11 -O0 -g -Wall -Wextra \
          -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable \
          -Wno-unused-function -Wno-missing-field-initializers \
          -DUNITY_INCLUDE_DOUBLE -DUNITY_DOUBLE_PRECISION=1e-9 $(INCS)
LDLIBS := -lm -lpthread

ifeq ($(SAN),1)
CFLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
# Let the tests catch the crashes themselves (TEST_ASSERT_NO_CRASH)
export ASAN_OPTIONS ?= detect_leaks=0:handle_segv=0:handle_sigfpe=0:allow_user_segv_handler=1
endif

SIM := -DCONFIG_SIMULATION_ON=1

STATE  ?= $(COMP)/global_data/state/state.c
PARAMS ?= $(COMP)/global_data/parameters/parameters.c
BMP    ?= $(COMP)/drivers/height/bmp180.c
MPU    ?= $(COMP)/drivers/imu/mpu6050.c

COMMON := unity/unity.c mocks/mock_esp.c mocks/mock_ros.c

# --------------------------------------------------------------------------
#  Test list:  <name>_SRC = test file,  <name>_EXTRA = real modules linked,
#              <name>_FLAGS = extra compiler flags
# --------------------------------------------------------------------------
TESTS := led \
         motors_hw motors_sim \
         state parameters \
         bmp180 mpu6050 quaternions \
         height_hw height_sim \
         imu_hw imu_sim \
         attitude_controller odometry tuning \
         system_hw system_sim \
         ros_coordinator_hw ros_coordinator_sim \
         main

led_SRC                  := test_led.c
motors_hw_SRC            := test_motors.c
motors_sim_SRC           := test_motors.c
motors_sim_FLAGS         := $(SIM)
state_SRC                := test_state.c
parameters_SRC           := test_parameters.c
bmp180_SRC               := test_bmp180.c
mpu6050_SRC              := test_mpu6050.c
quaternions_SRC          := test_quaternions.c
height_hw_SRC            := test_height.c
height_hw_EXTRA          := $(STATE) $(BMP)
height_sim_SRC           := test_height.c
height_sim_EXTRA         := $(STATE) $(BMP)
height_sim_FLAGS         := $(SIM)
imu_hw_SRC               := test_imu.c
imu_hw_EXTRA             := $(STATE) $(MPU)
imu_sim_SRC              := test_imu.c
imu_sim_EXTRA            := $(STATE) $(MPU)
imu_sim_FLAGS            := $(SIM)
attitude_controller_SRC  := test_attitude_controller.c
attitude_controller_EXTRA:= $(STATE)
tuning_SRC               := test_tuning.c
odometry_SRC             := test_odometry.c
odometry_EXTRA           := $(STATE)
system_hw_SRC            := test_system.c
system_hw_EXTRA          := $(STATE)
system_sim_SRC           := test_system.c
system_sim_EXTRA         := $(STATE)
system_sim_FLAGS         := $(SIM)
ros_coordinator_hw_SRC   := test_ros_coordinator.c
ros_coordinator_hw_EXTRA := $(STATE) $(PARAMS) $(BMP)
ros_coordinator_sim_SRC  := test_ros_coordinator.c
ros_coordinator_sim_EXTRA:= $(STATE) $(PARAMS) $(BMP)
ros_coordinator_sim_FLAGS:= $(SIM)
main_SRC                 := test_main.c

ifdef MODULE_SRC
MODFLAG := -DMODULE_SRC='"$(abspath $(MODULE_SRC))"'
endif

# Rebuild when any test, mock or production file changes.
DEPS := $(wildcard *.c *.h mocks/*.c mocks/*.h mocks/*/*.h mocks/*/*/*.h) \
        $(wildcard $(COMP)/*/*.c $(COMP)/*/*/*.c $(COMP)/*/include/*.h $(COMP)/*/*/include/*.h) \
        ../main/main.c Makefile

$(BUILD)/%: $(DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $($*_FLAGS) $(MODFLAG) $($*_SRC) $($*_EXTRA) $(COMMON) -o $@ $(LDLIBS)

# --------------------------------------------------------------------------
test:
	@mkdir -p $(BUILD); \
	total=0; failed=0; broken=0; tests=0; fails=0; ignored=0; \
	printf "\n%-24s %-8s %s\n" "TEST" "RESULT" "DETAILS"; \
	printf "%-24s %-8s %s\n"   "----" "------" "-------"; \
	for t in $(TESTS); do \
	  total=$$((total+1)); \
	  if ! $(MAKE) -s --no-print-directory $(BUILD)/$$t > $(BUILD)/$$t.build.log 2>&1; then \
	    broken=$$((broken+1)); \
	    err=$$(grep -m1 -E 'error:' $(BUILD)/$$t.build.log | sed 's|.*/||'); \
	    printf "%-24s %-8s %s\n" "$$t" "NO BUILD" "$$err"; \
	    continue; \
	  fi; \
	  ./$(BUILD)/$$t > $(BUILD)/$$t.log 2>&1; rc=$$?; \
	  line=$$(grep -E '^[0-9]+ Tests [0-9]+ Failures [0-9]+ Ignored' $(BUILD)/$$t.log); \
	  if [ -z "$$line" ]; then \
	    failed=$$((failed+1)); printf "%-24s %-8s %s\n" "$$t" "CRASH" "exit code $$rc, see $(BUILD)/$$t.log"; continue; \
	  fi; \
	  n=$$(echo $$line | awk '{print $$1}'); f=$$(echo $$line | awk '{print $$3}'); i=$$(echo $$line | awk '{print $$5}'); \
	  tests=$$((tests+n)); fails=$$((fails+f)); ignored=$$((ignored+i)); \
	  if [ "$$f" -eq 0 ]; then res=PASS; else res=FAIL; failed=$$((failed+1)); fi; \
	  printf "%-24s %-8s %s\n" "$$t" "$$res" "$$line"; \
	done; \
	echo; \
	echo "Suites: $$total  (failed: $$failed, not built: $$broken)"; \
	echo "Tests:  $$tests  (failures: $$fails, ignored: $$ignored)"; \
	echo "Details of a suite:  make run T=<name>   (logs in $(BUILD)/)"; \
	[ $$failed -eq 0 ] && [ $$broken -eq 0 ]

run:
	@test -n "$(T)" || (echo "Use: make run T=<test>   (make list)"; exit 1)
	@rm -f $(BUILD)/$(T)
	@$(MAKE) -s --no-print-directory $(BUILD)/$(T)
	./$(BUILD)/$(T)

list:
	@for t in $(TESTS); do echo $$t; done

clean:
	rm -rf $(BUILD)

.PHONY: test run list clean
