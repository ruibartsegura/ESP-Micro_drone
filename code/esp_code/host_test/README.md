# Host unit tests

Unit tests for every module of the firmware. They run on the computer with GCC and [Unity](https://github.com/ThrowTheSwitch/Unity): you do not need the ESP32, the sensors, ESP-IDF or ROS.

## Run the tests

```bash
cd esp_code/host_test
make test                  # build and run everything, and print a summary
make run T=motors_sim      # run one suite and see every test
make list                  # list the suites
make test SAN=1            # with AddressSanitizer + UBSan (finds memory errors)
make clean
```

Modules with code for the simulation are built twice: `*_hw` (real drone) and `*_sim` (with `CONFIG_SIMULATION_ON`).

The summary shows one line per suite:

| Result     | Meaning |
|------------|---------|
| `PASS`     | every test passed |
| `FAIL`     | some tests failed (see `make run T=<suite>` or `build/<suite>.log`) |
| `NO BUILD` | the module does not compile in that mode (see `build/<suite>.build.log`) |
| `CRASH`    | the test program stopped before the end |

**Tests marked `[BUG]`** in their comment fail with the current code on purpose: each one documents a real defect and explains it. When you fix the bug, the test passes. Tests with `TEST_IGNORE_MESSAGE` are for features that are not implemented yet (TODO).

## How it works

- Each test file **includes the `.c` file of the module** (`#include MODULE_SRC`), so the tests can also use its static functions and variables.
- `mocks/` replaces ESP-IDF, FreeRTOS and micro-ROS. The mocks **record every call**, so a test can check what the module did:
  - `freertos/`: tasks (`mock_find_task()`), delays, critical sections, queues. `mock_run_task(fn, arg, N)` runs a task loop and stops it after N delays.
  - `driver/gpio.h`, `driver/ledc.h`: pin levels, PWM duties.
  - `driver/i2c.h`: a fake bus with a register map for each device (`mock_i2c_regs[addr][reg]`). Hooks can simulate a chip (see the fake BMP180 in `test_bmp180.c`), and `mock_i2c_fail_at` makes a transaction fail.
  - `rcl/`, `rclc/`: every publisher, subscriber, service, timer and executor handle is saved with its name, type and QoS. `mock_rcl_fail_at` makes an init call fail.
  - `sdkconfig.h`: a copy of the real menuconfig values. **Update it if you change menuconfig.**
- `test_helpers.h`: `TEST_ASSERT_NO_CRASH()` turns a crash (NULL pointer, division by zero) into a normal failure, so the other tests still run.
- Some suites also link real modules (`state.c`, `parameters.c`, `bmp180.c`, `mpu6050.c`); see the `*_EXTRA` variables in the `Makefile`.

To test a different version of a module (for example, a fix you are trying):

```bash
make run T=height_hw MODULE_SRC=/path/to/height.c
```

## Add tests

**New test in an existing suite:** write a `void test_xxx(void)` function in the test file and add `RUN_TEST(test_xxx);` in its `main()`. Call `mock_reset_all()` in `setUp()` (all the files already do it).

**New module:**

1. Create `test_<module>.c`. Copy the header and the structure of another test file (for example `test_led.c`):
   ```c
   #ifndef MODULE_SRC
   #define MODULE_SRC "../components/<path>/<module>.c"
   #endif
   #include MODULE_SRC
   ```
2. Write mocks in the test file for the functions of other modules that it calls.
3. Add the suite to the `Makefile`: put its name in `TESTS`, and add `<name>_SRC`, plus `<name>_EXTRA` (real modules to link) and `<name>_FLAGS := $(SIM)` if it needs them. If it has simulation code, add two suites: `<name>_hw` and `<name>_sim`.
4. If the module uses an ESP-IDF header that is not in `mocks/` yet, add it there, and its implementation in `mocks/mock_esp.c`. Reset its variables in `mock_reset_esp()`.
5. If the module includes a new component, add its `include` folder to `INCS` in the `Makefile`.
