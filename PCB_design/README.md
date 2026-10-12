# PCB design

Here you can find the design of the drone PCB, made with KiCad.

## [Plug mini PCB (GY-87)](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/PCB_design/ESP_plug_mini_PCB_gy87/drone)

The PCB is also the frame of the drone. The boards are not soldered on it, they are plugged in:

- **ESP32-S2 mini** (Wemos S2 Mini), on two 2x8 pin sockets.
- **GY-87** sensor board, on a 1x8 header. The firmware uses its MPU-6050 (accelerometer + gyroscope) and its BMP180 (barometer).

The board also has:

- 4 MOSFET drivers (AO3400A) for the motors.
- 3.3 V regulator (AP2112K-3.3).
- JST-PH connector for the battery.
- Status LEDs and a push button.

### Files

| File | What it is |
|------|------------|
| `drone.kicad_pro`, `drone.kicad_sch`, `drone.kicad_pcb` | KiCad project, schematic and PCB. |
| `*.pretty/`, `fp-lib-table` | Custom footprints used by the PCB. |
| `production/` | Files to manufacture it: Gerbers (`production/drone/`), `BOM.csv`, pick and place (`drone-all.pos`) and the 3D model (`drone.step`). |

The 3D models of the ESP32-S2 mini and the GY-87 are in [`extra_step_files/`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/PCB_design/extra_step_files).
