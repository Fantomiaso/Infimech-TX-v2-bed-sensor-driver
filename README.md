# Piezo Bed Sensor v1.0.0

## Description

Automated calibration and validation module for piezoelectric bed sensor in Klipper.

Bed sensor is the original component from **Infimech TX v2** printer (also known as **Flyingbear S1 new revision**).

## Status

Implementation tested and functional in current form (v1.0.0).

## Hardware

Developed and tested on the following configuration:

- **Board:** BIGTREETECH Manta M8P V2.0 (STM32H723)
- **Host:** CB2 (single-board computer running Klipper)
- **Drivers:** TMC2209 in UART mode
- **Kinematics:** CoreXY
- **Printer:** Infimech TX v2 / Flyingbear S1 (new revision)
- **Bed Sensor:** Piezoelectric based on STM32G030

### Sensor Connection

This version requires connecting the bed sensor to the **MCU** I2C bus (microcontroller board), not the host I2C (CB2).

**I2C Bus (Required):**
- **SCL:** PA8 — clock signal
- **SDA:** PC9 — data
- **I2C Address:** 0x40
- **Speed:** 400 kHz

**PROBE (Endstop):**
- **Pin:** PF4 (Endstop1 on M8P v2)
- **Configuration:** `pin: !PF4` (inverted signal)

**Power (IMPORTANT):**

The sensor board has a specific power scheme. Two options available (only one at a time!):
1. Via I2C connector: 3.3V voltage
2. Via PROBE connector: 5V voltage

On the Manta M8P v2.0 board, it is recommended to power the sensor with **5V through the PROBE cable**.

### Compatibility

The provided configs (`printer.cfg`, `piezo_sensor.cfg`) contain settings for the Manta M8P v2.0 board and are provided **for reference only**.

If you have a different board:
- Check the I2C pinout on your board (may differ from PA8/PC9)
- Check the endstop pin for sensor connection (may differ from PF4)
- Adapt the configuration for your board
- I2C and/or PROBE connector remapping may be required. Compare pinouts between your old board and new board.

## File Installation

### Required Files

1. `piezo_bed_sensor.py` — main Python module for Klipper
2. `piezo_sensor.cfg` — sensor configuration file

### File Locations

#### piezo_bed_sensor.py

**Target path:** `~/klipper/klippy/extras/piezo_bed_sensor.py`

**Method 1: Via Web Interface + SSH (Recommended)**
1. Upload `piezo_bed_sensor.py` through web interface (Mainsail/Fluidd) to config folder
2. Connect via SSH (PuTTY)
3. Copy file:
   ```bash
   cp ~/printer_data/config/piezo_bed_sensor.py ~/klipper/klippy/extras/
   ```

**Method 2: Via SCP**
```bash
scp piezo_bed_sensor.py pi@printer_ip:~/klipper/klippy/extras/
```

**Method 3: Via Midnight Commander (MC)**
1. Connect via SSH
2. Launch `mc`
3. Copy file to `~/klipper/klippy/extras/`

#### piezo_sensor.cfg

**Target path:** `~/printer_data/config/piezo_sensor.cfg`

Upload via web interface to config folder.

#### printer.cfg

Add to `printer.cfg`:
```ini
[include piezo_sensor.cfg]
```

Ensure `[probe]` section exists:
```ini
[probe]
pin: !PF4
z_offset: 0
speed: 5.0
samples: 3
sample_retract_dist: 2.0
lift_speed: 5.0
```

**Important:** pin `!PF4` is for Manta M8P v2.0. Use your pin for other boards.

### Restart

After placing all files:
```
RESTART
```

Verification:
```
READ_BED_SENSOR
```

## Sensor Operating Principle

The bed sensor consists of a board with STM32G030 microcontroller which:
1. Measures bed vibration through 4 ADC channels (12-bit, range 0–4095)
2. Compares current values with thresholds
3. When any channel exceeds threshold — activates output pin (PF4)

Thresholds are stored in **STM32G030 RAM** (volatile memory). On full power-off, thresholds reset.

### I2C Communication Format

**Read (16 bytes):**
```
I2C Read (address 0x40, 16 bytes)
```

Response contains 8 uint16 values (Big-Endian):
- `[0-3]` — Current ADC values (channels 0–3)
- `[4-7]` — Current thresholds (channels 0–3)

Example response:
```
Response (hex): 00 A1 00 A0 00 9F 00 A2 01 19 01 18 01 15 01 19
Unpacked:
  ADC: [161, 160, 159, 162]
  THR: [281, 280, 277, 281]
```

**Write thresholds (9 bytes):**
```
Byte 0: 0x00 ("write thresholds" command)
Bytes 1-2: Channel 0 threshold (uint16, Big-Endian)
Bytes 3-4: Channel 1 threshold
Bytes 5-6: Channel 2 threshold
Bytes 7-8: Channel 3 threshold
```

Example writing thresholds `[281, 280, 277, 281]`:
```
Request: 00 01 19 01 18 01 15 01 19
```

## Commands

### READ_BED_SENSOR

Read current values from sensor.
```
READ_BED_SENSOR
```
Outputs ADC values and thresholds for 4 channels.

### SET_BED_SENSOR_THRESHOLD

Manually set thresholds.
```
SET_BED_SENSOR_THRESHOLD THRESHOLDS=220,220,220,220
```

### VALIDATE_BED_SENSOR

Validate sensor functionality before printing.

**Stages:**
1. **Read baseline** — 10 measurements of mechanical noise
2. **Sensitivity check (Stage 1)** — shake with low thresholds. Sensor must trigger.
3. **False trigger check (Stage 2)** — shake with working thresholds. Sensor must NOT trigger.

On false triggers, automatic calibration starts.

### CALIBRATE_BED_SENSOR

Full sensor calibration.

**Process:**
1. Move to safe position (X110 Y110 Z50)
2. Read baseline (100 noise measurements)
3. **First axis calibration** (default XY):
   - For each channel, binary search for stable threshold
   - `_find_stable` finds threshold where sensor does not trigger from vibration
   - Add `calibration_margin` (100 ADC) to threshold
4. **Second axis calibration** (default Z):
   - Check thresholds from first axis
   - Adaptive search if needed for each channel
5. Final verification (if enabled)
6. Save thresholds to STM32 volatile RAM

**Recommended order:** XY first, then Z (`shake_XY_first: True`). Reverse order (Z→XY) was tested less thoroughly.

**Saving thresholds:** After calibration, copy final values from console to `default_thresholds` in `piezo_sensor.cfg`, then perform `RESTART`.

## Configuration

See `piezo_sensor.cfg` file with detailed comments for each parameter.

Main parameters:
- `i2c_bus` — I2C bus (for M8P v2: `i2c3_PA8_PC9`)
- `calibration_safe_x/y/z` — safe position for calibration
- `calibration_vibration_accel` — vibration acceleration (20000 mm/s²)
- `calibration_margin` — threshold margin (100 ADC)
- `shake_XY_first` — calibration order (True = XY first)
- `default_thresholds` — working thresholds (4 numbers separated by commas)
- `validation_duration` — validation duration (1.0 sec)

## Possible Issues

### Sensor Not Reading
- Check I2C connection (SDA, SCL, GND, VCC)
- Ensure sensor is connected to MCU bus, not host
- Check address: must be 0x40
- Check power: only one source (3.3V or 5V)

### VALIDATE_BED_SENSOR: "Sensor does NOT trigger"
- Check bed mounting screw tightness
- Ensure bed is not "floating" on springs
- Check PF4 signal wire connection

### False Triggers
- Outdated thresholds — run calibration
- Mechanical interference — increase `calibration_margin`

### Error "Second axis calibration failed"
- Set `shake_XY_first: True`
- Check sensor wiring

### Thresholds Reset After Restart
- STM32G030 stores thresholds in volatile RAM
- For permanent storage, specify them in `default_thresholds` in `piezo_sensor.cfg`
- Or set manually: `SET_BED_SENSOR_THRESHOLD THRESHOLDS=...`

## License

GNU GPLv3
