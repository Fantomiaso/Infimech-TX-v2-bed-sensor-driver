# Piezo Bed Sensor - Klipper Extras Module (Release v1.0.0)
# Copyright (C) 2024-2026 Arlou Dzmitry
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# MODULE DESCRIPTION:
#   Module implements automatic calibration and validation of piezoelectric
#   bed sensor via auxiliary STM32G030 microcontroller (I2C address 0x40).
#   This sensor is the original component from Infimech TX v2 / Flyingbear S1.
#
#   Operating principle:
#   1. STM32G030 measures bed vibration through 4 ADC channels (12-bit, 0-4095)
#   2. Thresholds determine at what signal level to register touch
#   3. When any channel exceeds threshold — output pin (!PF4) goes active
#   4. Module automatically finds optimal thresholds using binary search
#   5. Vibration uses Klipper homing API (drip_move) for MCU synchronization

import logging
import struct

# I2C communication constants
I2C_ADDR = 0x40       # Fixed STM32G030 address on I2C bus
BYTES_TO_READ = 16    # 8 values × 2 bytes (uint16 Big-Endian)

class PiezoBedSensor:
    def __init__(self, config):
        # Core Klipper object references
        self.printer = config.get_printer()      # Printer object for accessing other modules
        self.name = config.get_name()            # Configuration section name
        self.reactor = self.printer.get_reactor()  # Event reactor for async timing operations
        
        # These will be initialized later in _handle_ready() to avoid accessing
        # modules that aren't loaded yet during initial config parsing
        self.toolhead = None       # ToolHead object for carriage movement control
        self.gcode = None          # GCode object for command registration
        self.mcu_endstop = None    # Hardware endstop from probe (monitors PF4 pin)

        # Initialize I2C communication with STM32G030 sensor board
        # MCU_I2C_from_config sets up the bus according to piezo_sensor.cfg parameters
        self.i2c_address = config.getint('i2c_address', I2C_ADDR)
        from . import bus
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=self.i2c_address)

        # Safe calibration position - where head moves before vibration tests
        # Should be bed center or a stable point away from edges
        self.cal_safe_x = config.getfloat('calibration_safe_x', 111.)
        self.cal_safe_y = config.getfloat('calibration_safe_y', 112.)
        self.cal_safe_z = config.getfloat('calibration_safe_z', 50.)
        
        # Vibration generation parameters
        # High acceleration (20000 mm/s²) creates strong mechanical vibrations
        # that the piezo sensor can detect, simulating nozzle touch conditions
        self.cal_vib_accel = config.getfloat('calibration_vibration_accel', 20000.)
        self.cal_vib_freq = config.getfloat('calibration_vibration_freq', 20.)
        self.cal_block_duration = config.getfloat('calibration_block_duration', 1.0)

        # Calibration strategy parameters
        # margin: safety buffer added to found threshold (prevents false triggers)
        # step_min: minimum binary search resolution (ADC counts)
        # baseline_samples: how many readings to average for noise floor
        self.cal_margin = config.getint('calibration_margin', 100)
        self.cal_step_min = config.getint('calibration_min_step', 20)
        self.cal_baseline_samples = config.getint('calibration_baseline_samples', 100)
        
        # Axis order: XY first is recommended as it matches real touch conditions
        self.shake_xy_first = config.getboolean('shake_XY_first', True)
        self.perform_final_test = config.getboolean('perform_final_test', True)
        
        # Validation timing and sensitivity
        self.validation_duration = config.getfloat('validation_duration', 3.0)
        self.validation_test_margin = config.getint('validation_test_margin', 20)
        
        # Target bed temperature during calibration (thermal expansion affects sensitivity)
        self.calibration_bed_temp = config.getint('calibration_bed_temp', 60)

        # Load default thresholds from config if specified
        # These are written to STM32 RAM immediately if present
        self.default_thresholds = None
        default_str = config.get('default_thresholds', None)
        if default_str:
            try:
                self.default_thresholds = [int(x.strip()) for x in default_str.split(',')]
                if len(self.default_thresholds) != 4:
                    raise ValueError("Need 4 values")
            except Exception as e:
                logging.warning("Invalid default_thresholds: %s. Ignoring.", str(e))
                self.default_thresholds = None

        # Initialize empty data arrays
        self.thresholds = []       # Current trigger thresholds (4 channels)
        self.sensor_values = []    # Current ADC readings (4 channels)
        self.calibrated = False    # Whether successful calibration has occurred

        # If defaults provided, write them to sensor's volatile RAM immediately
        if self.default_thresholds:
            self._write_thresholds(self.default_thresholds)

        # Defer full initialization until all Klipper modules are loaded
        # This avoids race conditions during startup
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        logging.info("PiezoBedSensor v1.0.0 initialized")

    def _handle_ready(self):
        """Called when Klipper finishes loading all modules.
        
        Here we:
        1. Obtain references to toolhead and gcode (guaranteed to exist now)
        2. Get mcu_endstop from probe module for hardware trigger monitoring
        3. Register our custom G-code commands with Klipper
        """
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode = self.printer.lookup_object('gcode')
        
        # Connect to probe's MCU endstop for hardware trigger detection
        # The probe module already configures PF4 pin; we reuse that setup
        try:
            probe = self.printer.lookup_object('probe')
            self.mcu_endstop = probe.mcu_probe.mcu_endstop
        except Exception:
            raise self.printer.config_error(
                "[piezo_bed_sensor] requires [probe] section in printer.cfg")
        
        # Register G-code commands that users can execute
        self.gcode.register_command('READ_BED_SENSOR', self.cmd_read_bed_sensor)
        self.gcode.register_command('SET_BED_SENSOR_THRESHOLD', self.cmd_set_bed_sensor_threshold)
        self.gcode.register_command('CALIBRATE_BED_SENSOR', self.cmd_calibrate_bed_sensor)
        self.gcode.register_command('VALIDATE_BED_SENSOR', self.cmd_validate_bed_sensor)
        logging.info("PiezoBedSensor v1.0.0 ready")

    def _read_sensor(self):
        """Read current ADC values and thresholds from STM32G030 via I2C.
        
        Data format (16 bytes total):
          Bytes 0-7:   Current ADC values for channels 0-3 (4 × uint16 Big-Endian)
          Bytes 8-15:  Current thresholds for channels 0-3 (4 × uint16 Big-Endian)
        
        Returns True on successful read, False on error.
        """
        try:
            # Read 16 bytes from I2C without sending a write preamble first
            params = self.i2c.i2c_read([], BYTES_TO_READ)
            if params is None or 'response' not in params:
                return False
            raw_bytes = bytes(params['response'])
            if len(raw_bytes) < BYTES_TO_READ:
                return False

            # Unpack 8 unsigned 16-bit integers in Big-Endian format
            all_values = struct.unpack('>8H', raw_bytes)
            self.thresholds = list(all_values[0:4])      # First 4 values = thresholds
            self.sensor_values = list(all_values[4:8])   # Last 4 values = ADC readings
            return True
        except Exception:
            return False

    def _write_thresholds(self, thresholds):
        """Write new threshold values to STM32G030 via I2C.
        
        Packet format (9 bytes):
          Byte 0:     0x00 (write command identifier for STM32 firmware)
          Bytes 1-2:  Threshold for channel 0 (uint16 Big-Endian)
          Bytes 3-4:  Threshold for channel 1
          Bytes 5-6:  Threshold for channel 2
          Bytes 7-8:  Threshold for channel 3
        
        Note: Thresholds are stored in STM32's volatile RAM and will be lost
        on power cycle. Save them in piezo_sensor.cfg for persistence.
        
        Returns True on success, False on error.
        """
        try:
            target_thresholds = list(thresholds[0:4])
            # Pack 4 uint16 values into 8 bytes (Big-Endian)
            data_bytes = struct.pack('>4H', *target_thresholds)
            # Prepend 0x00 command byte required by STM32 firmware
            packet = [0x00] + list(data_bytes)
            self.i2c.i2c_write(packet)
            self.thresholds = target_thresholds
            # Brief pause ensures STM32 completes flash/RAM write
            self.reactor.pause(self.reactor.monotonic() + 0.002)
            return True
        except Exception:
            return False

    def _run_vibration(self, axes, duration):
        """Generate controlled mechanical vibration and check for sensor trigger.
        
        Uses Klipper's drip_move API (standard homing mechanism) to ensure
        perfect synchronization between movement and endstop monitoring.
        
        Parameters:
            axes: 'xy' for horizontal vibration, 'z' for vertical vibration
            duration: How long to vibrate in seconds
            
        Returns:
            (triggered, count): triggered is bool, count is 0 or 1
        """
        if self.mcu_endstop is None:
            raise self.printer.command_error(
                "mcu_endstop not available. Is [probe] configured?")
        
        # Save current state so we can restore it after vibration
        saved_accel = self.toolhead.max_accel
        saved_pos = self.toolhead.get_position()
        
        # Temporarily increase acceleration for aggressive shaking
        self.toolhead.set_max_velocities(None, self.cal_vib_accel, None, None)
        
        # Calculate number of shake cycles based on duration
        num_cycles = max(1, int(duration * 8))
        triggered = False
        pos = list(saved_pos)
        
        if axes == 'xy':
            # XY vibration: rapid ±4mm jerks at 250mm/s
            # At 20000 mm/s² acceleration, head reaches 250mm/s within 4mm
            speed = 250.0
            for _ in range(num_cycles):
                # Jerk in positive X/Y direction
                target = [pos[0] + 4.0, pos[1] + 4.0, pos[2]]
                print_time = self.toolhead.get_last_move_time()
                # Start hardware endstop monitoring on MCU
                completion = self.mcu_endstop.home_start(
                    print_time, 0.000015, 4, 0.001)
                self.toolhead.dwell(0.001)  # Sync timing
                # Execute movement; will abort early if trigger detected
                self.toolhead.drip_move(target, speed, completion)
                # Wait for trigger or movement completion
                trigger_time = self.mcu_endstop.home_wait(
                    self.toolhead.get_last_move_time())
                if trigger_time > 0.0:
                    triggered = True
                    break
                pos = self.toolhead.get_position()
                
                # Jerk in negative X/Y direction
                target = [pos[0] - 4.0, pos[1] - 4.0, pos[2]]
                print_time = self.toolhead.get_last_move_time()
                completion = self.mcu_endstop.home_start(
                    print_time, 0.000015, 4, 0.001)
                self.toolhead.dwell(0.001)
                self.toolhead.drip_move(target, speed, completion)
                trigger_time = self.mcu_endstop.home_wait(
                    self.toolhead.get_last_move_time())
                if trigger_time > 0.0:
                    triggered = True
                    break
                pos = self.toolhead.get_position()
        else:
            # Z vibration: ±2mm vertical movement at 50mm/s
            z_cycles = max(1, int(duration * 4))
            z_speed = 50.0
            for _ in range(z_cycles):
                # Move up
                target = [pos[0], pos[1], pos[2] + 2.0]
                print_time = self.toolhead.get_last_move_time()
                completion = self.mcu_endstop.home_start(
                    print_time, 0.000015, 4, 0.001)
                self.toolhead.dwell(0.001)
                self.toolhead.drip_move(target, z_speed, completion)
                trigger_time = self.mcu_endstop.home_wait(
                    self.toolhead.get_last_move_time())
                if trigger_time > 0.0:
                    triggered = True
                    break
                pos = self.toolhead.get_position()
                
                # Move down
                target = [pos[0], pos[1], pos[2] - 2.0]
                print_time = self.toolhead.get_last_move_time()
                completion = self.mcu_endstop.home_start(
                    print_time, 0.000015, 4, 0.001)
                self.toolhead.dwell(0.001)
                self.toolhead.drip_move(target, z_speed, completion)
                trigger_time = self.mcu_endstop.home_wait(
                    self.toolhead.get_last_move_time())
                if trigger_time > 0.0:
                    triggered = True
                    break
                pos = self.toolhead.get_position()
        
        # Return to starting position
        self.toolhead.manual_move(saved_pos, 100)
        self.toolhead.wait_moves()
        
        # Restore original acceleration
        self.toolhead.set_max_velocities(None, saved_accel, None, None)
        self.toolhead.wait_moves()
        
        # Pause for mechanical vibrations to settle
        self.reactor.pause(self.reactor.monotonic() + 0.05)
        
        return triggered, 1 if triggered else 0

    def _test_channel(self, channel, value, axes, duration):
        """Test a single channel with a specific threshold value.
        
        Sets the test channel to 'value', disables others (4095).
        If already calibrated, preserves working thresholds for other channels.
        Then runs vibration and reports if trigger occurred.
        
        Returns (triggered, count) or (None, 0) on communication error.
        """
        test_thresholds = [4095] * 4  # Disable all channels initially
        test_thresholds[channel] = value  # Set test threshold for target channel
        
        # Preserve existing calibrated thresholds for non-test channels
        if self.calibrated and len(self.thresholds) == 4:
            for i in range(4):
                if i != channel and self.thresholds[i] > 0 and self.thresholds[i] < 4095:
                    test_thresholds[i] = self.thresholds[i]
        
        # Log test parameters for debugging
        logging.info("_test_channel: ch=%d, value=%d, axes=%s, writing=%s",
                     channel, value, axes, str(test_thresholds))
                     
        if not self._write_thresholds(test_thresholds):
            return None, 0
        return self._run_vibration(axes, duration)

    def _find_stable(self, channel, start, low, axes, gcmd):
        """Binary search to find the threshold where sensor stops triggering.
        
        Algorithm:
        1. Exponential phase: double threshold until sensor stops triggering
        2. Binary phase: narrow down to precise boundary (within cal_step_min)
        
        Parameters:
            channel: Which ADC channel to test (0-3)
            start: Initial threshold to test (usually baseline × 2)
            low: Lower bound (baseline value)
            axes: Which axes to vibrate ('xy' or 'z')
            gcmd: GCode command object for user feedback
            
        Returns: Stable threshold value, or None if search fails.
        """
        gcmd.respond_info(" Ch%d: start=%d, low=%d" % (channel, start, low))
        test = start
        high = None
        step = 0

        # Phase 1: Exponential search to find upper bound
        while True:
            step += 1
            triggered, count = self._test_channel(channel, test, axes, self.cal_block_duration)
            gcmd.respond_info("  Step%d: test=%d, triggered=%s, count=%d" % (step, test, triggered, count))
            if triggered is None:
                gcmd.respond_info("  Step%d: ERROR test returned None" % step)
                return None
            if not triggered:
                high = test  # Found upper boundary
                break
            low = test
            test = test * 2
            if test > 4095:
                # Reached ADC maximum - test once more at 4095
                test = 4095
                step += 1
                triggered, count = self._test_channel(channel, test, axes, self.cal_block_duration)
                gcmd.respond_info("  Step%d: test=%d, triggered=%s, count=%d" % (step, test, triggered, count))
                if triggered is None:
                    gcmd.respond_info("  Step%d: ERROR test returned None" % step)
                    return None
                if triggered:
                    # Even max value triggers - indicates hardware problem
                    gcmd.respond_info(" ERROR: Ch%d even 4095 triggers!" % channel)
                    return None
                high = test
                break

        # Phase 2: Binary search between low and high
        while (high - low) > self.cal_step_min:
            step += 1
            mid = (low + high) // 2
            triggered, count = self._test_channel(channel, mid, axes, self.cal_block_duration)
            gcmd.respond_info("  Step%d: mid=%d, triggered=%s, count=%d" % (step, mid, triggered, count))
            if triggered is None:
                gcmd.respond_info("  Step%d: ERROR test returned None" % step)
                return None
            if triggered:
                low = mid  # Still triggering - need higher threshold
            else:
                high = mid  # Not triggering - can try lower threshold

        gcmd.respond_info(" Ch%d stable: %d" % (channel, high))
        return high

    def _calibrate_axis(self, axes, prev_thresholds, baseline, gcmd):
        """Calibrate thresholds for one axis (XY or Z).
        
        If prev_thresholds is None, performs full calibration from scratch
        by finding stable threshold for each of 4 channels.
        
        If prev_thresholds is provided, tests them first and only recalibrates
        channels that trigger (adaptive mode).
        
        Returns: List of 4 calibrated thresholds, or None on failure.
        """
        axis_name = "XY" if axes == 'xy' else "Z"
        gcmd.respond_info("\n=== Calibrating %s axis ===" % axis_name)
        gcmd.respond_info(" Baseline: %s, prev_thresholds: %s" % (str(baseline), str(prev_thresholds)))

        if prev_thresholds is None:
            # Full calibration: find stable threshold for each channel
            gcmd.respond_info(" Calibrating each channel from scratch...")
            thresholds = []
            for ch in range(4):
                gcmd.respond_info(" Channel %d..." % ch)
                start = baseline[ch] * 2  # Start at 2× baseline
                low = baseline[ch]         # Lower bound is baseline
                stable = self._find_stable(ch, start, low, axes, gcmd)
                if stable is None:
                    return None
                final = min(stable + self.cal_margin, 4095)
                gcmd.respond_info(" Channel %d final: %d" % (ch, final))
                thresholds.append(final)
            gcmd.respond_info(" Axis %s thresholds: %s" % (axis_name, str(thresholds)))
            return thresholds
        else:
            # Adaptive calibration: test existing thresholds
            gcmd.respond_info(" Testing previous thresholds: %s" % str(prev_thresholds))
            triggered, count = self._test_channel(0, prev_thresholds[0], axes, self.cal_block_duration)
            gcmd.respond_info(" Channel 0 test: prev=%d, triggered=%s, count=%d" % (prev_thresholds[0], triggered, count))
            if triggered is None:
                return None
            if not triggered:
                # Channel 0 stable - assume all channels are stable
                gcmd.respond_info(" Channel 0 stable. Using previous thresholds.")
                return list(prev_thresholds)

            # Channel 0 triggers - test and recalibrate each channel individually
            gcmd.respond_info(" Channel 0 triggers. Testing all channels...")
            z_thresholds = []
            for ch in range(4):
                gcmd.respond_info(" Testing channel %d (prev=%d)..." % (ch, prev_thresholds[ch]))
                triggered, count = self._test_channel(ch, prev_thresholds[ch], axes, self.cal_block_duration)
                gcmd.respond_info(" Channel %d test: prev=%d, triggered=%s, count=%d" % (ch, prev_thresholds[ch], triggered, count))
                if triggered is None:
                    return None
                if not triggered:
                    gcmd.respond_info(" Channel %d: stable" % ch)
                    z_thresholds.append(prev_thresholds[ch])
                else:
                    # Channel needs recalibration
                    gcmd.respond_info(" Channel %d: triggers, adapting..." % ch)
                    new_stable = self._find_stable(ch, prev_thresholds[ch], prev_thresholds[ch], axes, gcmd)
                    if new_stable is None:
                        return None
                    z_thresholds.append(min(new_stable + self.cal_margin, 4095))
            gcmd.respond_info(" Axis %s adapted thresholds: %s" % (axis_name, str(z_thresholds)))
            return z_thresholds

    def cmd_read_bed_sensor(self, gcmd):
        """Handle READ_BED_SENSOR G-code command.
        
        Reads current ADC values and thresholds from sensor and displays them.
        Useful for quick health checks and diagnostics.
        """
        if not self._read_sensor():
            gcmd.respond_info("ERROR: Failed to read sensor")
            return
        msg = "\n=== Piezo Bed Sensor ===\n"
        for i in range(4):
            msg += "Sensor %d: ADC=%d THR=%d\n" % (i, self.sensor_values[i], self.thresholds[i])
        gcmd.respond_info(msg)

    def cmd_set_bed_sensor_threshold(self, gcmd):
        """Handle SET_BED_SENSOR_THRESHOLD G-code command.
        
        Allows manual override of thresholds without running calibration.
        Format: SET_BED_SENSOR_THRESHOLD THRESHOLDS=400,400,400,400
        """
        thr_str = gcmd.get('THRESHOLDS', None)
        if thr_str:
            try:
                thresholds = [int(x.strip()) for x in thr_str.split(',')]
                if len(thresholds) != 4:
                    raise ValueError("Need 4 values")
            except Exception:
                gcmd.respond_info("ERROR: Use THRESHOLDS=400,400,400,400")
                return
        else:
            gcmd.respond_info("Usage: SET_BED_SENSOR_THRESHOLD THRESHOLDS=...")
            return

        if self._write_thresholds(thresholds):
            gcmd.respond_info("OK: Thresholds set")
        else:
            gcmd.respond_info("ERROR: Failed to write")

    def cmd_validate_bed_sensor(self, gcmd):
        """Handle VALIDATE_BED_SENSOR G-code command.
        
        Two-stage validation:
        Stage 1: Tests sensor sensitivity with low thresholds (must trigger)
        Stage 2: Tests for false triggers with normal thresholds (must NOT trigger)
        
        If false triggers detected, automatically starts recalibration.
        Called automatically by START_PRINT macro before each print.
        """
        gcmd.respond_info("Validating piezo sensor...")
        self.toolhead.wait_moves()

        # Phase 1: Measure baseline (average of 10 readings)
        baseline = [0] * 4
        count = 0
        for _ in range(10):
            if self._read_sensor():
                for i in range(4):
                    baseline[i] += self.sensor_values[i]
                count += 1
            self.reactor.pause(self.reactor.monotonic() + 0.02)

        if count == 0:
            raise gcmd.error("ERROR: Cannot read sensor during validation")

        baseline = [b // count for b in baseline]
        max_baseline = max(baseline)
        test_threshold = min(max_baseline + self.validation_test_margin, 4095)
        gcmd.respond_info("Baseline: %s, max=%d, test_threshold=%d" % (str(baseline), max_baseline, test_threshold))

        # Stage 1: Sensitivity check - sensor MUST trigger at low thresholds
        test_thresholds = [test_threshold] * 4
        if not self._write_thresholds(test_thresholds):
            raise gcmd.error("ERROR: Failed to write test thresholds")

        triggered_xy, count_xy = self._run_vibration('xy', self.validation_duration)
        if not triggered_xy:
            raise gcmd.error("ERROR: Sensor does NOT trigger with low thresholds! Check wiring or mechanical issues. Threshold=%d" % test_threshold)

        triggered_z, count_z = self._run_vibration('z', self.validation_duration)
        if not triggered_z:
            raise gcmd.error("ERROR: Z-axis sensor does NOT trigger! Check bed mounting, screws, or run BED_SENSOR_CALIBRATE to update thresholds.")

        gcmd.respond_info("Trigger OK (XY:%d, Z:%d triggers registered via native probe endstop)." % (count_xy, count_z))
        gcmd.respond_info("Stage 2: Testing for false triggers...")

        if not self._read_sensor():
            raise gcmd.error("ERROR: Cannot read sensor during false trigger test")

        # Check if we have any valid stored thresholds
        if (len(self.thresholds) == 4 and all(t == 0 for t in self.thresholds)) or (self.default_thresholds is None and not self.calibrated):
            gcmd.respond_info("No valid thresholds. Starting calibration...")
            self.cmd_calibrate_bed_sensor(gcmd)
            return self.calibrated

        # Stage 2: False trigger check with working thresholds
        normal_thresholds = self.default_thresholds if self.default_thresholds else self.thresholds
        if not self._write_thresholds(normal_thresholds):
            raise gcmd.error("ERROR: Failed to restore normal thresholds")

        triggered_xy, count_xy = self._run_vibration('xy', self.validation_duration)
        triggered_z, count_z = self._run_vibration('z', self.validation_duration)
        if triggered_xy or triggered_z:
            total = count_xy + count_z
            gcmd.respond_info("WARNING: False trigger detected (%d times)! Running full calibration..." % total)
            self.cmd_calibrate_bed_sensor(gcmd)
            return self.calibrated
        else:
            gcmd.respond_info("Validation passed. Sensor OK.")
            return True

    def cmd_calibrate_bed_sensor(self, gcmd):
        """Handle CALIBRATE_BED_SENSOR G-code command.
        
        Full calibration procedure:
        1. Verify all axes are homed
        2. Move to safe calibration position
        3. Measure baseline (mechanical noise floor)
        4. Calibrate first axis (default: XY)
        5. Calibrate second axis (default: Z)
        6. Optionally verify new thresholds
        7. Save thresholds to sensor's volatile RAM
        
        Note: Thresholds must be manually saved to piezo_sensor.cfg for persistence!
        """
        gcmd.respond_info("\n========================================\n Piezo Bed Sensor Calibration v1.0.0\n========================================")
        gcmd.respond_info("Current thresholds before calibration: %s" % str(self.thresholds))
        
        # Verify homing is complete
        kin = self.toolhead.get_kinematics().get_status(None)
        for axis in ['x', 'y', 'z']:
            if axis not in kin['homed_axes'].lower():
                gcmd.respond_info("ERROR: %s not homed" % axis.upper())
                return

        # Move to safe position for calibration
        self.toolhead.manual_move([self.cal_safe_x, self.cal_safe_y, self.cal_safe_z], 100)
        self.toolhead.wait_moves()

        # Measure baseline - average of many readings to determine noise floor
        gcmd.respond_info("Reading baseline (%d samples)..." % self.cal_baseline_samples)
        baseline = [0] * 4
        count = 0
        for _ in range(self.cal_baseline_samples):
            if self._read_sensor():
                for i in range(4):
                    baseline[i] += self.sensor_values[i]
                count += 1
            self.reactor.pause(self.reactor.monotonic() + 0.02)

        if count == 0:
            gcmd.respond_info("ERROR: Failed to read baseline")
            return

        baseline = [b // count for b in baseline]
        gcmd.respond_info("Baseline: %s" % str(baseline))

        # Calibrate axes in configured order
        first_axis, second_axis = ('xy', 'z') if self.shake_xy_first else ('z', 'xy')
        first_thresholds = self._calibrate_axis(first_axis, None, baseline, gcmd)
        if first_thresholds is None:
            gcmd.respond_info("ERROR: First axis calibration failed")
            return

        second_thresholds = self._calibrate_axis(second_axis, first_thresholds, baseline, gcmd)
        if second_thresholds is None:
            gcmd.respond_info("ERROR: Second axis calibration failed")
            return

        # Save final thresholds to sensor RAM
        gcmd.respond_info("\nFinal thresholds: %s" % str(second_thresholds))
        if not self._write_thresholds(second_thresholds):
            gcmd.respond_info("ERROR: Failed to write")
            return

        # Optional verification with new thresholds
        if self.perform_final_test:
            gcmd.respond_info("Verifying...")
            for name, code in [("First", first_axis), ("Second", second_axis)]:
                triggered, count = self._run_vibration(code, 1.0)
                gcmd.respond_info(" %s: %s" % (name, "WARNING: triggers (%dx)" % count if triggered else "OK"))

        self.calibrated = True
        gcmd.respond_info("\n=== Calibration Complete ===\nFinal: %s" % str(second_thresholds))

    def get_status(self, eventtime):
        """Provide sensor status for external services (Moonaker/Fluidd).
        
        Called automatically by Klipper's status polling mechanism.
        Includes protection against empty arrays during startup.
        
        Returns dict with:
            sensor_values: list of 4 ADC readings
            thresholds: list of 4 threshold values  
            calibrated: bool indicating calibration state
            endstop_triggered: bool (True if any channel exceeds threshold)
        """
        if len(self.sensor_values) != 4 or len(self.thresholds) != 4:
            # Data not available yet - return safe defaults
            return {
                'sensor_values': [],
                'thresholds': [],
                'calibrated': self.calibrated,
                'endstop_triggered': False
            }
        return {
            'sensor_values': self.sensor_values,
            'calibrated': self.calibrated,
            'thresholds': self.thresholds,
            'endstop_triggered': any(self.sensor_values[i] > self.thresholds[i] for i in range(4))
        }

# Klipper module entry point - called when configuration is loaded
def load_config(config):
    return PiezoBedSensor(config)
