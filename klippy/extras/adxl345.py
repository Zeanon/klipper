# Support for reading acceleration data from an adxl345 chip
#
# Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, time, collections, multiprocessing, os
from . import bus, bulk_sensor

# ADXL345 registers
REG_DEVID = 0x00
REG_BW_RATE = 0x2C
REG_POWER_CTL = 0x2D
REG_DATA_FORMAT = 0x31
REG_FIFO_CTL = 0x38
REG_MOD_READ = 0x80
REG_MOD_MULTI = 0x40

QUERY_RATES = {
    25: 0x8, 50: 0x9, 100: 0xa, 200: 0xb, 400: 0xc,
    800: 0xd, 1600: 0xe, 3200: 0xf,
}

ADXL345_DEV_ID = 0xe5
SET_FIFO_CTL = 0x90

FREEFALL_ACCEL = 9.80665 * 1000.
SCALE_XY = 0.003774 * FREEFALL_ACCEL # 1 / 265 (at 3.3V) mg/LSB
SCALE_Z  = 0.003906 * FREEFALL_ACCEL # 1 / 256 (at 3.3V) mg/LSB

Accel_Measurement = collections.namedtuple(
    'Accel_Measurement', ('time', 'accel_x', 'accel_y', 'accel_z'))

# Helper class to obtain measurements
class AccelQueryHelper:
    def __init__(self, printer):
        self.printer = printer
        self.is_finished = False
        print_time = printer.lookup_object('toolhead').get_last_move_time()
        self.request_start_time = self.request_end_time = print_time
        self.msgs = []
        self.samples = []
    def finish_measurements(self):
        toolhead = self.printer.lookup_object('toolhead')
        self.request_end_time = toolhead.get_last_move_time()
        toolhead.wait_moves()
        self.is_finished = True
    def handle_batch(self, msg):
        if self.is_finished:
            return False
        if len(self.msgs) >= 10000:
            # Avoid filling up memory with too many samples
            return False
        self.msgs.append(msg)
        return True
    def has_valid_samples(self):
        for msg in self.msgs:
            data = msg['data']
            first_sample_time = data[0][0]
            last_sample_time = data[-1][0]
            if (first_sample_time > self.request_end_time
                    or last_sample_time < self.request_start_time):
                continue
            # The time intervals [first_sample_time, last_sample_time]
            # and [request_start_time, request_end_time] have non-zero
            # intersection. It is still theoretically possible that none
            # of the samples from msgs fall into the time interval
            # [request_start_time, request_end_time] if it is too narrow
            # or on very heavy data losses. In practice, that interval
            # is at least 1 second, so this possibility is negligible.
            return True
        return False
    def get_samples(self):
        if not self.msgs:
            return self.samples
        total = sum([len(m['data']) for m in self.msgs])
        count = 0
        self.samples = samples = [None] * total
        for msg in self.msgs:
            for samp_time, x, y, z in msg['data']:
                if samp_time < self.request_start_time:
                    continue
                if samp_time > self.request_end_time:
                    break
                samples[count] = Accel_Measurement(samp_time, x, y, z)
                count += 1
        del samples[count:]
        return self.samples
    def write_to_file(self, filename):
        def write_impl():
            try:
                # Try to re-nice writing process
                os.nice(20)
            except:
                pass
            f = open(filename, "w")
            f.write("#time,accel_x,accel_y,accel_z\n")
            samples = self.samples or self.get_samples()
            for t, accel_x, accel_y, accel_z in samples:
                f.write("%.6f,%.6f,%.6f,%.6f\n" % (
                    t, accel_x, accel_y, accel_z))
            f.close()
        write_proc = multiprocessing.Process(target=write_impl)
        write_proc.daemon = True
        write_proc.start()

# Helper class for G-Code commands
class AccelCommandHelper:
    def __init__(self, config, chip):
        self.printer = config.get_printer()
        self.chip = chip
        self.bg_client = None
        name_parts = config.get_name().split()
        self.base_name = name_parts[0]
        self.name = name_parts[-1]
        self.register_commands(self.name)
        self.disabled_heaters = config.getlist("disable_heaters", [])
        if len(name_parts) == 1:
            if self.name == "adxl345" or not config.has_section("adxl345"):
                self.register_commands(None)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
    def read_accelerometer(self):
        aclient = self.chip.start_internal_client()
        self.printer.lookup_object('toolhead').dwell(1.)
        aclient.finish_measurements()
        values = aclient.get_samples()
        _, accel_x, accel_y, accel_z = values[-1]
    def _handle_ready(self):
        try:
            self.read_accelerometer()
            connected = True
        except Exception as e:
            connected = False
        for heater_name in self.disabled_heaters:
            heater_object = self.printer.lookup_object(heater_name)
            if not hasattr(heater_object, 'heater'):
                raise self.printer.config_error(
                    "'%s' is not a valid heater."
                    % (heater_name,))
            heater = heater_object.heater
            if not hasattr(heater, 'set_enabled'):
                raise self.printer.config_error(
                    "'%s' is not a valid heater."
                    % (heater_name,))
            heater.set_enabled(not connected)
    def register_commands(self, name):
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("ACCELEROMETER_MEASURE", "CHIP", name,
                                   self.cmd_ACCELEROMETER_MEASURE,
                                   desc=self.cmd_ACCELEROMETER_MEASURE_help)
        gcode.register_mux_command("ACCELEROMETER_QUERY", "CHIP", name,
                                   self.cmd_ACCELEROMETER_QUERY,
                                   desc=self.cmd_ACCELEROMETER_QUERY_help)
        gcode.register_mux_command("ACCELEROMETER_DEBUG_READ", "CHIP", name,
                                   self.cmd_ACCELEROMETER_DEBUG_READ,
                                   desc=self.cmd_ACCELEROMETER_DEBUG_READ_help)
        gcode.register_mux_command("ACCELEROMETER_DEBUG_WRITE", "CHIP", name,
                                   self.cmd_ACCELEROMETER_DEBUG_WRITE,
                                   desc=self.cmd_ACCELEROMETER_DEBUG_WRITE_help)
    cmd_ACCELEROMETER_MEASURE_help = "Start/stop accelerometer"
    def cmd_ACCELEROMETER_MEASURE(self, gcmd):
        if self.bg_client is None:
            # Start measurements
            self.bg_client = self.chip.start_internal_client()
            gcmd.respond_info("accelerometer measurements started")
            return
        # End measurements
        name = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        if not name.replace('-', '').replace('_', '').isalnum():
            raise gcmd.error("Invalid NAME parameter")
        bg_client = self.bg_client
        self.bg_client = None
        bg_client.finish_measurements()
        # Write data to file
        if self.base_name == self.name:
            filename = "/tmp/%s-%s.csv" % (self.base_name, name)
        else:
            filename = "/tmp/%s-%s-%s.csv" % (self.base_name, self.name, name)
        bg_client.write_to_file(filename)
        gcmd.respond_info("Writing raw accelerometer data to %s file"
                          % (filename,))
    cmd_ACCELEROMETER_QUERY_help = "Query accelerometer for the current values"
    def cmd_ACCELEROMETER_QUERY(self, gcmd):
        aclient = self.chip.start_internal_client()
        self.printer.lookup_object('toolhead').dwell(1.)
        aclient.finish_measurements()
        values = aclient.get_samples()
        if not values:
            raise gcmd.error("No accelerometer measurements found")
        _, accel_x, accel_y, accel_z = values[-1]
        gcmd.respond_info("accelerometer values (x, y, z): %.6f, %.6f, %.6f"
                          % (accel_x, accel_y, accel_z))
    cmd_ACCELEROMETER_DEBUG_READ_help = "Query register (for debugging)"
    def cmd_ACCELEROMETER_DEBUG_READ(self, gcmd):
        reg = gcmd.get("REG", minval=0, maxval=126, parser=lambda x: int(x, 0))
        val = self.chip.read_reg(reg)
        gcmd.respond_info("Accelerometer REG[0x%x] = 0x%x" % (reg, val))
    cmd_ACCELEROMETER_DEBUG_WRITE_help = "Set register (for debugging)"
    def cmd_ACCELEROMETER_DEBUG_WRITE(self, gcmd):
        reg = gcmd.get("REG", minval=0, maxval=126, parser=lambda x: int(x, 0))
        val = gcmd.get("VAL", minval=0, maxval=255, parser=lambda x: int(x, 0))
        self.chip.set_reg(reg, val)

# Helper to read the axes_map parameter from the config
def read_axes_map(config):
    am = {'x': (0, SCALE_XY), 'y': (1, SCALE_XY), 'z': (2, SCALE_Z),
          '-x': (0, -SCALE_XY), '-y': (1, -SCALE_XY), '-z': (2, -SCALE_Z)}
    axes_map = config.getlist('axes_map', ('x','y','z'), count=3)
    if any([a not in am for a in axes_map]):
        raise config.error("Invalid axes_map parameter")
    return [am[a.strip()] for a in axes_map]

BYTES_PER_SAMPLE = 5
SAMPLES_PER_BLOCK = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE

BATCH_UPDATES = 0.100

# Printer class that controls ADXL345 chip
class ADXL345:
    def __init__(self, config):
        self.printer = config.get_printer()
        AccelCommandHelper(config, self)
        self.axes_map = read_axes_map(config)
        self.data_rate = config.getint('rate', 3200)
        if self.data_rate not in QUERY_RATES:
            raise config.error("Invalid rate parameter: %d" % (self.data_rate,))
        # Setup mcu sensor_adxl345 bulk query code
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=5000000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_adxl345_cmd = None
        mcu.add_config_cmd("config_adxl345 oid=%d spi_oid=%d"
                           % (oid, self.spi.get_oid()))
        mcu.add_config_cmd("query_adxl345 oid=%d rest_ticks=0"
                           % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=oid)
        # Clock tracking
        chip_smooth = self.data_rate * BATCH_UPDATES * 2
        self.clock_sync = bulk_sensor.ClockSyncRegression(mcu, chip_smooth)
        self.clock_updater = bulk_sensor.ChipClockUpdater(self.clock_sync,
                                                          BYTES_PER_SAMPLE)
        self.last_error_count = 0
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._finish_measurements, BATCH_UPDATES)
        self.name = config.get_name().split()[-1]
        hdr = ('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')
        self.batch_bulk.add_mux_endpoint("adxl345/dump_adxl345", "sensor",
                                         self.name, {'header': hdr})
    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.query_adxl345_cmd = self.mcu.lookup_command(
            "query_adxl345 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.clock_updater.setup_query_command(
            self.mcu, "query_adxl345_status oid=%c", oid=self.oid, cq=cmdqueue)
    def read_reg(self, reg):
        params = self.spi.spi_transfer([reg | REG_MOD_READ, 0x00])
        response = bytearray(params['response'])
        return response[1]
    def set_reg(self, reg, val, minclock=0):
        self.spi.spi_send([reg, val & 0xFF], minclock=minclock)
        stored_val = self.read_reg(reg)
        if stored_val != val:
            raise self.printer.command_error(
                    "Failed to set ADXL345 register [0x%x] to 0x%x: got 0x%x. "
                    "This is generally indicative of connection problems "
                    "(e.g. faulty wiring) or a faulty adxl345 chip." % (
                        reg, val, stored_val))
    def start_internal_client(self):
        aqh = AccelQueryHelper(self.printer)
        self.batch_bulk.add_client(aqh.handle_batch)
        return aqh
    # Measurement decoding
    def _extract_samples(self, raw_samples):
        # Load variables to optimize inner loop below
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self.axes_map
        last_sequence = self.clock_updater.get_last_sequence()
        time_base, chip_base, inv_freq = self.clock_sync.get_time_translation()
        # Process every message in raw_samples
        count = seq = 0
        samples = [None] * (len(raw_samples) * SAMPLES_PER_BLOCK)
        for params in raw_samples:
            seq_diff = (params['sequence'] - last_sequence) & 0xffff
            seq_diff -= (seq_diff & 0x8000) << 1
            seq = last_sequence + seq_diff
            d = bytearray(params['data'])
            msg_cdiff = seq * SAMPLES_PER_BLOCK - chip_base
            for i in range(len(d) // BYTES_PER_SAMPLE):
                d_xyz = d[i*BYTES_PER_SAMPLE:(i+1)*BYTES_PER_SAMPLE]
                xlow, ylow, zlow, xzhigh, yzhigh = d_xyz
                if yzhigh & 0x80:
                    self.last_error_count += 1
                    continue
                rx = (xlow | ((xzhigh & 0x1f) << 8)) - ((xzhigh & 0x10) << 9)
                ry = (ylow | ((yzhigh & 0x1f) << 8)) - ((yzhigh & 0x10) << 9)
                rz = ((zlow | ((xzhigh & 0xe0) << 3) | ((yzhigh & 0xe0) << 6))
                      - ((yzhigh & 0x40) << 7))
                raw_xyz = (rx, ry, rz)
                x = round(raw_xyz[x_pos] * x_scale, 6)
                y = round(raw_xyz[y_pos] * y_scale, 6)
                z = round(raw_xyz[z_pos] * z_scale, 6)
                ptime = round(time_base + (msg_cdiff + i) * inv_freq, 6)
                samples[count] = (ptime, x, y, z)
                count += 1
        self.clock_sync.set_last_chip_clock(seq * SAMPLES_PER_BLOCK + i)
        del samples[count:]
        return samples
    # Start, stop, and process message batches
    def _start_measurements(self):
        # In case of miswiring, testing ADXL345 device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        dev_id = self.read_reg(REG_DEVID)
        if dev_id != ADXL345_DEV_ID:
            raise self.printer.command_error(
                "Invalid adxl345 id (got %x vs %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty adxl345 chip."
                % (dev_id, ADXL345_DEV_ID))
        # Setup chip in requested query rate
        self.set_reg(REG_POWER_CTL, 0x00)
        self.set_reg(REG_DATA_FORMAT, 0x0B)
        self.set_reg(REG_FIFO_CTL, 0x00)
        self.set_reg(REG_BW_RATE, QUERY_RATES[self.data_rate])
        self.set_reg(REG_FIFO_CTL, SET_FIFO_CTL)
        # Start bulk reading
        self.bulk_queue.clear_samples()
        rest_ticks = self.mcu.seconds_to_clock(4. / self.data_rate)
        self.query_adxl345_cmd.send([self.oid, rest_ticks])
        self.set_reg(REG_POWER_CTL, 0x08)
        logging.info("ADXL345 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.clock_updater.note_start()
        self.last_error_count = 0
    def _finish_measurements(self):
        # Halt bulk reading
        self.set_reg(REG_POWER_CTL, 0x00)
        self.query_adxl345_cmd.send_wait_ack([self.oid, 0])
        self.bulk_queue.clear_samples()
        logging.info("ADXL345 finished '%s' measurements", self.name)
    def _process_batch(self, eventtime):
        self.clock_updater.update_clock()
        raw_samples = self.bulk_queue.pull_samples()
        if not raw_samples:
            return {}
        samples = self._extract_samples(raw_samples)
        if not samples:
            return {}
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.clock_updater.get_last_overflows()}

def load_config(config):
    return ADXL345(config)

def load_config_prefix(config):
    return ADXL345(config)
