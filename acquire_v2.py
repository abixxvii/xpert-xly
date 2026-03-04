import time
import megaind
import sm_tc


# ===========================================================================
# CALIBRATION CONSTANTS — measure these during setup
# ===========================================================================

# Pressure (0-7500 PSI)
#   ZERO: vent sensor to atmosphere, read mA -> set P_MA_ZERO
#   SPAN: apply a known reference pressure, read mA -> set P_MA_SPAN
#         (or leave at 20.0 if you trust the sensor's factory span)
P_MA_ZERO  = 4.0     # mA at 0 PSI
P_MA_SPAN  = 20.0    # mA at 7500 PSI
P_FULL_SCALE_PSI = 7500.0

# Current — set FULL SCALE to match your jumper setting (100 / 150 / 200)
C_MA_ZERO  = 4.0     # mA at 0 A
C_MA_SPAN  = 20.0    # mA at full scale
C_FULL_SCALE_A = 200.0   # <-- match your jumper

# Thermocouples
ACTIVE_TCS = list(range(1, 9))   # TC1..TC8, trim if fewer connected


# ===========================================================================
# SENSOR CLASSES
# ===========================================================================

class Analog420mA:
    """
    4-20mA sensor with two-point calibration.

    Two-point calibration formula:
        val = ((mA - ma_zero) / (ma_span - ma_zero)) * full_scale

    This corrects for both zero-offset and span errors in the real sensor,
    unlike the ideal-assumption formula that hardcodes 4.0 and 16.0.
    """

    def __init__(self, stack, channel, full_scale, label="Sensor",
                 ma_zero=4.0, ma_span=20.0):
        self.stack      = stack
        self.channel    = channel
        self.full_scale = float(full_scale)
        self.label      = label
        self.ma_zero    = float(ma_zero)
        self.ma_span    = float(ma_span)
        self._span      = self.ma_span - self.ma_zero   # pre-computed span
        self.history    = []

    def get_physical_val(self, smooth=False):
        mA = megaind.get4_20In(self.stack, self.channel)

        # loop health check
        if mA < 3.5:
            return 0.0, "OFFLINE"
        if mA > 20.5:
            return 0.0, "OVER-RANGE"

        # guard: avoid divide-by-zero if misconfigured
        if self._span == 0.0:
            return 0.0, "CAL-ERROR"

        # two-point calibrated conversion
        val = ((mA - self.ma_zero) / self._span) * self.full_scale
        val = max(0.0, min(self.full_scale, val))

        # small deadband near zero — suppresses noise at rest
        if val < (self.full_scale * 0.005):
            val = 0.0

        # optional rolling average smoothing
        if smooth:
            self.history.append(val)
            if len(self.history) > 5:
                self.history.pop(0)
            val = sum(self.history) / len(self.history)

        return val, "OK"


class RPMSensor:
    """
    Frequency-based RPM from optocoupler pulse counter.

    RPM = (pulses / elapsed_seconds) * 60 / PPR
    Counter wrap handled via % 65536 (16-bit hardware counter).
    Shaft-stopped timeout zeroes RPM after 2s with no pulses.
    """

    def __init__(self, stack, channel, ppr=1):
        self.stack   = stack
        self.channel = channel
        self.ppr     = max(1, int(ppr))

        megaind.setOptoRisingCountEnable(stack, channel, 1)
        megaind.setOptoFallingCountEnable(stack, channel, 0)
        megaind.rstOptoCount(stack, channel)

        self.last_count      = megaind.getOptoCount(stack, channel)
        self.last_time       = time.monotonic()
        self.last_pulse_time = self.last_time
        self.last_rpm        = 0.0

    def read(self):
        now     = time.monotonic()
        elapsed = now - self.last_time

        if elapsed >= 0.2:
            current_count = megaind.getOptoCount(self.stack, self.channel)
            pulses        = (current_count - self.last_count) % 65536

            rpm = (pulses / elapsed) * 60.0 / self.ppr if elapsed > 0 else 0.0

            if pulses > 0:
                self.last_pulse_time = now

            self.last_count = current_count
            self.last_time  = now
            self.last_rpm   = max(0.0, rpm)

        # shaft-stopped timeout
        if (now - self.last_pulse_time) > 2.0:
            self.last_rpm = 0.0

        return self.last_rpm


# ===========================================================================
# ACQUISITION
# ===========================================================================

def initialize_hardware():
    pressure = Analog420mA(
        stack=0, channel=1,
        full_scale=P_FULL_SCALE_PSI,
        label="Pressure",
        ma_zero=P_MA_ZERO,
        ma_span=P_MA_SPAN,
    )

    current = Analog420mA(
        stack=0, channel=2,
        full_scale=C_FULL_SCALE_A,
        label="Current",
        ma_zero=C_MA_ZERO,
        ma_span=C_MA_SPAN,
    )

    motor_rpm = RPMSensor(stack=0, channel=1, ppr=1)

    tc_board = sm_tc.SMtc(0)
    for ch in ACTIVE_TCS:
        tc_board.set_sensor_type(ch, 2)   # J-type

    return {
        "pressure":  pressure,
        "current":   current,
        "motor_rpm": motor_rpm,
        "tc_board":  tc_board,
        "active_tcs": ACTIVE_TCS,
    }


def get_snapshot(sensors):
    pressure  = sensors["pressure"]
    current   = sensors["current"]
    motor_rpm = sensors["motor_rpm"]
    tc_board  = sensors["tc_board"]
    active_tcs = sensors["active_tcs"]

    ts = time.strftime("%Y-%m-%d %H:%M:%S")

    rpm_val          = motor_rpm.read()
    psi_val, p_status = pressure.get_physical_val()
    amp_val, a_status = current.get_physical_val(smooth=True)

    tc_vals = {}
    for ch in active_tcs:
        try:
            tc_vals[f"tc{ch}"] = round(tc_board.get_temp(ch), 1)
        except Exception:
            tc_vals[f"tc{ch}"] = None

    return {
        "timestamp": ts,
        "rpm":       round(rpm_val, 1),
        "psi":       round(psi_val, 1),
        "amps":      round(amp_val, 2),
        "thermocouples": tc_vals,
        "_status": {
            "pressure": p_status,
            "current":  a_status,
        },
    }
