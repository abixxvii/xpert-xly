import time
import megaind
import sm_tc


# ++++++ SENSOR CLASSES ++++++

class Analog420mA:

    # Linear scaling:
    #     4 mA  - > 0
    #     20 mA - > full_scale

    def __init__(self, stack, channel, full_scale, label="Sensor", offset=0.0):
        self.stack = stack
        self.channel = channel
        self.full_scale = float(full_scale)
        self.label = label
        self.history = []
        self.offset = float(offset)

        # raw value acq
    def get_physical_val(self, smooth=False):
        mA = megaind.get4_20In(self.stack, self.channel)

        # loop health check
        if mA < 3.5:
            return 0.0, "OFFLINE"
        if mA > 20.5:
            return 0.0, "OVER-RANGE"

        # linear scaling
        val = ((mA - 4.0) / 16.0) * self.full_scale + self.offset
        val = max(0.0, val)

        # small deadband near zero — tightened for bench testing
        # 0.005 = 37.5 PSI on 7500 FS, too aggressive for low-pressure bench work
        # restore to 0.005 when installed on barrel
        if val < (self.full_scale * 0.001):
            val = 0.0

        # OPTIONAL smoothing
        if smooth:
            self.history.append(val)
            if len(self.history) > 5:
                self.history.pop(0)
            val = sum(self.history) / len(self.history)

        return val, f"{mA:6.3f} mA"


class RPMSensor:

    def __init__(self, stack, channel, ppr=1):
        self.stack = stack
        self.channel = channel
        self.ppr = max(1, int(ppr))

        megaind.setOptoRisingCountEnable(stack, channel, 1)
        megaind.setOptoFallingCountEnable(stack, channel, 0)
        megaind.rstOptoCount(stack, channel)

        self.last_count = megaind.getOptoCount(stack, channel)
        self.last_time = time.monotonic()
        self.last_pulse_time = self.last_time

        self.last_rpm = 0.0

    def read(self):
        now = time.monotonic()
        elapsed = now - self.last_time

    
        # #1 — window/frequency
        if elapsed >= 0.2:

            current_count = megaind.getOptoCount(self.stack, self.channel)

            # wrap-safe delta (assuming 16-bit (2 ^16) counter 0..65535)
            pulses = (current_count - self.last_count) % 65536

            if elapsed > 0:
                rpm = (pulses / elapsed) * 60.0 / self.ppr
            else:
                rpm = 0.0

            if pulses > 0:
                self.last_pulse_time = now

            self.last_count = current_count
            self.last_time = now
            self.last_rpm = max(0.0, rpm)

        # timeout protection (if shaft stopped)
        if (now - self.last_pulse_time) > 2.0:
            self.last_rpm = 0.0

        return self.last_rpm


       
        # #2 — period-Based, experimental(good for low speeds, bad for high speeds)
       
        # current_count = megaind.getOptoCount(self.stack, self.channel)
        # pulses = (current_count - self.last_count) % 65536

        # if pulses > 0:
        #     dt = now - self.last_pulse_time
        #     if dt > 0:
        #         rpm = 60.0 / (dt * self.ppr)
        #         self.last_rpm = 0.7 * self.last_rpm + 0.3 * rpm

        #     self.last_pulse_time = now
        #     self.last_count = current_count

        # # timeout protection
        # if (now - self.last_pulse_time) > 2.0:
        #     self.last_rpm = 0.0

        # return self.last_rpm
        

# +++++ ACQUISITION +++++

def initialize_hardware():
    pressure = Analog420mA(stack=0, channel=1, full_scale=7500.0)

    #########################################################
    CURRENT_FS_A = 100.0  # 100/150/200 acc to jumper

    current = Analog420mA(stack=0, channel=2, full_scale=CURRENT_FS_A)

    motor_rpm = RPMSensor(stack=0, channel=1, ppr=1)

    ACTIVE_TCS = list(range(1, 9))
    tc_board = sm_tc.SMtc(0)

    for ch in ACTIVE_TCS:
        tc_board.set_sensor_type(ch, 2)  # J-type

    return {
        "pressure": pressure,
        "current": current,
        "motor_rpm": motor_rpm,
        "tc_board": tc_board,
        "active_tcs": ACTIVE_TCS,
    }


def get_snapshot(sensors):
    pressure = sensors["pressure"]
    current = sensors["current"]
    motor_rpm = sensors["motor_rpm"]
    tc_board = sensors["tc_board"]
    ACTIVE_TCS = sensors["active_tcs"]

    ts_full = time.strftime("%Y-%m-%d %H:%M:%S")

    rpm_val = motor_rpm.read()
    psi_val, p_status = pressure.get_physical_val()
    amp_val, a_status = current.get_physical_val(smooth=True)

    tc_vals = {}
    for ch in ACTIVE_TCS:
        try:
            tc_vals[f"tc{ch}"] = round(tc_board.get_temp(ch), 1)
        except Exception:
            tc_vals[f"tc{ch}"] = None  # keeps pipeline alive

    return {
        "timestamp": ts_full,
        "rpm": round(rpm_val, 1),
        "psi": round(psi_val, 1),
        "amps": round(amp_val, 2),
        "thermocouples": tc_vals,
        "_status": {
            "pressure": p_status,
            "current": a_status
        }
    }