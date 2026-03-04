"""
Extrusion Facility - Sensor Calibration & Test Script
======================================================
Hardware:
  - Pressure : MT204P7.5MST  (MPI)      -- 4-20mA, 0-7500 PSI
  - Current  : CCT50-100     (Dwyer)    -- 4-20mA, 0-100 A
  - RPM      : IES201        (IFM)      -- Inductive prox, pulse count
  - Temp     : Dual-ch J-type TC        -- via sm_tc board

DAQ: Sequent Microsystems Industrial HAT (megaind + sm_tc)

CALIBRATION PROCEDURE:
-----------------------
Pressure:
  1. Vent sensor to atmosphere (0 PSI), read raw mA -> set PRESSURE_MA_ZERO
  2. Apply known reference pressure, read raw mA -> set PRESSURE_MA_SPAN
     (Expected mA at ref: 4.0 + (ref_psi / 7500.0) * 16.0)

Current:
  1. Clamp sensor with no load (0A) -> set CURRENT_MA_ZERO
  2. Run at a known load (measured by clamp meter) -> set CURRENT_MA_SPAN
     (Expected mA at ref: 4.0 + (ref_amps / 100.0) * 16.0)

RPM:
  1. Count targets on your shaft physically -> set MOTOR_PPR
  2. Verify with a strobe tach or phone camera at a known speed

Thermocouples:
  1. Put probes in boiling water -> should read ~100C (adjust for altitude)
  2. If offset, note it and set TC_OFFSET per channel
"""

import time
import csv
import os
import threading
import statistics
import megaind
import sm_tc

# ===========================================================================
# CALIBRATION CONSTANTS — EDIT THESE
# ===========================================================================

# --- Pressure (MT204P7.5MST) ---
# Full scale in PSI for installed pressure transducer
PRESSURE_FULL_SCALE_PSI = 7500.0
PRESSURE_MA_ZERO  = 4.0    # <-- mA reading at 0 PSI (vent to atmosphere, measure this)
PRESSURE_MA_SPAN  = 20.0   # <-- mA reading at full scale (or interpolate from known ref)

# --- Current (CCT50-100) ---
CURRENT_FULL_SCALE_AMPS = 100.0
CURRENT_MA_ZERO  = 4.0     # <-- mA at 0A
CURRENT_MA_SPAN  = 20.0    # <-- mA at full scale

# --- RPM (IES201) ---
MOTOR_PPR = 1              # <-- targets on shaft (count physically)

# --- Thermocouples ---
ACTIVE_TCS = list(range(1, 3))   # <-- change to range(1, 9) if you have 8 TCs
TC_OFFSET  = {ch: 0.0 for ch in range(1, 9)}  # per-channel offset in °C after boiling water test

# ===========================================================================
# FILTER CONFIG
# ===========================================================================

# EMA alpha: 0.1 = very smooth/laggy, 0.5 = balanced, 0.9 = fast/noisy
# Melt pressure changes slowly -> 0.2 is reasonable
# Current changes faster -> 0.4
PRESSURE_EMA_ALPHA = 0.2
CURRENT_EMA_ALPHA  = 0.4

# Median filter window for RPM (rejects spike outliers, must be odd)
RPM_MEDIAN_WINDOW = 5

# ===========================================================================
# HELPERS
# ===========================================================================

def linear_4_20(ma, ma_zero, ma_span, full_scale):
    """
    Two-point calibrated 4-20mA to engineering units.

    Math:
        Normalize mA to 0-1 range using actual calibrated zero and span:
            normalized = (mA - ma_zero) / (ma_span - ma_zero)
        Scale to full range:
            value = normalized * full_scale

        This is just solving for the equation of a line y = mx + b where:
            m = full_scale / (ma_span - ma_zero)
            b = -ma_zero * m
    """
    if ma_span == ma_zero:
        return 0.0  # guard against division by zero
    val = (ma - ma_zero) / (ma_span - ma_zero) * full_scale
    return max(0.0, min(full_scale, val))   # clamp to valid range


class EMAFilter:
    """
    Exponential Moving Average filter.

    Each output = alpha * new_input + (1 - alpha) * last_output

    Intuition: alpha is how much you trust the newest reading vs history.
    First sample seeds the filter directly (no lag on startup).
    """
    def __init__(self, alpha):
        self.alpha = alpha
        self._value = None

    def update(self, new_val):
        if self._value is None:
            self._value = new_val   # seed on first sample
        else:
            self._value = self.alpha * new_val + (1.0 - self.alpha) * self._value
        return self._value

    @property
    def value(self):
        return self._value


class MedianFilter:
    """
    Median filter — keeps a rolling window, returns the middle value.

    Unlike a mean, the median is resistant to outliers/spikes.
    A single bad reading in a window of 5 cannot shift the result
    by more than moving from position 2 to position 3 in the sorted list.
    """
    def __init__(self, window=5):
        self.window = window
        self._buf = []

    def update(self, new_val):
        self._buf.append(new_val)
        if len(self._buf) > self.window:
            self._buf.pop(0)
        return statistics.median(self._buf)

    @property
    def value(self):
        if not self._buf:
            return 0.0
        return statistics.median(self._buf)


# ===========================================================================
# SENSOR CLASSES
# ===========================================================================

class Analog420mA:
    def __init__(self, stack, channel, full_scale, label,
                 ma_zero, ma_span, ema_alpha=0.3):
        self.stack      = stack
        self.channel    = channel
        self.full_scale = full_scale
        self.label      = label
        self.ma_zero    = ma_zero
        self.ma_span    = ma_span
        self._ema       = EMAFilter(ema_alpha)
        self.raw_ma     = 0.0
        self.status     = "INIT"

    def read(self):
        """Returns (filtered_value, raw_value, status_string)"""
        ma = megaind.get4_20In(self.stack, self.channel)
        self.raw_ma = ma

        if ma < 3.0:
            self.status = "OFFLINE"
            return 0.0, ma, "OFFLINE"

        if ma < 3.8:
            self.status = "UNDER-RANGE"
            return 0.0, ma, "UNDER-RANGE"

        raw_val = linear_4_20(ma, self.ma_zero, self.ma_span, self.full_scale)
        filtered_val = self._ema.update(raw_val)
        self.status = "OK"

        return filtered_val, ma, "OK"


class RPMSensor:
    """
    Threaded RPM sensor — decouples timing from main loop.

    Math:
        Pulses counted over elapsed time gives frequency in Hz.
        RPM = (pulses / elapsed) * 60 / PPR

        PPR = pulses per revolution (number of targets on shaft).
        Running in its own thread with a fixed 0.5s window gives
        consistent timing regardless of what the main loop is doing.

    Median filter rejects missed/double-count outliers.
    """
    def __init__(self, stack, channel, ppr=1):
        self.stack  = stack
        self.channel = channel
        self.ppr    = ppr
        self._filter = MedianFilter(window=RPM_MEDIAN_WINDOW)
        self._rpm   = 0.0
        self._lock  = threading.Lock()
        self._last_count = megaind.getOptoCount(stack, channel)
        self._last_time  = time.time()
        megaind.setOptoRisingCountEnable(stack, channel, 1)
        t = threading.Thread(target=self._poll, daemon=True)
        t.start()

    def _poll(self):
        while True:
            time.sleep(0.5)   # fixed measurement window
            now     = time.time()
            elapsed = now - self._last_time
            count   = megaind.getOptoCount(self.stack, self.channel)
            pulses  = count - self._last_count

            self._last_count = count
            self._last_time  = now

            if elapsed > 0:
                raw_rpm = (pulses / elapsed) * 60.0 / self.ppr
                filtered = self._filter.update(max(0.0, raw_rpm))
                with self._lock:
                    self._rpm = filtered

    def read(self):
        with self._lock:
            return self._rpm


class ThermocoupleSensor:
    def __init__(self, stack, channels, tc_type=2):
        """
        tc_type: 2 = J-type (per sm_tc library)
        Cold junction compensation is handled onboard — nothing to do in SW.
        """
        self.board    = sm_tc.SMtc(stack)
        self.channels = channels
        for ch in channels:
            self.board.set_sensor_type(ch, tc_type)

    def read_all(self):
        """Returns dict {channel: temp_C}"""
        results = {}
        for ch in self.channels:
            try:
                raw = self.board.get_temp(ch)
                corrected = raw + TC_OFFSET.get(ch, 0.0)
                results[ch] = round(corrected, 1)
            except Exception:
                results[ch] = None   # open/disconnected TC
        return results


# ===========================================================================
# DIAGNOSTIC / CALIBRATION MODE
# ===========================================================================

def run_calibration_check(pressure, current, rpm, tc):
    """
    Run this first to verify your calibration constants.
    Prints raw mA values so you can set PRESSURE_MA_ZERO etc.
    """
    print("\n" + "="*50)
    print("  CALIBRATION CHECK MODE")
    print("  Watch raw mA values and compare to expectations")
    print("  Ctrl+C to exit")
    print("="*50)

    try:
        while True:
            p_val, p_ma, p_st = pressure.read()
            c_val, c_ma, c_st = current.read()
            r_val = rpm.read()
            temps = tc.read_all()

            print(f"\n[{time.strftime('%H:%M:%S')}]")
            print(f"  PRESSURE : {p_ma:6.3f} mA raw | {p_val:7.1f} PSI  | {p_st}")
            print(f"    zero={PRESSURE_MA_ZERO:.3f}mA  span={PRESSURE_MA_SPAN:.3f}mA")
            print(f"    @ 0 PSI  expected mA: {PRESSURE_MA_ZERO:.3f}")
            print(f"    @ full   expected mA: {PRESSURE_MA_SPAN:.3f}")

            print(f"  CURRENT  : {c_ma:6.3f} mA raw | {c_val:7.2f} A    | {c_st}")
            print(f"    zero={CURRENT_MA_ZERO:.3f}mA  span={CURRENT_MA_SPAN:.3f}mA")

            print(f"  RPM      : {r_val:7.1f} RPM (PPR={MOTOR_PPR})")

            for ch, temp in temps.items():
                if temp is None:
                    print(f"  TC{ch}       : OPEN/DISCONNECTED")
                else:
                    print(f"  TC{ch}       : {temp:6.1f} °C  (offset={TC_OFFSET.get(ch,0.0):+.1f})")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nCalibration check done.")


# ===========================================================================
# MAIN LOGGING LOOP
# ===========================================================================

def main(mode="log"):
    pressure = Analog420mA(
        stack=0, channel=1,
        full_scale=PRESSURE_FULL_SCALE_PSI,
        label="Pressure",
        ma_zero=PRESSURE_MA_ZERO,
        ma_span=PRESSURE_MA_SPAN,
        ema_alpha=PRESSURE_EMA_ALPHA
    )

    current = Analog420mA(
        stack=0, channel=2,
        full_scale=CURRENT_FULL_SCALE_AMPS,
        label="Current",
        ma_zero=CURRENT_MA_ZERO,
        ma_span=CURRENT_MA_SPAN,
        ema_alpha=CURRENT_EMA_ALPHA
    )

    rpm = RPMSensor(stack=0, channel=1, ppr=MOTOR_PPR)
    tc  = ThermocoupleSensor(stack=0, channels=ACTIVE_TCS)

    if mode == "calibrate":
        run_calibration_check(pressure, current, rpm, tc)
        return

    # --- Logging mode ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    timestamp  = time.strftime("%Y%m%d_%H%M%S")
    filename   = os.path.join(script_dir, f"bench_test_{timestamp}.csv")

    print(f"--- LOGGING TO: {filename} ---")

    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)

        header = ["Timestamp", "RPM", "PSI", "Amps", "PSI_raw_mA", "Amps_raw_mA"] + \
                 [f"TC{ch}_C" for ch in ACTIVE_TCS]
        writer.writerow(header)

        try:
            while True:
                ts_full = time.strftime("%Y-%m-%d %H:%M:%S")
                ts_cli  = time.strftime("%H:%M:%S")

                p_val, p_ma, p_st = pressure.read()
                c_val, c_ma, c_st = current.read()
                r_val = rpm.read()
                temps = tc.read_all()

                row = [ts_full,
                       round(r_val, 1),
                       round(p_val, 1),
                       round(c_val, 2),
                       round(p_ma, 3),
                       round(c_ma, 3)] + \
                      [temps.get(ch, None) for ch in ACTIVE_TCS]

                writer.writerow(row)
                f.flush()

                # --- CLI display ---
                print("\033c", end="")
                print("==========================================")
                print(f"   LIVE BENCH TEST | {ts_cli}")
                print(f"   FILE: {os.path.basename(filename)}")
                print("==========================================")
                print(f" MOTOR SPEED : {r_val:7.1f} RPM")
                print(f" MELT PRESS  : {p_val:7.1f} PSI  ({p_ma:.3f} mA) [{p_st}]")
                print(f" MOTOR LOAD  : {c_val:7.2f} A    ({c_ma:.3f} mA) [{c_st}]")
                print("------------------------------------------")
                for ch in ACTIVE_TCS:
                    t = temps.get(ch)
                    display = f"{t:.1f} °C" if t is not None else "OPEN"
                    print(f" TC{ch:<2}        : {display}")
                print("==========================================")
                print(" Ctrl+C to stop | run with 'calibrate' arg to check cal")

                time.sleep(0.5)

        except KeyboardInterrupt:
            print(f"\n[DONE] Data saved to {filename}")


# ===========================================================================
# ENTRY POINT
# ===========================================================================

if __name__ == "__main__":
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "log"
    # Usage:
    #   python sensor_test.py            -> logging mode
    #   python sensor_test.py calibrate  -> raw mA diagnostic mode
    main(mode=mode)