import time, csv, sys, os
import megaind
import sm_tc

STACK = 0

PRESSURE_CH = 1
CURRENT_CH  = 2
RPM_CH      = 1
TC_CHANNELS = list(range(1, 9))

SAMPLE_PERIOD_S = 0.5

PRESSURE_MAX_PSI = 7500.0   # 0–7500 psi
CURRENT_MAX_A    = 100.0    # set acc to jumper range (100/150/200)
RPM_PPR          = 1

P_ZERO_MA, P_SPAN_MA = 4.0, 20.0
C_ZERO_MA, C_SPAN_MA = 4.0, 20.0

def map_4_20(ma, zero, span, full_scale):
    # Return (value, status)
    if ma < 3.6:
        return 0.0, "UNDER/OPEN"
    if ma > 21.0:
        return full_scale, "OVER"
    val = (ma - zero) / (span - zero) * full_scale
    if val < 0:
        return 0.0, "UNDER"
    if val > full_scale:
        return full_scale, "OVER"
    return val, "OK"

class MovingAvg:
    def __init__(self, n):
        self.n = n
        self.buf = []
    def add(self, x):
        self.buf.append(x)
        if len(self.buf) > self.n:
            self.buf.pop(0)
        return sum(self.buf) / len(self.buf)

def main(mode="log"):
    tc = sm_tc.SMtc(STACK)
    # smtc type mapping: [0..7] -> [B, E, J, K, N, R, S, T], so J=2
    for ch in TC_CHANNELS:
        tc.set_sensor_type(ch, 2)

    megaind.setOptoRisingCountEnable(STACK, RPM_CH, 1)

    last_count = megaind.getOptoCount(STACK, RPM_CH)
    last_t = time.monotonic()

    p_filt = MovingAvg(5)
    c_filt = MovingAvg(5)
    rpm_filt = MovingAvg(3)

    if mode == "log":
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = f"bench_test_{ts}.csv"
        with open(filename, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "Timestamp",
                "RPM",
                "Pressure_mA","Pressure_PSI","Pressure_Status",
                "Current_mA","Current_A","Current_Status",
                *[f"TC{ch}_C" for ch in TC_CHANNELS],
            ])
            print(f"--- LOGGING TO: {filename} ---")

            next_t = time.monotonic()
            while True:
                next_t += SAMPLE_PERIOD_S

                now_wall = time.strftime("%Y-%m-%d %H:%M:%S")
                now = time.monotonic()

                # --- Analog reads with status ---
                try:
                    p_ma = megaind.get4_20In(STACK, PRESSURE_CH)
                except Exception:
                    p_ma = float("nan")
                try:
                    c_ma = megaind.get4_20In(STACK, CURRENT_CH)
                except Exception:
                    c_ma = float("nan")

                p_val, p_status = map_4_20(p_ma if p_ma == p_ma else 0.0, P_ZERO_MA, P_SPAN_MA, PRESSURE_MAX_PSI)
                c_val, c_status = map_4_20(c_ma if c_ma == c_ma else 0.0, C_ZERO_MA, C_SPAN_MA, CURRENT_MAX_A)

                p_s = p_filt.add(p_val)
                c_s = c_filt.add(c_val)

                # --- RPM ---
                try:
                    count = megaind.getOptoCount(STACK, RPM_CH)
                except Exception:
                    count = last_count

                dt = max(1e-6, now - last_t)
                dc = count - last_count
                if dc < 0:
                    # crude wrap handling; if you know counter width, handle properly
                    dc = 0

                rpm = (dc / dt) * 60.0 / RPM_PPR
                rpm = rpm_filt.add(max(0.0, rpm))

                last_count, last_t = count, now

                # --- Temps ---
                temps = []
                for ch in TC_CHANNELS:
                    try:
                        temps.append(tc.get_temp(ch))
                    except Exception:
                        temps.append("")

                # --- Write row ---
                w.writerow([now_wall,
                            round(rpm, 2),
                            p_ma, round(p_s, 2), p_status,
                            c_ma, round(c_s, 2), c_status,
                            *temps])
                f.flush()

                # --- Simple live view ---
                print("\033c", end="")
                print(f"=== LIVE DATA | {now_wall} ===")
                print(f"RPM:   {rpm:8.2f}")
                print(f"PRESS: {p_s:8.2f} PSI   ({p_status}, {p_ma:.3f} mA)")
                print(f"CURR:  {c_s:8.2f} A     ({c_status}, {c_ma:.3f} mA)")
                print("TEMPS: " + " | ".join(f"TC{ch}:{temps[i] if temps[i] != '' else 'ERR'}"
                                         for i, ch in enumerate(TC_CHANNELS)))

                # fixed cadence sleep
                time.sleep(max(0.0, next_t - time.monotonic()))
    else:
        print("Use mode=log in this version.")

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "log"
    main(mode)