import time
from acquire import initialize_hardware, get_snapshot

sensors = initialize_hardware()

print("--- LIVE SENSOR DATA | Ctrl+C to stop ---\n")

try:
    while True:
        data = get_snapshot(sensors)

        tcs = data["thermocouples"]
        tc_str = "  ".join(
            f"TC{ch}: {val}°C" if val is not None else f"TC{ch}: OPEN"
            for ch, val in tcs.items()
        )

        print(f"[{data['timestamp']}]")
        print(f"  RPM   : {data['rpm']:>8.1f}")
        print(f"  PSI   : {data['psi']:>8.1f}  [{data['_status']['pressure']}]")
        print(f"  AMPS  : {data['amps']:>8.2f}  [{data['_status']['current']}]")
        print(f"  TEMPS : {tc_str}")
        print()

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped.")
