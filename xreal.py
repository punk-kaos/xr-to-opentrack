import subprocess
import socket
import re
import struct
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 4242

def stream_from_driver():
    # Launch the driver process
    proc = subprocess.Popen(
        ["./xrealAirLinuxDriver"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1
    )

    print("Started xrealAirLinuxDriver...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_number = 0

    pattern = re.compile(r"Roll:\s*(-?\d+\.\d+);\s*Pitch:\s*(-?\d+\.\d+);\s*Yaw:\s*(-?\d+\.\d+)")

    try:
        for line in proc.stdout:
            match = pattern.search(line)
            if match:
                roll = float(match.group(1))
                pitch = float(match.group(2))
                yaw = float(match.group(3))

                msg = struct.pack(
                    '<6dQ',
                    0.0, 0.0, 0.0,  # x, y, z
                    yaw, pitch, roll,
                    frame_number
                )
                sock.sendto(msg, (UDP_IP, UDP_PORT))

                print(f"Sent to OpenTrack: yaw={yaw:.2f}, pitch={pitch:.2f}, roll={roll:.2f}")
                frame_number += 1

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        proc.terminate()
        proc.wait()
        sock.close()

if __name__ == "__main__":
    stream_from_driver()
