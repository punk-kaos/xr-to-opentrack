import subprocess
import socket
import re
import struct
import numpy as np
import time

# --- Full Fusion AHRS with accel + mag correction ---
class FusionAhrs:
    def __init__(self, beta=0.2, sample_rate=200):
        self.beta = beta
        self.sample_period = 1.0 / sample_rate
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_roll = 0.0
        self.last_yaw = 0.0
        self.last_roll_deg = 0.0
        self.last_yaw_deg = 0.0

    def update(self, gyro, accel, mag):
        q = self.q
        gyro = np.array(gyro) * 0.6
        gx, gy, gz = np.radians(gyro)
        ax, ay, az = accel
        mx, my, mz = mag

        norm = np.linalg.norm([ax, ay, az])
        if norm == 0:
            return
        ax, ay, az = ax / norm, ay / norm, az / norm

        norm = np.linalg.norm([mx, my, mz])
        if norm == 0:
            return
        mx, my, mz = mx / norm, my / norm, mz / norm

        q1, q2, q3, q4 = q

        f = np.array([
            2*(q2*q4 - q1*q3) - ax,
            2*(q1*q2 + q3*q4) - ay,
            2*(0.5 - q2**2 - q3**2) - az
        ])
        j = np.array([
            [-2*q3,  2*q4, -2*q1, 2*q2],
            [ 2*q2,  2*q1,  2*q4, 2*q3],
            [ 0.0,  -4*q2, -4*q3, 0.0]
        ])
        step = j.T @ f
        step /= np.linalg.norm(step)

        q_dot = 0.5 * self._quat_multiply(q, [0, gx, gy, gz]) - self.beta * step
        q += q_dot * self.sample_period
        self.q = q / np.linalg.norm(q)

    def _quat_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def unwrap_angle(self, current_deg, previous_deg):
        delta = current_deg - previous_deg
        if delta > 180:
            current_deg -= 360
        elif delta < -180:
            current_deg += 360
        return current_deg

    def get_rpy(self):
        q = self.q
        w, x, y, z = q
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        pitch = np.arcsin(2*(w*y - z*x))
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))

        roll, pitch, yaw = roll, -pitch, -yaw
        roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])

        roll_deg = self.unwrap_angle(roll_deg, self.last_roll_deg)
        self.last_roll_deg = roll_deg

        yaw_deg = self.unwrap_angle(yaw_deg, self.last_yaw_deg)
        self.last_yaw_deg = yaw_deg

        alpha = 0.15
        self.last_roll = self.last_roll * (1 - alpha) + roll_deg * alpha
        self.last_yaw = self.last_yaw * (1 - alpha) + yaw_deg * alpha

        return [self.last_roll, pitch_deg, self.last_yaw]

# --- Sensor driver + UDP transmitter ---
UDP_IP = "127.0.0.1"
UDP_PORT = 4242

def apply_deadzone(value, threshold=0.5):
    return 0.0 if abs(value) < threshold else value

def stream_with_fusion():
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

    ahrs = FusionAhrs(sample_rate=200)
    last_rpy = np.zeros(3)
    alpha = 0.2

    g_pattern = re.compile(r"G:\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)")
    a_pattern = re.compile(r"A:\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)")
    m_pattern = re.compile(r"M:\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)")

    g = a = m = None

    try:
        for line in proc.stdout:
            line = line.strip()
            if g_match := g_pattern.match(line):
                g = tuple(float(x) for x in g_match.groups())
            elif a_match := a_pattern.match(line):
                a = tuple(float(x) for x in a_match.groups())
            elif m_match := m_pattern.match(line):
                m = tuple(float(x) for x in m_match.groups())
            elif all((g, a, m)):
                ahrs.update(g, a, m)
                rpy = np.array(ahrs.get_rpy())
                smoothed = last_rpy * (1 - alpha) + rpy * alpha
                last_rpy = smoothed

                pitch_sens = 1.1
                yaw_sens = 0.8
                roll_sens = 0.8

                roll, pitch, yaw = smoothed
                roll = apply_deadzone(roll * roll_sens)
                pitch = apply_deadzone(pitch * pitch_sens)
                yaw = apply_deadzone(yaw * yaw_sens)

                msg = struct.pack('<6dQ', 0.0, 0.0, 0.0, yaw, pitch, roll, frame_number)
                sock.sendto(msg, (UDP_IP, UDP_PORT))
                frame_number += 1
                g = a = m = None

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        proc.terminate()
        proc.wait()
        sock.close()

if __name__ == "__main__":
    stream_with_fusion()

