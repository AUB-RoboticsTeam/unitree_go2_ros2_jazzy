#!/usr/bin/env python3

"""Publish the Unitree Go2 BLE remote as sensor_msgs/Joy.

The bundled Unitree remote exposes a vendor GATT service instead of the
Bluetooth HID gamepad profile.  This node drives the installed BlueZ
``gatttool`` client through a pseudo-terminal, subscribes to the remote's
notifications, decodes its 20-byte packets, and publishes conventional Joy
messages for junior_ctrl.
"""

import math
import os
import pty
import re
import select
import shutil
import struct
import subprocess
import threading
import time

import rospy
from sensor_msgs.msg import Joy


NOTIFICATION_RE = re.compile(
    rb"Notification handle = 0x[0-9a-fA-F]+ value:\s*"
    rb"((?:[0-9a-fA-F]{2}\s+){19}[0-9a-fA-F]{2})"
)


class UnitreeBleJoy(object):
    """Connect to a Unitree BLE controller and publish its current state."""

    def __init__(self):
        self.address = str(rospy.get_param("~address", "98:DA:10:02:9C:4C"))
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 2.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 0.35))
        self.cccd_handle = int(rospy.get_param("~cccd_handle", 0x0024))
        self.write_handle = int(rospy.get_param("~write_handle", 0x0026))
        self.handshake_hex = str(rospy.get_param("~handshake_hex", "3539353332623332"))

        if not re.fullmatch(r"(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}", self.address):
            raise ValueError(f"Invalid Bluetooth address: {self.address}")
        if not re.fullmatch(r"(?:[0-9A-Fa-f]{2})+", self.handshake_hex):
            raise ValueError("handshake_hex must contain an even number of hex digits")

        self.publisher = rospy.Publisher(self.joy_topic, Joy, queue_size=10)
        self._stop_event = threading.Event()
        self._process_lock = threading.Lock()
        self._process = None
        self._master_fd = None
        self._last_packet_time = 0.0
        self._neutral_sent = True
        self._packet_count = 0

        # Wall-clock watchdog, so it keeps running even if /clock stalls.
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

        self._worker = threading.Thread(target=self._connection_loop, daemon=True)
        self._worker.start()

        rospy.loginfo(f"Unitree BLE remote {self.address} -> {self.joy_topic}")

    def shutdown(self):
        self._stop_event.set()
        self._stop_process()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

    def _connection_loop(self):
        executable = shutil.which("gatttool")
        if executable is None:
            rospy.logerr("gatttool was not found. Install the BlueZ command-line tools.")
            return

        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                self._run_session(executable)
            except Exception as exc:  # Keep reconnecting after transient BLE errors.
                if not self._stop_event.is_set():
                    rospy.logwarn(f"BLE connection lost: {exc}")
            finally:
                self._stop_process()
                self._publish_neutral_once()

            if self._stop_event.wait(max(0.1, self.reconnect_delay)):
                break

    def _run_session(self, executable):
        master_fd, slave_fd = pty.openpty()
        process = subprocess.Popen(
            [executable, "-b", self.address, "-t", "public", "-I"],
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            close_fds=True,
        )
        os.close(slave_fd)
        with self._process_lock:
            self._process = process
            self._master_fd = master_fd

        self._read_until(master_fd, b"[LE]>", 5.0)
        self._write_command(master_fd, "connect")
        self._read_until(master_fd, b"Connection successful", 12.0)

        self._write_command(master_fd, "mtu 64")
        time.sleep(0.25)
        self._write_command(
            master_fd, f"char-write-req 0x{self.cccd_handle:04x} 0100"
        )
        self._read_until(master_fd, b"written successfully", 5.0)
        self._write_command(
            master_fd,
            f"char-write-cmd 0x{self.write_handle:04x} {self.handshake_hex}",
        )

        rospy.loginfo(f"Connected to Unitree remote {self.address}")
        buffer = b""
        while not self._stop_event.is_set():
            if process.poll() is not None:
                raise RuntimeError(f"gatttool exited with status {process.returncode}")

            readable, _, _ = select.select([master_fd], [], [], 0.5)
            if not readable:
                continue
            chunk = os.read(master_fd, 4096)
            if not chunk:
                raise RuntimeError("controller closed the BLE connection")
            buffer += chunk

            last_end = 0
            for match in NOTIFICATION_RE.finditer(buffer):
                raw_hex = match.group(1).split()
                self._handle_packet(bytes(int(value, 16) for value in raw_hex))
                last_end = match.end()
            if last_end:
                buffer = buffer[last_end:]
            elif len(buffer) > 8192:
                buffer = buffer[-1024:]

    def _read_until(self, fd, expected, timeout):
        deadline = time.monotonic() + timeout
        data = b""
        while time.monotonic() < deadline and not self._stop_event.is_set():
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            data += chunk
            if expected in data:
                return data
            if b"connect error" in data.lower() or b"error:" in data.lower():
                break
        text = data.decode(errors="replace").replace("\r", " ").replace("\n", " ")
        raise RuntimeError(
            f"timed out waiting for {expected.decode(errors='replace')}: {text[-240:]}"
        )

    @staticmethod
    def _write_command(fd, command):
        os.write(fd, (command + "\n").encode())

    def _handle_packet(self, packet):
        if len(packet) != 20:
            return

        lx, rx, ry, ly, shoulder, face, battery, _rssi = struct.unpack(
            "<ffffBBBB", packet
        )
        axes = [
            self._safe_axis(lx),
            self._safe_axis(ly),
            0.0,
            self._safe_axis(rx),
            self._safe_axis(ry),
        ]

        # Indices 0..7 intentionally match RosCmdPanel's default mapping:
        # A, B, X, Y, L1, R1, L2, Start. Remaining controller inputs follow.
        buttons = [
            self._bit(face, 0),       # A
            self._bit(face, 1),       # B
            self._bit(face, 2),       # X
            self._bit(face, 3),       # Y
            self._bit(shoulder, 1),   # L1
            self._bit(shoulder, 0),   # R1
            self._bit(shoulder, 5),   # L2
            self._bit(shoulder, 2),   # Start
            self._bit(shoulder, 4),   # R2
            self._bit(shoulder, 3),   # Select
            self._bit(shoulder, 6),   # F1
            self._bit(shoulder, 7),   # F2
            self._bit(face, 4),       # D-pad up
            self._bit(face, 5),       # D-pad right
            self._bit(face, 6),       # D-pad down
            self._bit(face, 7),       # D-pad left
        ]

        message = Joy()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "unitree_ble_remote"
        message.axes = axes
        message.buttons = buttons
        if rospy.is_shutdown():
            return
        try:
            self.publisher.publish(message)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
            raise

        self._last_packet_time = time.monotonic()
        self._neutral_sent = False
        self._packet_count += 1
        if self._packet_count == 1:
            rospy.loginfo(
                f"Receiving controller data at approximately 20 Hz; battery {battery}%"
            )

    @staticmethod
    def _safe_axis(value):
        if not math.isfinite(value):
            return 0.0
        return float(max(-1.0, min(1.0, value)))

    @staticmethod
    def _bit(value, bit):
        return 1 if value & (1 << bit) else 0

    def _watchdog_loop(self):
        while not self._stop_event.wait(0.1):
            self._watchdog()

    def _watchdog(self):
        if (
            not self._neutral_sent
            and time.monotonic() - self._last_packet_time > self.stale_timeout
        ):
            rospy.logwarn("Controller data timed out; publishing neutral /joy")
            self._publish_neutral_once()

    def _publish_neutral_once(self):
        if self._neutral_sent:
            return
        if rospy.is_shutdown():
            self._neutral_sent = True
            return
        message = Joy()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "unitree_ble_remote"
        message.axes = [0.0] * 5
        message.buttons = [0] * 16
        try:
            self.publisher.publish(message)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise
        self._neutral_sent = True

    def _stop_process(self):
        with self._process_lock:
            process = self._process
            master_fd = self._master_fd
            self._process = None
            self._master_fd = None

        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        if master_fd is not None:
            try:
                os.close(master_fd)
            except OSError:
                pass


def main():
    rospy.init_node("unitree_ble_joy")
    node = UnitreeBleJoy()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
