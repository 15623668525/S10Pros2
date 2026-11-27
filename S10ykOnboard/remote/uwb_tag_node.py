#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nooploop uBeacon Tag (UART V1.8) -> ROS2 publisher node.

Serial: /dev/ttyCH343USB0 @115200 8N1 (vendor driver usb_ch343)
Frame : 0xAA | payload_size(u16 LE) | payload | checksum(u8, sum of all prior & 0xFF)
        payload = UID(6) + FrameID(1) + MsgID(1) + MsgLen(1) + MsgPayload

Topics:
  /uwb/pose      geometry_msgs/PoseWithCovarianceStamped  position [m] + noise covariance
  /uwb/velocity  geometry_msgs/TwistStamped               velocity [m/s]
  /uwb/status    std_msgs/String                          JSON diagnostics, 1 Hz
"""

import json
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None

SYNC = 0xAA
MSG_LOCATION_RESULT = 0x44
FIXED_LEN = 36          # Location_time(8)+Pos(12)+Vel(6)+PosNoise(3)+VelNoise(3)+Map(1)+Err(1)+Rsv(1)+Cnt(1)
MIN_PAYLOAD = 12
MAX_PAYLOAD = 512


class FrameParser(object):
    """Resync-safe stream parser with checksum validation."""

    def __init__(self):
        self.buf = bytearray()
        self.good = 0
        self.bad_sum = 0
        self.bad_len = 0

    def reset(self):
        self.buf = bytearray()

    def feed(self, chunk):
        self.buf += chunk
        frames = []
        while True:
            idx = self.buf.find(b"\xaa")
            if idx < 0:
                self.buf = bytearray()
                return frames
            if idx:
                del self.buf[:idx]
            if len(self.buf) < 3:
                return frames
            payload_size = self.buf[1] | (self.buf[2] << 8)
            if payload_size < MIN_PAYLOAD or payload_size > MAX_PAYLOAD:
                self.bad_len += 1
                del self.buf[:1]
                continue
            total = payload_size + 4
            if len(self.buf) < total:
                return frames
            frame = bytes(self.buf[:total])
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                self.bad_sum += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            self.good += 1
            frames.append(frame)


def parse_location(frame):
    """Return dict for MSG_LOCATION_RESULT, or None if not that message."""
    msg_id = frame[10]
    if msg_id != MSG_LOCATION_RESULT:
        return None
    msg_len = frame[11]
    t_us = struct.unpack_from("<Q", frame, 12)[0]
    px, py, pz = struct.unpack_from("<3f", frame, 20)
    vx, vy, vz = struct.unpack_from("<3h", frame, 32)
    pn = (frame[38], frame[39], frame[40])
    vn = (frame[41], frame[42], frame[43])
    map_id = frame[44]
    err_area = frame[45]
    anchor_cnt = frame[47]

    anchors = []
    end = len(frame) - 1
    off = 48
    while off + 4 <= end:
        addr = frame[off] | (frame[off + 1] << 8)
        rssi = frame[off + 2]
        rate = frame[off + 3]
        anchors.append({
            "addr": addr,
            "rssi_dbm": round(-rssi / 2.0, 1),
            "rate": round(rate / 255.0, 3),
        })
        off += 4

    derived_cnt = max(0, (msg_len - FIXED_LEN) // 4) if msg_len >= FIXED_LEN else 0
    return {
        "uid": frame[3:9].hex(),
        "msg_len": msg_len,
        "device_time_us": t_us,
        "pos": (px, py, pz),
        "vel_ms": (vx / 100.0, vy / 100.0, vz / 100.0),
        "pos_noise_m": (pn[0] / 100.0, pn[1] / 100.0, pn[2] / 100.0),
        "vel_noise_ms": (vn[0] / 100.0, vn[1] / 100.0, vn[2] / 100.0),
        "map_id": map_id,
        "error_code": err_area & 0x0F,
        "area_id": (err_area >> 4) & 0x0F,
        "anchor_cnt": anchor_cnt,
        "anchor_cnt_derived": derived_cnt,
        "anchors": anchors,
    }


class UwbTagNode(Node):

    def __init__(self):
        super().__init__("uwb_tag_node")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyCH343USB0").value
        self.baud = p("baud", 115200).value
        self.frame_id = p("frame_id", "uwb_map").value
        self.pose_topic = p("pose_topic", "/uwb/pose").value
        self.vel_topic = p("velocity_topic", "/uwb/velocity").value
        self.status_topic = p("status_topic", "/uwb/status").value
        self.publish_velocity = p("publish_velocity", True).value
        self.publish_status = p("publish_status", True).value
        self.status_period = p("status_period", 1.0).value
        self.z_offset = p("z_offset", 0.0).value
        self.default_sigma = p("default_sigma_m", 0.30).value
        self.stale_reopen_sec = p("stale_reopen_sec", 5.0).value

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 20)
        self.pub_vel = self.create_publisher(TwistStamped, self.vel_topic, 20)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        self.lock = threading.Lock()
        self.count = 0
        self.count_prev = 0
        self.last_msg = None
        self.last_rx = 0.0
        self.reopen = 0
        self.parser = FrameParser()
        self.connected = False

        self.stop_ev = threading.Event()
        self.thread = threading.Thread(target=self._reader_loop, name="uwb_reader")
        self.thread.daemon = True
        self.thread.start()

        if self.publish_status:
            self.create_timer(max(0.2, float(self.status_period)), self._publish_status)
        self.get_logger().info(
            "uBeacon node up: %s @%d -> %s" % (self.port, self.baud, self.pose_topic))

    def _reader_loop(self):
        if serial is None:
            self.get_logger().error("pyserial not installed, cannot open serial")
            return
        while not self.stop_ev.is_set() and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    self.parser.reset()
                    self.connected = True
                    self.get_logger().info("serial opened: %s" % self.port)
                    last_data = time.time()
                    while not self.stop_ev.is_set() and rclpy.ok():
                        chunk = ser.read(max(1, ser.in_waiting))
                        if not chunk:
                            if time.time() - last_data > float(self.stale_reopen_sec):
                                self.get_logger().warn(
                                    "no data for %.1fs -> reopen" % (time.time() - last_data))
                                break
                            continue
                        last_data = time.time()
                        for frame in self.parser.feed(chunk):
                            self._handle_frame(frame)
            except Exception as exc:
                self.get_logger().warn("serial error: %r" % (exc,))
            finally:
                self.connected = False
            self.reopen += 1
            self.stop_ev.wait(1.0)

    def _handle_frame(self, frame):
        info = parse_location(frame)
        if info is None:
            return
        stamp = self.get_clock().now().to_msg()

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        px, py, pz = info["pos"]
        pose.pose.pose.position.x = float(px)
        pose.pose.pose.position.y = float(py)
        pose.pose.pose.position.z = float(pz) + float(self.z_offset)
        pose.pose.pose.orientation.w = 1.0
        cov = pose.pose.covariance
        vals = info["pos_noise_m"]
        for axis in range(3):
            sigma = vals[axis] if vals[axis] > 0.0 else float(self.default_sigma)
            if sigma > 5.0:      # 255 cm placeholder => bogus, fall back
                sigma = float(self.default_sigma)
            cov[axis * 7] = sigma * sigma
        self.pub_pose.publish(pose)

        if self.publish_velocity:
            tw = TwistStamped()
            tw.header.stamp = stamp
            tw.header.frame_id = self.frame_id
            tw.twist.linear.x = info["vel_ms"][0]
            tw.twist.linear.y = info["vel_ms"][1]
            tw.twist.linear.z = info["vel_ms"][2]
            self.pub_vel.publish(tw)

        with self.lock:
            self.count += 1
            self.last_msg = info
            self.last_rx = time.time()

    def _publish_status(self):
        now = time.time()
        with self.lock:
            cnt = self.count
            last = dict(self.last_msg) if self.last_msg else None
            age = (now - self.last_rx) if self.last_rx else -1.0
        fps = (cnt - self.count_prev) / max(1e-6, float(self.status_period))
        self.count_prev = cnt
        payload = {
            "connected": self.connected,
            "fps": round(fps, 2),
            "frames_ok": cnt,
            "bad_checksum": self.parser.bad_sum,
            "bad_length": self.parser.bad_len,
            "reopens": self.reopen,
            "age_sec": round(age, 2) if age >= 0 else -1.0,
            "port": self.port,
        }
        if last:
            payload.update({
                "uid": last["uid"],
                "pos": [round(v, 3) for v in last["pos"]],
                "vel_ms": [round(v, 3) for v in last["vel_ms"]],
                "pos_noise_m": [round(v, 3) for v in last["pos_noise_m"]],
                "map_id": last["map_id"],
                "area_id": last["area_id"],
                "error_code": last["error_code"],
                "anchor_cnt": last["anchor_cnt"],
                "anchor_cnt_derived": last["anchor_cnt_derived"],
                "anchors": last["anchors"],
                "device_time_us": last["device_time_us"],
            })
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(msg)

    def shutdown(self):
        self.stop_ev.set()
        self.thread.join(timeout=2.0)


def main():
    rclpy.init()
    node = UwbTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
