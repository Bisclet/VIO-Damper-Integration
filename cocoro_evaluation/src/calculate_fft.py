#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
import rosbag2_py

import matplotlib.pyplot as plt


class ImuFFTFromBag(Node):

    def __init__(self):
        super().__init__('imu_fft_from_bag')

        self.declare_parameter("bag_path", "")
        self.declare_parameter("topic", "/imu")
        self.declare_parameter("storage_id", "sqlite3")  # default = db3

        bag_path = self.get_parameter("bag_path").value
        self.topic = self.get_parameter("topic").value
        self.storage_id = self.get_parameter("storage_id").value

        if bag_path == "":
            self.get_logger().error("Parameter bag_path is empty")
            exit(1)

        if self.storage_id not in ["sqlite3", "mcap"]:
            self.get_logger().error(
                f"Invalid storage_id '{self.storage_id}'. Use 'sqlite3' or 'mcap'"
            )
            exit(1)

        self.t = []
        self.ax = []
        self.ay = []
        self.az = []

        self.get_logger().info(
            f"Reading bag: {bag_path} (storage: {self.storage_id})"
        )

        self.read_bag(bag_path)

    def read_bag(self, bag_path):

        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id=self.storage_id
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}

        if self.topic not in type_map:
            self.get_logger().error(f"Topic '{self.topic}' not found in bag")
            exit(1)

        while reader.has_next():
            topic, data, stamp = reader.read_next()

            if topic != self.topic:
                continue

            if type_map[topic] != "sensor_msgs/msg/Imu":
                continue

            msg = deserialize_message(data, Imu)

            self.t.append(stamp * 1e-9)
            self.ax.append(msg.linear_acceleration.x)
            self.ay.append(msg.linear_acceleration.y)
            self.az.append(msg.linear_acceleration.z)

        self.get_logger().info(f"Loaded {len(self.t)} IMU samples")


def compute_fft(t, signal):

    t = np.array(t)
    signal = np.array(signal)

    dt = np.mean(np.diff(t))
    fs = 1.0 / dt

    signal = signal - np.mean(signal)

    fft_vals = np.fft.fft(signal)
    freqs = np.fft.fftfreq(len(signal), d=dt)

    fft_mag = 2.0 / len(signal) * np.abs(fft_vals)

    mask = (freqs > 1) & (freqs <= 40.0)
    return freqs[mask], fft_mag[mask], fs


def integrate_displacement(t, accel):

    t = np.array(t)
    accel = np.array(accel)

    dt = np.diff(t, prepend=t[0])

    # Remove bias
    accel = accel - np.mean(accel)

    # Velocity
    vel = np.cumsum(accel * dt)

    # Remove linear drift
    vel = vel - np.linspace(vel[0], vel[-1], len(vel))

    # Displacement
    disp = np.cumsum(vel * dt)

    # Remove drift again
    disp = disp - np.linspace(disp[0], disp[-1], len(disp))

    # Center
    disp = disp - np.mean(disp)

    return disp


def annotate_top_peaks(ax, freqs, mags, n=3, min_sep=2.0):

    if len(freqs) == 0:
        return

    order = np.argsort(mags)[::-1]
    chosen = []

    for i in order:
        f = freqs[i]

        if all(abs(f - freqs[j]) >= min_sep for j in chosen):
            chosen.append(i)

        if len(chosen) == n:
            break

    for i in chosen:
        f = freqs[i]
        m = mags[i]

        ax.plot(f, m, 'ro')
        ax.annotate(f"{f:.2f} Hz",
                    xy=(f, m),
                    xytext=(5, 5),
                    textcoords="offset points",
                    fontsize=9,
                    color='red')


def plot_results(node):

    dx = integrate_displacement(node.t, node.ax)
    dy = integrate_displacement(node.t, node.ay)
    dz = integrate_displacement(node.t, node.az)

    fx, mx, fs = compute_fft(node.t, node.ax)
    fy, my, _ = compute_fft(node.t, node.ay)
    fz, mz, _ = compute_fft(node.t, node.az)

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    # FFT acceleration
    axes[0, 0].plot(fx, mx)
    annotate_top_peaks(axes[0, 0], fx, mx)
    axes[0, 0].set_title("FFT accel X")
    axes[0, 0].set_xlabel("Hz")
    axes[0, 0].grid(True)

    axes[0, 1].plot(fy, my)
    annotate_top_peaks(axes[0, 1], fy, my)
    axes[0, 1].set_title("FFT accel Y")
    axes[0, 1].set_xlabel("Hz")
    axes[0, 1].grid(True)

    axes[0, 2].plot(fz, mz)
    annotate_top_peaks(axes[0, 2], fz, mz)
    axes[0, 2].set_title("FFT accel Z")
    axes[0, 2].set_xlabel("Hz")
    axes[0, 2].grid(True)

    # Displacement orbits
    axes[1, 0].plot(dx, dy)
    axes[1, 0].set_title("Displacement Orbit XY")
    axes[1, 0].axis('equal')
    axes[1, 0].grid(True)

    axes[1, 1].plot(dx, dz)
    axes[1, 1].set_title("Displacement Orbit XZ")
    axes[1, 1].axis('equal')
    axes[1, 1].grid(True)

    axes[1, 2].plot(dy, dz)
    axes[1, 2].set_title("Displacement Orbit YZ")
    axes[1, 2].axis('equal')
    axes[1, 2].grid(True)

    fig.suptitle(
        f"IMU FFT + Displacement Orbits (IMU freq ≈ {fs:.1f} Hz)",
        fontsize=16
    )

    plt.tight_layout()
    plt.show()


def main():
    rclpy.init()
    node = ImuFFTFromBag()

    plot_results(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
