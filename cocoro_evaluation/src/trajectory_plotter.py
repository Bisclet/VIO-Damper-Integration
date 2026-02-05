#!/usr/bin/env python3

import os
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
import rosbag2_py

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


class TrajectoryFromBag(Node):

    def __init__(self):
        super().__init__('trajectory_from_bag')

        self.declare_parameter("bag_path", "")
        bag_path = self.get_parameter("bag_path").get_parameter_value().string_value

        if bag_path == "":
            self.get_logger().error("Parameter bag_path is empty")
            exit(1)

        self.basalt_x = []
        self.basalt_y = []
        self.mavros_x = []
        self.mavros_y = []

        self.get_logger().info(f"Reading bag: {bag_path}")

        self.read_bag(bag_path)

    def read_bag(self, bag_path):

        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id="sqlite3"
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}

        while reader.has_next():
            topic, data, _ = reader.read_next()

            if topic not in type_map:
                continue

            if type_map[topic] != "nav_msgs/msg/Odometry":
                continue

            msg = deserialize_message(data, Odometry)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            if topic == "/basalt/odom":
                self.basalt_x.append(x)
                self.basalt_y.append(y)

            elif topic == "/mavros/local_position/odom":
                self.mavros_x.append(x)
                self.mavros_y.append(y)

        self.get_logger().info(
            f"Loaded {len(self.basalt_x)} basalt and {len(self.mavros_x)} mavros points"
        )


def rotate(x, y, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return c*x - s*y, s*x + c*y


def autoscale(ax, x, y):
    margin = 0.05
    xmin, xmax = np.min(x), np.max(x)
    ymin, ymax = np.min(y), np.max(y)
    dx = xmax - xmin
    dy = ymax - ymin
    ax.set_xlim(xmin - dx*margin, xmax + dx*margin)
    ax.set_ylim(ymin - dy*margin, ymax + dy*margin)


def interactive_plot(node):

    bx = np.array(node.basalt_x)
    by = np.array(node.basalt_y)
    mx = np.array(node.mavros_x)
    my = np.array(node.mavros_y)

    fig, axes = plt.subplots(2, 1, figsize=(8, 10))
    plt.subplots_adjust(bottom=0.25)

    line_b, = axes[0].plot(bx, by, 'r-')
    axes[0].set_title("Basalt trajectory")
    axes[0].axis('equal')
    axes[0].grid(True)

    line_m, = axes[1].plot(mx, my, 'b-')
    axes[1].set_title("MAVROS trajectory")
    axes[1].axis('equal')
    axes[1].grid(True)

    autoscale(axes[0], bx, by)
    autoscale(axes[1], mx, my)

    ax_b = plt.axes([0.2, 0.1, 0.65, 0.03])
    ax_m = plt.axes([0.2, 0.05, 0.65, 0.03])

    slider_b = Slider(ax_b, "Basalt rot Z", -np.pi, np.pi, valinit=0)
    slider_m = Slider(ax_m, "MAVROS rot Z", -np.pi, np.pi, valinit=0)

    def update(val):
        rb_x, rb_y = rotate(bx, by, slider_b.val)
        rm_x, rm_y = rotate(mx, my, slider_m.val)

        line_b.set_xdata(rb_x)
        line_b.set_ydata(rb_y)
        line_m.set_xdata(rm_x)
        line_m.set_ydata(rm_y)

        autoscale(axes[0], rb_x, rb_y)
        autoscale(axes[1], rm_x, rm_y)

        fig.canvas.draw_idle()

    slider_b.on_changed(update)
    slider_m.on_changed(update)

    plt.show()


def main():
    rclpy.init()
    node = TrajectoryFromBag()

    interactive_plot(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
