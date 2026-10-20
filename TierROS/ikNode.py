import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import time 
import serial 
import math as ma
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import UInt8MultiArray  

SERIAL_PORT = "/dev/cu.usbmodem11201"  
BAUD_RATE = 115200
UPDATE_INTERVAL_MS = 40               

s1d = 1
s2d = 1 
s1o = 90 
s2o = 90 

l_1 = 4.764
l_2 = 4.773
fixed = np.array([0.0, 0.0])

a = 0.02
l_a = 0.05

pico = None


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, '/serial_write', 10)

    def send_joint_offsets(self, absolute):
        msg = UInt8MultiArray() 

        delta_deg = np.degrees(np.asarray(absolute, dtype=float))
        offsets = np.rint(delta_deg).astype(int) # Rounds the interger cause w`e can't all that 
        command = f"A,0,0,{offsets[0]},{offsets[1]},0,0\n"
        msg.data = command.encode("ascii")

        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)
        
        



def plotAngle(angles):
    x1 = l_1 * ma.cos(angles[0])
    y1 = l_1 * ma.sin(angles[0])

    x2 = l_1 * ma.cos(angles[0]) + l_2 * ma.cos(angles[0] + angles[1])
    y2 = l_1 * ma.sin(angles[0]) + l_2 * ma.sin(angles[0] + angles[1])

    return x1, y1, x2, y2


def main():
    jointAngles = np.array([(ma.pi/2), -1*(ma.pi / 2)], dtype=float)
    target = np.array([4.0, 4.5], dtype=float)

    fig, ax = plt.subplots(figsize=(6, 6))
    fig.subplots_adjust(bottom=0.15)

    ax.set_xlim(-15, 15)
    ax.set_ylim(-15, 15)
    ax.set_aspect('equal')
    ax.grid(True)

    ax.plot(fixed[0], fixed[1], 'ro')
    target_plot, = ax.plot(target[0], target[1], 'rx', markersize=8)

    arm, = ax.plot([], [], 'o-', color='green', markersize=5, linewidth=2)

    x_s1, y_s1, x_s2, y_s2 = plotAngle(jointAngles)
    arm.set_data([0, x_s1, x_s2], [0, y_s1, y_s2])


    keys = set()
    step = 0.05

    def key_press(event):
        if event.key is not None:
            keys.add(event.key.lower())

    def key_release(event):
        if event.key is not None:
            keys.discard(event.key.lower())

    fig.canvas.mpl_connect("key_press_event", key_press)
    fig.canvas.mpl_connect("key_release_event", key_release)

    def update(frame):
        nonlocal jointAngles, minimal_publisher

        if '1' in keys:
            target[1] += step
        if '2' in keys:
            target[1] -= step
        if '3' in keys:
            target[0] -= step
        if '4' in keys:
            target[0] += step

        target_plot.set_data([target[0]], [target[1]])
        x_1, y_1, x_2, y_2 = plotAngle(jointAngles)

        err = np.array([target[0] - x_2, target[1] - y_2])

        j = np.array([
            [
                -(l_1 * ma.sin(jointAngles[0])) - (l_2 * ma.sin(jointAngles[0] + jointAngles[1])),
                -(l_2 * ma.sin(jointAngles[0] + jointAngles[1]))
            ],
            [
                (l_1 * ma.cos(jointAngles[0])) + (l_2 * ma.cos(jointAngles[0] + jointAngles[1])),
                (l_2 * ma.cos(jointAngles[0] + jointAngles[1]))
            ]
        ])
        deltaAngle = (j.T @ np.linalg.inv(j @ j.T + (l_a ** 2) * np.identity(2))) @ (a * err)
        jointAngles += deltaAngle
        
        minimal_publisher.send_joint_offsets(jointAngles)
        rclpy.spin_once(minimal_publisher)
        # Update arm plot
        x_3, y_3, x_4, y_4 = plotAngle(jointAngles)
        arm.set_data([0, x_3, x_4], [0, y_3, y_4])

        ax.set_title(f"step: {frame}   err: {np.linalg.norm(err):.4f}")
        return arm, target_plot

    ani = None

    def start(event):
        nonlocal ani
        if ani is not None:
            return

        button.label.set_text("Running")
        ani = FuncAnimation(fig, update, frames=3000, interval=40, repeat=False)
        fig.canvas.draw_idle()
    rclpy.init(args=None)
    
    minimal_publisher = MinimalPublisher()
    button_ax = fig.add_axes([0.4, 0.03, 0.2, 0.06])
    button = Button(button_ax, "Button")
    button.on_clicked(start)

    plt.show()


if __name__ == "__main__":
    main()