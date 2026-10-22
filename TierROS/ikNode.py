import numpy as np
import matplotlib as mpl 
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import math as ma
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import UInt8MultiArray  
    

s1d = 1
s2d = 1 
s1o = 90 
s2o = 90 

l_1 = 4.764
l_2 = 4.773
fixed = np.array([0.0, 0.0])

a = 0.02
l_a = 0.05


offset1 = -10 # Axis 1 
offset2 = 175 # Axis 2 
offset3 = 0 # wrist 
offset4 = 0  # claw degrees

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, '/serial_write', 10)

    def send_joint_offsets(self, absolute, wrist, claw, motorDir, motorSpeed):
        msg = UInt8MultiArray() 

        delta_deg = np.degrees(np.asarray(absolute, dtype=float))
        offsets = np.rint(delta_deg).astype(int) # Rounds the interger cause w`e can't all that 
        command = f"A,{motorSpeed},{motorDir},{offsets[0]+offset1},{offsets[1]+offset2},{np.rint(np.degrees(wrist))+offset3},{claw+offset4}\n"
        msg.data = command.encode("ascii")
        
        self.get_logger().info(command)
        self.publisher_.publish(msg)
        

def plotAngle(angles):
    x1 = l_1 * ma.cos(angles[0])
    y1 = l_1 * ma.sin(angles[0])

    x2 = l_1 * ma.cos(angles[0]) + l_2 * ma.cos(angles[0] + angles[1])
    y2 = l_1 * ma.sin(angles[0]) + l_2 * ma.sin(angles[0] + angles[1])

    return x1, y1, x2, y2

def plot_wrist(wrist_angle, claw):
    x0 = 10
    y0 = -10

    ray_length = 5

    x1 = x0 + ray_length * np.cos(wrist_angle)
    y1 = y0 + ray_length * np.sin(wrist_angle)

    cx = -10
    cy = -10

    claw_half = claw / 50

    dx = np.cos(0) * claw_half

    claw_x = [cx - dx, cx + dx]
    claw_y = [cy, cy]

    return [x0, x1], [y0, y1], claw_x, claw_y

def main():
    motorSpeed = 0 
    motorDir = 2 # 0 is left 1 is right 2 is stop 
    wrist = np.pi/2 
    claw = 0 
    for key in mpl.rcParams:
        if key.startswith('keymap.'):
            mpl.rcParams[key] = []
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
    wrist_ray, = ax.plot([], [], 'b-', linewidth=2)
    claw_line, = ax.plot([], [], 'r-', linewidth=3)

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
        nonlocal jointAngles, wrist, claw, motorDir, motorSpeed
        move = np.array([0.0, 0.0])
        if 'w' in keys:
            move[1] += step
        if 's' in keys:
            move[1] -= step
        if 'a' in keys:
            move[0] -= step
        if 'd' in keys:
            move[0] += step
        new_target = target + move
        if np.linalg.norm(new_target) <= 9:
            target[:] = new_target
        if 'i' in keys: 
            wrist += step
        if 'k' in keys: 
            wrist -= step
        if 'n' in keys: 
            claw = min(180, claw + 3)
        if 'm' in keys: 
            claw = max(0, claw - 3)
        if ',' in keys: 
            claw = min(180, claw + 5)
        if '.' in keys: 
            claw = max(0, claw - 5)
        if 'j' in keys: 
            motorSpeed = 150 
            motorDir = 0 
        elif 'l' in keys: 
            motorSpeed = 150 
            motorDir = 1 
        else: 
            motorSpeed = 0 
            motorDir = 2  

        target_plot.set_data([target[0]], [target[1]])
        x_1, y_1, x_2, y_2 = plotAngle(jointAngles)
        ray_x, ray_y, claw_x, claw_y = plot_wrist(wrist, claw)

        wrist_ray.set_data(ray_x, ray_y)
        claw_line.set_data(claw_x, claw_y)

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
        
        minimal_publisher.send_joint_offsets(jointAngles, wrist, claw, motorDir, motorSpeed)
       # rclpy.spin_once(minimal_publisher)
        # Update arm plot
        x_3, y_3, x_4, y_4 = plotAngle(jointAngles)
        arm.set_data([0, x_3, x_4], [0, y_3, y_4])

        ax.set_title(f"step: {frame}   err: {np.linalg.norm(err):.4f}")
        return arm, target_plot, wrist_ray, claw_line

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
