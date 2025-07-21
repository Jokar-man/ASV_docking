import cv2
import time
import math
import random
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 30)
        self.ax.set_ylim(0, 30)
        self.line, = self.ax.plot([], [], 'bo-', label='ASV Path')
        self.target_plot, = self.ax.plot([], [], 'g*', label='Target Poles')
        self.heading_arrow = None
        self.path_x = []
        self.path_y = []
        self.ax.grid(True)
        self.ax.legend()
        plt.title('Corrected Parallel Docking Simulation')

    def update(self, x, y, yaw, targets=None, detected_poles=None):
        self.path_x.append(x)
        self.path_y.append(y)
        self.line.set_data(self.path_x, self.path_y)

        if targets:
            tx = [t[0] for t in targets]
            ty = [t[1] for t in targets]
            self.target_plot.set_data(tx, ty)

        if self.heading_arrow:
            self.heading_arrow.remove()

        arrow_length = 1.5
        dx = arrow_length * math.cos(yaw)
        dy = arrow_length * math.sin(yaw)
        self.heading_arrow = self.ax.quiver(x, y, dx, dy, color='blue',
                                            scale_units='xy', scale=1, width=0.01)

        if detected_poles:
            if hasattr(self, 'current_detected_poles_plot'):
                for p in self.current_detected_poles_plot:
                    p.remove()
            self.current_detected_poles_plot = []
            for (px, py) in detected_poles:
                p, = self.ax.plot(px, py, 'ro', markersize=8)
                self.current_detected_poles_plot.append(p)

        self.fig.canvas.draw()
        plt.pause(0.01)

class ASVSimulator:
    def __init__(self):
        self.sim_x = 15.0
        self.sim_y = 0.0
        self.sim_yaw = math.radians(90)
        self.visualizer = Visualizer()
        self.detected_poles_coords = None

        self.start_point = (15, 0)
        self.target_point = (5, 25)
        self.red_green_entry = (15, 10)
        self.targets = [self.target_point]
        self.visualizer.ax.plot(*zip(*self.targets), 'go')
        self.visualizer.ax.legend()

    def update_plot(self):
        self.visualizer.update(self.sim_x, self.sim_y, self.sim_yaw,
                               targets=self.targets, detected_poles=self.detected_poles_coords)

    def read_gps(self):
        return self.sim_x, self.sim_y, self.sim_yaw

    def simulate_cv_detection(self):
        if random.random() > 0.3:
            print("CV Detection: Success")
            return [(11.87, 20.0), (16.0, 20.0)]
        else:
            print("CV Detection: Failed")
            return None

    def stop(self):
        time.sleep(0.5)

    def turn_to(self, angle_deg):
        target_yaw = math.radians(angle_deg)
        print(f"Turning to {angle_deg:.2f}°...")
        while True:
            yaw_error = (target_yaw - self.sim_yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(yaw_error) < math.radians(2):
                self.sim_yaw = target_yaw
                self.update_plot()
                break
            self.sim_yaw += math.copysign(math.radians(5), yaw_error)
            self.update_plot()
            time.sleep(0.05)
        self.stop()

    def move_straight(self, distance):
        print(f"Moving straight {distance:.2f} m...")
        steps = int(abs(distance) / 0.1)
        direction = 1 if distance >= 0 else -1
        for _ in range(steps):
            self.sim_x += direction * 0.1 * math.cos(self.sim_yaw)
            self.sim_y += direction * 0.1 * math.sin(self.sim_yaw)
            self.update_plot()
            time.sleep(0.05)
        rem = abs(distance) - steps * 0.1
        self.sim_x += direction * rem * math.cos(self.sim_yaw)
        self.sim_y += direction * rem * math.sin(self.sim_yaw)
        self.update_plot()
        self.stop()

    def move_reverse(self, distance):
        print(f"Reversing {distance:.2f} m...")
        self.move_straight(-distance)

    def move_to_right_angle(self, target):
        tx, ty = target
        dx = tx - self.sim_x
        dy = ty - self.sim_y

        if abs(dx) > 0.1:
            self.turn_to(0 if dx > 0 else 180)
            self.move_straight(abs(dx))

        if abs(dy) > 0.1:
            self.turn_to(90 if dy > 0 else 270)
            self.move_straight(abs(dy))

    def parallel_docking_procedure(self):
        print("\n--- Starting Parallel Docking with Midpoint Approach ---")

        # Step a: Move through red-green poles
        self.move_to_right_angle(self.red_green_entry)

        # Step b: Detect or fallback docking poles
        poles = self.simulate_cv_detection()
        if poles:
            self.detected_poles_coords = poles
        else:
            poles = [(11.87, 20.0), (16.0, 20.0)]
            self.detected_poles_coords = poles

        self.update_plot()

        # Step b (continued): Compute midpoint of poles
        x1, y1 = poles[0]
        x2, y2 = poles[1]
        midpoint_x = (x1 + x2) / 2
        approach_y = y1
        approach_point = (midpoint_x, approach_y)
        self.move_to_right_angle(approach_point)

        # Step c: Turn 210° and reverse to align stern with poles
        self.turn_to(210)
        self.move_reverse(2.0)

        # Step d: Rotate to 180° (align parallel to dock)
        self.turn_to(180)

        # Step e: Turn to 210° and move forward to exit
        self.turn_to(210)
        self.move_straight(4.0)

        # Step f: Turn to 90°, head to final target
        # self.turn_to(90)
        self.move_to_right_angle(self.target_point)

    def run(self):
        print("ASV Simulation Started")
        self.parallel_docking_procedure()
        print("Docking Complete. Mission Success.")
        plt.show()

if __name__ == '__main__':
    sim = ASVSimulator()
    try:
        sim.run()
    except KeyboardInterrupt:
        print("Simulation Interrupted.")
    finally:
        plt.close(sim.visualizer.fig)
