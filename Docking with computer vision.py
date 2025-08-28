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
        self.line, = self.ax.plot([], [], 'bo-', label='Vehicle Path')
        self.target_plot, = self.ax.plot([], [], 'r*', label='Targets')
        self.heading_arrow = None
        self.path_x = []
        self.path_y = []
        self.ax.grid(True)
        self.ax.legend()
        plt.title('ASV Docking Simulation with CV Integration')

    def update(self, x, y, yaw, targets=None, detected_poles=None):
        self.path_x.append(x)
        self.path_y.append(y)
        self.line.set_data(self.path_x, self.path_y)
        
        if targets:
            target_x = [t[0] for t in targets]
            target_y = [t[1] for t in targets]
            self.target_plot.set_data(target_x, target_y)

        if self.heading_arrow:
            self.heading_arrow.remove()

        arrow_length = 1.5
        dx_arrow = arrow_length * math.cos(yaw)
        dy_arrow = arrow_length * math.sin(yaw)
        
        self.heading_arrow = self.ax.quiver(x, y, dx_arrow, dy_arrow, color='blue', 
                                           scale_units='xy', scale=1, width=0.01,
                                           headwidth=5, headlength=7)

        if detected_poles:
            if hasattr(self, 'current_detected_poles_plot'):
                for p in self.current_detected_poles_plot:
                    p.remove()
            self.current_detected_poles_plot = []
            for (px, py) in detected_poles:
                pole_point, = self.ax.plot(px, py, 'ro', markersize=8)
                self.current_detected_poles_plot.append(pole_point)


        self.fig.canvas.draw()
        plt.pause(0.01)


class ASVSimulator:
    def __init__(self):
        self.sim_x = 15.0
        self.sim_y = 0.0
        self.sim_yaw = math.radians(90)
        self.current_pose = (self.sim_x, self.sim_y, self.sim_yaw)
        self.path_x = []
        self.path_y = []
        self.detected_poles_coords = None
        self.visualizer = Visualizer()

        self.start_point = (15, 0)
        self.end_point = (5, 25)
        self.targets = [self.start_point, self.end_point]

        self.visualizer.ax.plot([self.start_point[0]], [self.start_point[1]], 'bo', label='Start (Point 9)')
        self.visualizer.ax.plot([self.end_point[0]], [self.end_point[1]], 'go', label='End (Point 10)')
        self.visualizer.ax.legend()

    def read_gps(self):
        self.current_pose = (self.sim_x, self.sim_y, self.sim_yaw)
        return self.current_pose

    def update_plot(self):
        self.visualizer.update(self.sim_x, self.sim_y, self.sim_yaw, 
                               targets=self.targets, 
                               detected_poles=self.detected_poles_coords)

    def simulate_cv_detection(self):
        
        if random.random() > 0.2:
            print("Computer vision successfully detected poles (simulated).")

            return [(9.0, 20.0), (11.0, 20.0)] 
        else:
            print("Computer vision failed to detect poles (simulated).")
            return None
        
        
    def stop(self):
        time.sleep(0.5)

    def turn_to(self, angle_deg):
        target_yaw = math.radians(angle_deg)
        print(f"Turning to {angle_deg:.2f} degrees...")
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
        print(f"Turn complete. Current yaw: {math.degrees(self.sim_yaw):.2f} degrees.")

    def move_straight(self, distance):
        if distance < 0.05:
            print(f"Skipping move_straight for negligible distance: {distance:.2f}m")
            return
        print(f"Moving straight for {distance:.2f} meters...")
        vx = math.cos(self.sim_yaw)
        vy = math.sin(self.sim_yaw)
        steps = int(distance / 0.1)
        for i in range(steps):
            self.sim_x += vx * 0.1
            self.sim_y += vy * 0.1
            self.update_plot()
            time.sleep(0.05)
        remaining_dist = distance - (steps * 0.1)
        self.sim_x += vx * remaining_dist
        self.sim_y += vy * remaining_dist
        self.update_plot()
        self.stop()
        print(f"Move straight complete. Current Pos: ({self.sim_x:.2f}, {self.sim_y:.2f})")

    def move_to_right_angle(self, target):
        print(f"Moving to target {target} using right angle turns...")
        start_x, start_y, _ = self.read_gps()
        
        dx = target[0] - self.sim_x
        dy = target[1] - self.sim_y

        if abs(dx) > 0.1:
            target_heading_deg = 0 if dx > 0 else 180
            self.turn_to(target_heading_deg)
            self.move_straight(abs(dx))

        if abs(dy) > 0.1:
            target_heading_deg = 90 if dy > 0 else 270
            self.turn_to(target_heading_deg)
            self.move_straight(abs(dy))
        print(f"Reached target {target}.")

    def move_reverse(self, distance):
        if distance < 0.05:
            print(f"Skipping move_reverse for negligible distance: {distance:.2f}m")
            return
        print(f"Reversing for {distance:.2f} meters...")
        vx = -math.cos(self.sim_yaw)
        vy = -math.sin(self.sim_yaw)
        steps = int(distance / 0.1)
        for i in range(steps):
            self.sim_x += vx * 0.1
            self.sim_y += vy * 0.1
            self.update_plot()
            time.sleep(0.05)
        remaining_dist = distance - (steps * 0.1)
        self.sim_x += vx * remaining_dist
        self.sim_y += vy * remaining_dist
        self.update_plot()
        self.stop()
        print(f"Reverse complete. Current Pos: ({self.sim_x:.2f}, {self.sim_y:.2f})")

    def run(self):
        print("Starting ASV Docking Simulation.")

        staging_point = (15, 18)
        print(f"\n--- Phase 1: Moving to staging point {staging_point} ---")
        self.move_to_right_angle(staging_point)
        self.read_gps()

        print("\n--- Phase 2: Performing CV detection ---")
        poles = self.simulate_cv_detection()
        if poles:
            print("Poles detected by computer vision. Using detected coordinates.")
            self.detected_poles_coords = poles
            x1, y1 = poles[0]
            x2, y2 = poles[1]
        else:
            print("Computer vision failed. Using fallback coordinates for poles.")
            x1, y1 = 9.0, 20.0
            x2, y2 = 11.0, 20.0
            self.detected_poles_coords = [(x1, y1), (x2, y2)]
        
        self.update_plot()
        time.sleep(1)

        midpoint = ((x1 + x2) / 2, (y1 + y2) / 2)

        desired_parking_depth = 0.0 # meters, customizable

        approach_buffer_distance = 2.0 # meters, how far in front of the poles to stop

        pole_line_y = (y1 + y2) / 2 # Average Y of the poles
        berth_entry_y_coord = pole_line_y - approach_buffer_distance


        docking_y_coord = pole_line_y + desired_parking_depth

        berth_approach_point = (midpoint[0], berth_entry_y_coord)
        print(f"\n--- Phase 3: Moving to berth approach point {berth_approach_point} ---")
        self.move_to_right_angle(berth_approach_point)
        self.read_gps()

        print("\n--- Phase 4: Turning to face dock and entering ---")
        self.turn_to(90) # Face North (assuming dock is North-South aligned)
        

        distance_to_dock_inside = docking_y_coord - self.sim_y
        
]        if distance_to_dock_inside < 0: # This means current_y is already past docking_y_coord
            print(f"Warning: Already past desired docking Y. Adjusting distance to 0. Current Y: {self.sim_y:.2f}, Docking Y: {docking_y_coord:.2f}")
            distance_to_dock_inside = 0
        
        self.move_straight(distance_to_dock_inside)
        self.read_gps()
        print("ASV is now docked.")
        time.sleep(2)

        print("\n--- Phase 5: Reversing out of the dock ---")
        distance_to_reverse_out = self.sim_y - berth_entry_y_coord
        
        # Ensure distance is positive for reversing
        if distance_to_reverse_out < 0:
            print(f"Warning: Reverse distance calculated as negative. Adjusting to 0. Current Y: {self.sim_y:.2f}, Berth Entry Y: {berth_entry_y_coord:.2f}")
            distance_to_reverse_out = 0

        self.move_reverse(distance_to_reverse_out)
        self.read_gps()

        print(f"\n--- Phase 6: Moving to final destination {self.end_point} ---")
        self.move_to_right_angle(self.end_point)
        self.read_gps()

        print("\nMission complete. ASV has reached the final destination.")
        plt.show()

if __name__ == '__main__':
    sim = ASVSimulator()
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:

        plt.close(sim.visualizer.fig)
