import sys
import json
import os
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D
import heapq
import math
import warnings
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler

warnings.filterwarnings('ignore')


def report_progress(percent, message="İşleniyor..."):
    print(json.dumps({"type": "progress", "value": percent, "msg": message}), flush=True)


class Drone3D:
    def __init__(self, start_position, end_position, drone_mass=1.5, max_speed=4.0, time_interval=0.1):
        self.current_position = np.array(start_position, dtype=float)
        self.current_velocity = np.zeros(3)
        self.destination = np.array(end_position, dtype=float)
        self.drone_mass = drone_mass
        self.maximum_speed = max_speed
        self.time_interval = time_interval
        self.position_log = [self.current_position.copy()]
        self.energy_usage = 0.0
        self.is_crashed = False 
    
    def move_drone(self, target_position, is_final_target=False):
        if self.is_crashed: return 

        direction_vector = target_position - self.current_position
        distance = np.linalg.norm(direction_vector)
        if distance > 1e-6:
            normalized_direction = direction_vector / distance
        else:
            normalized_direction = np.zeros(3)
        slowing_distance = 1.0 if not is_final_target else 4.0
        if distance < slowing_distance:
            desired_speed = self.maximum_speed * (distance / slowing_distance)
            desired_speed = max(desired_speed, 1.0) 
        else:
            desired_speed = self.maximum_speed
            
        desired_velocity = normalized_direction * desired_speed
        steering_force = desired_velocity - self.current_velocity
        max_thrust = 15.0
        steering_norm = np.linalg.norm(steering_force)
        if steering_norm > max_thrust:
            steering_force = (steering_force / steering_norm) * max_thrust
            
        acc = steering_force / self.drone_mass
        self.current_velocity += acc * self.time_interval
        
        current_speed = np.linalg.norm(self.current_velocity)
        if current_speed > self.maximum_speed:
            self.current_velocity = (self.current_velocity / current_speed) * self.maximum_speed
            current_speed = self.maximum_speed
            
        self.current_position += self.current_velocity * self.time_interval
        self.energy_usage += 0.5 * self.drone_mass * current_speed**2 * self.time_interval
        self.position_log.append(self.current_position.copy())

    def reached_destination(self, distance_threshold=0.8):
        return np.linalg.norm(self.current_position - self.destination) < distance_threshold


class ObstacleField:
    def __init__(self, world_size):
        self.world_size = world_size
        self.blocked_positions = set()
        self.spherical_obstacles = []  
    
    def place_round_obstacle(self, obstacle_center, obstacle_radius):
        cx, cy, cz = obstacle_center
        self.spherical_obstacles.append((cx, cy, cz, obstacle_radius))
        logical_radius = obstacle_radius + 1.2 
        min_x, max_x = int(cx - logical_radius), int(cx + logical_radius)
        min_y, max_y = int(cy - logical_radius), int(cy + logical_radius)
        min_z, max_z = int(cz - logical_radius), int(cz + logical_radius)
        for x_pos in range(max(0, min_x), min(self.world_size[0], max_x) + 1):
            for y_pos in range(max(0, min_y), min(self.world_size[1], max_y) + 1):
                for z_pos in range(max(0, min_z), min(self.world_size[2], max_z) + 1):
                    if math.sqrt((x_pos-cx)**2 + (y_pos-cy)**2 + (z_pos-cz)**2) <= logical_radius:
                        self.blocked_positions.add((x_pos, y_pos, z_pos))
                        
    def check_position_free(self, test_coordinates):
        x, y, z = int(round(test_coordinates[0])), int(round(test_coordinates[1])), int(round(test_coordinates[2]))
        return ((x, y, z) not in self.blocked_positions) and (0 <= x <= self.world_size[0] and 0 <= y <= self.world_size[1] and 0 <= z <= self.world_size[2])

    def get_danger_level(self, x_pos, y_pos, z_pos, danger_radius=10.0):
        max_danger = 0.0
        for (cx, cy, cz, r) in self.spherical_obstacles:
            dist = math.sqrt((x_pos-cx)**2 + (y_pos-cy)**2 + (z_pos-cz)**2)
            dist_to_surface = max(0.1, dist - (r + 1.2))
            if dist_to_surface < danger_radius:
                danger = (danger_radius / dist_to_surface) ** 2
                if danger > max_danger: max_danger = danger
        return max_danger

    def check_precise_collision(self, position):
        x, y, z = position
        for (cx, cy, cz, r) in self.spherical_obstacles:
            dist = math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)

            if dist <= r: 
                return True
        return False

class SafePathFinder:
    def __init__(self, navigation_map, safety_factor=20.0):
        self.navigation_map = navigation_map
        self.safety_factor = safety_factor
        self.movement_vectors = [(dx,dy,dz) for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1] if not (dx==0 and dy==0 and dz==0)]
    def estimate_distance(self, point_a, point_b):
        return math.sqrt(sum((point_a[i] - point_b[i])**2 for i in range(3)))
    def find_safe_route(self, start_location, end_location):
        start_point = tuple(int(coord) for coord in start_location)
        end_point = tuple(int(coord) for coord in end_location)
        search_queue = [(0, start_point)]
        path_history = {start_point: None}
        travel_cost = {start_point: 0}
        while search_queue:
            _, current_location = heapq.heappop(search_queue)
            if current_location == end_point:
                route_list = []
                curr = end_point
                while curr:
                    route_list.append(np.array(curr, dtype=float))
                    curr = path_history[curr]
                route_list.reverse()
                return route_list
            for dx, dy, dz in self.movement_vectors:
                next_loc = (current_location[0]+dx, current_location[1]+dy, current_location[2]+dz)
                if not self.navigation_map.check_position_free(next_loc): continue
                move_dist = math.sqrt(dx**2 + dy**2 + dz**2)
                danger = self.navigation_map.get_danger_level(*next_loc) * self.safety_factor
                total_cost = travel_cost[current_location] + move_dist + danger
                if next_loc not in travel_cost or total_cost < travel_cost[next_loc]:
                    travel_cost[next_loc] = total_cost
                    est = total_cost + self.estimate_distance(next_loc, end_point)
                    heapq.heappush(search_queue, (est, next_loc))
                    path_history[next_loc] = current_location
        return []

class FastPathFinder:
    def __init__(self, navigation_map, speed_factor=0.0):
        self.navigation_map = navigation_map
        self.movement_vectors = [(dx,dy,dz) for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1] if not (dx==0 and dy==0 and dz==0)]
    def estimate_distance(self, point_a, point_b):
        return math.sqrt(sum((point_a[i] - point_b[i])**2 for i in range(3)))
    def find_quick_path(self, start_location, end_location):
        start_point = tuple(int(coord) for coord in start_location)
        end_point = tuple(int(coord) for coord in end_location)
        search_queue = [(0, start_point)]
        path_history = {start_point: None}
        travel_cost = {start_point: 0}
        while search_queue:
            _, current_location = heapq.heappop(search_queue)
            if current_location == end_point:
                route_list = []
                curr = end_point
                while curr:
                    route_list.append(np.array(curr, dtype=float))
                    curr = path_history[curr]
                route_list.reverse()
                return route_list
            for dx, dy, dz in self.movement_vectors:
                next_loc = (current_location[0]+dx, current_location[1]+dy, current_location[2]+dz)
                if not self.navigation_map.check_position_free(next_loc): continue
                move_dist = math.sqrt(dx**2 + dy**2 + dz**2)
                total_cost = travel_cost[current_location] + move_dist
                if next_loc not in travel_cost or total_cost < travel_cost[next_loc]:
                    travel_cost[next_loc] = total_cost
                    est = total_cost + self.estimate_distance(next_loc, end_point) * 2.0
                    heapq.heappush(search_queue, (est, next_loc))
                    path_history[next_loc] = current_location
        return []

class FlightCoordinator:
    def get_simulation_data(self):
       
        return {
            "obstacles": [
                {"x": o[0], "y": o[1], "z": o[2], "r": o[3]} 
                for o in self.environment_map.spherical_obstacles
            ],
            "safe_path": [p.tolist() for p in self.drone_safe.position_log],
            "fast_path": [p.tolist() for p in self.drone_fast.position_log],
            "results": self.flight_results
        }


        data = coord.get_simulation_data()
        result = {
            "type": "result",
            "status": "success",
            "data": data,
            "png_url": f"/outputs/{png_name}" 
        }
        print(json.dumps(result), flush=True)

    def __init__(self, world_dimensions=(50,50,50)):
        self.world_dimensions = world_dimensions
        self.environment_map = None
        self.drone_safe = None
        self.drone_fast = None
        self.safe_route = []
        self.fast_route = []
        self.flight_results = {'safe_path': {'time':0,'energy':0,'path_len':0}, 'fast_path': {'time':0,'energy':0,'path_len':0}}
    
    def setup_environment(self, start_coords, target_coords):
        default_obstacles = [
            ((15, 15, 15), 4), ((25, 20, 25), 5), ((35, 30, 20), 4.5),
            ((20, 40, 30), 4), ((30, 25, 40), 5),
        ]
        self.environment_map = ObstacleField(self.world_dimensions)
        for center_point, radius_size in default_obstacles:
            self.environment_map.place_round_obstacle(center_point, radius_size)
        return start_coords, target_coords

    def check_line_of_sight(self, p1, p2, env_map, safety_margin=0.0):
        dist = np.linalg.norm(p2 - p1)
        if dist == 0: return True
        steps = int(dist * 5) + 1
        for i in range(1, steps):
            t = i / steps
            cx, cy, cz = p1 + t * (p2 - p1)
            margin_int = math.ceil(max(0.1, safety_margin))
            for dx in range(-margin_int, margin_int + 1):
                for dy in range(-margin_int, margin_int + 1):
                    for dz in range(-margin_int, margin_int + 1):
                        if math.sqrt(dx**2 + dy**2 + dz**2) <= safety_margin:
                            test_pos = (round(cx+dx), round(cy+dy), round(cz+dz))
                            if not env_map.check_position_free(test_pos): return False
        return True

    def smooth_path(self, path, env_map, safety_margin):
        if not path or len(path) <= 2: return path
        smoothed = [path[0]]
        current_idx = 0
        while current_idx < len(path) - 1:
            furthest = current_idx + 1
            for next_idx in range(len(path)-1, current_idx, -1):
                if self.check_line_of_sight(path[current_idx], path[next_idx], env_map, safety_margin):
                    furthest = next_idx
                    break
            smoothed.append(path[furthest])
            current_idx = furthest
        return smoothed
    
    def plan_navigation_routes(self, start_coords, target_coords):
        start_int = tuple(int(coord) for coord in start_coords)
        target_int = tuple(int(coord) for coord in target_coords)
        if start_int in self.environment_map.blocked_positions: self.environment_map.blocked_positions.remove(start_int)
        if target_int in self.environment_map.blocked_positions: self.environment_map.blocked_positions.remove(target_int)
        
        safe_finder = SafePathFinder(self.environment_map, safety_factor=20.0)
        raw_safe_route = safe_finder.find_safe_route(start_coords, target_coords)
        self.safe_route = self.smooth_path(raw_safe_route, self.environment_map, safety_margin=2.5)
        
        fast_finder = FastPathFinder(self.environment_map, speed_factor=0.0)
        raw_fast_route = fast_finder.find_quick_path(start_coords, target_coords)
        self.fast_route = self.smooth_path(raw_fast_route, self.environment_map, safety_margin=0.5)
        
        if not self.safe_route or not self.fast_route: return False
        return True
    
    def execute_flight_simulation(self, start_coords, target_coords):
        self.drone_safe = Drone3D(start_coords, target_coords, drone_mass=2.5, max_speed=2.5)
        t0 = time.time()
        self.simulate_drone_flight(self.drone_safe, self.safe_route)
        t1 = time.time()
        self.flight_results['safe_path']['time'] = t1 - t0
        self.flight_results['safe_path']['energy'] = self.drone_safe.energy_usage
        self.flight_results['safe_path']['path_len'] = len(self.drone_safe.position_log)
        
        self.drone_fast = Drone3D(start_coords, target_coords, drone_mass=1.0, max_speed=6.0)
        t0 = time.time()
        self.simulate_drone_flight(self.drone_fast, self.fast_route)
        t1 = time.time()
        self.flight_results['fast_path']['time'] = t1 - t0
        self.flight_results['fast_path']['energy'] = self.drone_fast.energy_usage
        self.flight_results['fast_path']['path_len'] = len(self.drone_fast.position_log)
    
    def simulate_drone_flight(self, drone_object, waypoint_list, max_steps=3000):
        waypoint_index = 0
        step_counter = 0
        
        while waypoint_index < len(waypoint_list) and step_counter < max_steps:
            prev_pos = drone_object.current_position.copy()

            target_waypoint = waypoint_list[waypoint_index]
            direction_vector = target_waypoint - drone_object.current_position
            distance = np.linalg.norm(direction_vector)
            current_speed = np.linalg.norm(drone_object.current_velocity)
            dynamic_threshold = max(1.5, current_speed * drone_object.time_interval * 2.5)
            
            if distance < dynamic_threshold:
                waypoint_index += 1
                if waypoint_index >= len(waypoint_list): break
                continue
                
            is_final = (waypoint_index == len(waypoint_list) - 1)
            drone_object.move_drone(target_waypoint, is_final_target=is_final)
            step_counter += 1

            if self.environment_map.check_precise_collision(drone_object.current_position):
                drone_object.is_crashed = True
                break

        while not drone_object.is_crashed and not drone_object.reached_destination(0.8) and step_counter < max_steps:
            drone_object.move_drone(drone_object.destination, is_final_target=True)
            step_counter += 1
            if self.environment_map.check_precise_collision(drone_object.current_position):
                drone_object.is_crashed = True
                break

    def generate_animation(self, output_file):
        fig = plt.figure(figsize=(14, 8))
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122, projection='3d')
        
        obs_plots_ax1 = []
        obs_plots_ax2 = []
        for (cx, cy, cz, r) in self.environment_map.spherical_obstacles:
            u = np.linspace(0, 2*np.pi, 20)
            v = np.linspace(0, np.pi, 10)
            x = cx + r * np.outer(np.cos(u), np.sin(v))
            y = cy + r * np.outer(np.sin(u), np.sin(v))
            z = cz + r * np.outer(np.ones(np.size(u)), np.cos(v))
            
            surf1 = ax1.plot_surface(x, y, z, color='green', alpha=0.1, shade=True, antialiased=False)
            surf2 = ax2.plot_surface(x, y, z, color='green', alpha=0.1, shade=True, antialiased=False)
            obs_plots_ax1.append(surf1)
            obs_plots_ax2.append(surf2)

        title_safe = ax1.set_title('Güvenli Rota', fontsize=12, weight='bold', backgroundcolor='green', color='white')
        title_fast = ax2.set_title('Hızlı Rota', fontsize=12, weight='bold', backgroundcolor='green', color='white')
        
        for ax in [ax1, ax2]:
            ax.set_xlim(0, self.world_dimensions[0])
            ax.set_ylim(0, self.world_dimensions[1])
            ax.set_zlim(0, self.world_dimensions[2])
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        
        max_frames = max(len(self.drone_safe.position_log), len(self.drone_fast.position_log))
        
        l1, = ax1.plot([], [], [], color='blue', linewidth=2.0, alpha=0.8, label='Rota')
        p1, = ax1.plot([], [], [], marker='o', color='red', markeredgecolor='black', markersize=8, label='İHA')
        
        l2, = ax2.plot([], [], [], color='blue', linewidth=2.0, alpha=0.8, label='Rota')
        p2, = ax2.plot([], [], [], marker='o', color='red', markeredgecolor='black', markersize=8, label='İHA')
        
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')

        def get_obstacle_color_rgba(obs_center, obs_radius, drone_pos_1, drone_pos_2):
            x1, y1, z1 = drone_pos_1
            x2, y2, z2 = drone_pos_2
            cx, cy, cz = obs_center
            dist1 = math.sqrt((x1-cx)**2 + (y1-cy)**2 + (z1-cz)**2) - obs_radius
            dist2 = math.sqrt((x2-cx)**2 + (y2-cy)**2 + (z2-cz)**2) - obs_radius
            min_dist = min(dist1, dist2)

            if min_dist <= 0.2: return (1.0, 0.0, 0.0, 0.6) 
            elif min_dist <= 4.0: return (1.0, 0.84, 0.0, 0.3) 
            else: return (0.0, 0.5, 0.0, 0.1) 

        def get_title_status(drone_object, drone_pos):
            if drone_object.is_crashed: 
                 return 'red', 'KAZA YAPILDI! SİMÜLASYON DURDU.'
            
            x, y, z = drone_pos
            min_dist = float('inf')
            for (cx, cy, cz, r) in self.environment_map.spherical_obstacles:
                dist = math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)
                dist_to_surface = dist - r
                if dist_to_surface < min_dist: min_dist = dist_to_surface
            
            if min_dist <= 0.1: return 'red', 'KIRMIZI: ÇARPIŞMA!'
            elif min_dist <= 3.0: return 'gold', 'SARI: DİKKAT!'
            else: return 'green', 'YEŞİL: GÜVENLİ'
        
        def animate_frame(frame):
            safe_frame_idx = min(frame, len(self.drone_safe.position_log)-1)
            current_safe = self.drone_safe.position_log[safe_frame_idx]
            
            if frame < len(self.drone_safe.position_log) + 10:
                history = np.array(self.drone_safe.position_log[:safe_frame_idx+1])
                l1.set_data(history[:,0], history[:,1]); l1.set_3d_properties(history[:,2])
                p1.set_data([current_safe[0]], [current_safe[1]]); p1.set_3d_properties([current_safe[2]])
                c, t = get_title_status(self.drone_safe, current_safe)
                title_safe.set_backgroundcolor(c); title_safe.set_text(f"Güvenli Rota\n[{t}]")

            fast_frame_idx = min(frame, len(self.drone_fast.position_log)-1)
            current_fast = self.drone_fast.position_log[fast_frame_idx]
            
            if frame < len(self.drone_fast.position_log) + 10:
                history = np.array(self.drone_fast.position_log[:fast_frame_idx+1])
                l2.set_data(history[:,0], history[:,1]); l2.set_3d_properties(history[:,2])
                p2.set_data([current_fast[0]], [current_fast[1]]); p2.set_3d_properties([current_fast[2]])
                c, t = get_title_status(self.drone_fast, current_fast)
                title_fast.set_backgroundcolor(c); title_fast.set_text(f"Hızlı Rota\n[{t}]")

            obstacles_data = self.environment_map.spherical_obstacles
            for i, surf in enumerate(obs_plots_ax1):
                center = obstacles_data[i][:3]; radius = obstacles_data[i][3]
                rgba = get_obstacle_color_rgba(center, radius, current_safe, current_fast)
                surf.set_facecolor(rgba)
            for i, surf in enumerate(obs_plots_ax2):
                center = obstacles_data[i][:3]; radius = obstacles_data[i][3]
                rgba = get_obstacle_color_rgba(center, radius, current_safe, current_fast)
                surf.set_facecolor(rgba)

            return l1, p1, l2, p2, title_safe, title_fast
        
        anim = FuncAnimation(fig, animate_frame, frames=range(0, max_frames + 15, 2), blit=False)
        writer = PillowWriter(fps=20)
        anim.save(output_file, writer=writer)
        plt.close()
    
    def generate_report(self, output_file):
        fig, axes = plt.subplots(1, 2, figsize=(10, 5))
        safe, fast = self.flight_results['safe_path'], self.flight_results['fast_path']
        axes[0].bar(['Güvenli', 'Hızlı'], [safe['time'], fast['time']], color=['blue','red'])
        axes[0].set_title('Süre (s)')
        axes[1].bar(['Güvenli', 'Hızlı'], [safe['energy'], fast['energy']], color=['blue','red'])
        axes[1].set_title('Enerji (J)')
        plt.savefig(output_file); plt.close()

def main():
    try:
        report_progress(5, "Başlatılıyor...")
        
        if len(sys.argv) < 8:
            print(json.dumps({"type":"result", "status": "error", "message": "Eksik parametreler"}), flush=True)
            return

        sx, sy, sz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        tx, ty, tz = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
        
        coord = FlightCoordinator()
        
        report_progress(20, "Ortam ve Engeller Kuruluyor...")
        coord.setup_environment((sx,sy,sz), (tx,ty,tz))
        
        report_progress(40, "A* Algoritması ile Rota Hesaplanıyor...")
        if not coord.plan_navigation_routes((sx,sy,sz), (tx,ty,tz)):
            print(json.dumps({"type":"result", "status": "error", "message": "Rota Bulunamadı"}), flush=True)
            return
            
        report_progress(60, "Fizik Simülasyonu Çalıştırılıyor...")
        coord.execute_flight_simulation((sx,sy,sz), (tx,ty,tz))
        
        report_progress(80, "3D Animasyon Render Ediliyor...")
        output_dir = os.path.join(os.path.dirname(__file__), 'public', 'outputs')
        if not os.path.exists(output_dir): os.makedirs(output_dir)
        
        ts = int(time.time())
        gif_name, png_name = f"sim_{ts}.gif", f"report_{ts}.png"
        
        coord.generate_animation(os.path.join(output_dir, gif_name))
        
        report_progress(95, "Raporlar Hazırlanıyor...")
        coord.generate_report(os.path.join(output_dir, png_name))
        
        result = {
            "type": "result",
            "status": "success",
            "gif_url": f"/outputs/{gif_name}",
            "png_url": f"/outputs/{png_name}"
        }
        print(json.dumps(result), flush=True)
        
    except Exception as e:
        print(json.dumps({"type":"result", "status": "error", "message": str(e)}), flush=True)

if __name__ == '__main__':
    main()