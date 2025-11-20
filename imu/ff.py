"""
continuum_sim_from_serial_WITH_LOGGING.py

This module provides a live simulation and animation of a continuum robot
using PyVista, driven by serial data.

It now *also* logs all incoming data (Timestamp, Kappa, Phi, X, Y, Z)
to a timestamped CSV file for later analysis.

Dependencies:
- numpy
- pyvista
- vtk
- os
- pyserial (pip install pyserial)
- csv (standard library)
- datetime (standard library)

Author: Motaz Elsaman
Date: 2024-04-28
(Modified for live serial data by Gemini: 2025-11-13)
(Fixed parsing and animation loop by Gemini: 2025-11-13)
(Added CSV logging by Gemini: 2025-11-13)
"""

import numpy as np
import pyvista as pv
import vtk
import os
import serial
import time
import pandas as pd # Keep for np/pd compatibility if needed
import csv                # --- NEW: For saving data
from datetime import datetime # --- NEW: For timestamping file

class ContinuumSegment:
    """
    Represents a single segment of a continuum robot.
    (Class is unchanged)
    """
    def __init__(self, length, radius=0.03, num_points=300):
        self.length = length
        self.radius = radius
        self.num_points = num_points
        if length <= 0:
            raise ValueError("Segment length must be positive.")
        if num_points < 2:
            raise ValueError("num_points must be at least 2.")

    @staticmethod
    def homogeneous_transform(s, kappa, phi):
        T = np.eye(4)
        if kappa > 1e-9:
            c = np.cos(kappa * s)
            s_val = np.sin(kappa * s)
            one_minus_c = 1 - c
            c_phi = np.cos(phi)
            s_phi = np.sin(phi)
            
            T[0, 0] = c_phi**2 * c + s_phi**2
            T[0, 1] = c_phi * s_phi * (c - 1)
            T[0, 2] = c_phi * s_val
            T[1, 0] = c_phi * s_phi * (c - 1)
            T[1, 1] = s_phi**2 * c + c_phi**2
            T[1, 2] = s_phi * s_val
            T[2, 0] = -c_phi * s_val
            T[2, 1] = -s_phi * s_val
            T[2, 2] = c
            T[0, 3] = c_phi * one_minus_c / kappa
            T[1, 3] = s_phi * one_minus_c / kappa
            T[2, 3] = s_val / kappa
        else:
            T[2, 3] = s
        return T

    def sample_curve(self, kappa, phi):
        s_vals = np.linspace(0, self.length, self.num_points)
        points = np.zeros((self.num_points, 3))
        for i, s in enumerate(s_vals):
            T = self.homogeneous_transform(s, kappa, phi)
            points[i, :] = T[:3, 3]
        return points

    @staticmethod
    def apply_global_transform(points, x=0, y=0, z=0):
        return points + np.array([x, y, z])

    @staticmethod
    def generate_shadow(points):
        shadow = points.copy()
        shadow[:, 2] = 0.001
        return shadow

    @staticmethod
    def create_shadow_mesh(shadow_points, radius=0.031):
        if shadow_points.shape[0] < 2 or np.allclose(shadow_points, shadow_points[0]):
            return None
        return pv.lines_from_points(shadow_points).tube(radius=radius)

class ContinuumRobot:
    """
    Represents a continuum robot composed of a *single* segment.
    (Class is unchanged)
    """
    def __init__(self, length, radius=0.03, base_height=0.0, base_pos=(0, 0), texture=None):
        self.segment = ContinuumSegment(length, radius=radius)
        self.base_height = base_height
        self.base_pos = base_pos
        self.texture = texture

    def get_segment_points(self, kappa, phi):
        seg_points = self.segment.sample_curve(kappa, phi)
        seg_points = ContinuumSegment.apply_global_transform(
            seg_points, *self.base_pos, self.base_height
        )
        return seg_points

    def get_shadow_mesh(self, seg_points):
        shadow = ContinuumSegment.generate_shadow(seg_points)
        shadow_mesh = ContinuumSegment.create_shadow_mesh(shadow, radius=self.segment.radius * 1.05)
        return shadow_mesh

    def get_tube_mesh(self, seg_points):
        if seg_points.shape[0] < 2:
            return None
        tube = pv.lines_from_points(seg_points).tube(self.segment.radius)
        if self.texture:
            tube.texture_map_to_plane(inplace=True)
        return tube

class ContinuumAnimation:
    """
    Handles the LIVE animation and visualization of a continuum robot
    based on data from a SERIAL PORT and logs data to a CSV file.
    """
    def __init__(self, robot1, port, baudrate=115200, robot2=None):
        
        self.robot1 = robot1
        self.robot2 = robot2
        self.port = port
        self.baudrate = baudrate
        self.serial_log = None
        
        # --- NEW: CSV File setup ---
        self.csv_file = None
        self.csv_writer = None
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.csv_filename = f"serial_data_log_{timestamp}.csv"
            
            # Note: 'newline=""' is required for the csv module
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write the header
            self.csv_writer.writerow(['Timestamp', 'Kappa', 'Phi', 'X', 'Y', 'Z'])
            print(f"Logging data to {self.csv_filename}")
            
        except Exception as e:
            print(f"Error: Could not open CSV file for writing: {e}")
            raise
        # ---------------------------

        try:
            self.serial_log = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connecting to serial port {self.port} at {self.baudrate}...")
            time.sleep(2) 
            print("Connection established.")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {self.port}: {e}")
            raise

        self.current_data = {
            'kappa': 0.0,
            'phi': 0.0,
            'x': 0.0,
            'y': 0.0,
            'z': self.robot1.segment.length
        }
        
        self._get_initial_data()
        
        self.robot_length = robot1.segment.length
        self.robot_radius = robot1.segment.radius
        radius_scale_factor = 80.0 
        self.env_size = self.robot_radius * radius_scale_factor
        self.z_max = self.robot_length * 1.5
        
        self.goal_center = (
            self.current_data['x'], 
            self.current_data['y'], 
            self.current_data['z']
        )
        self.goal_radius = self.robot_radius 

        self.plotter = pv.Plotter(notebook=False, window_size=[1024, 768])
        self.setup_scene()


    def _get_initial_data(self):
        """
        Helper to read the serial port until one of each *required* value is found.
        """
        print("Flushing serial buffer and waiting for initial data...")
        self.serial_log.flushInput()
        start_time = time.time()
        
        data_found = {k: False for k in self.current_data}
        
        while time.time() - start_time < 5.0:
            if all(data_found.values()):
                print("Initial data acquired.")
                print(f"  > Start State: {self.current_data}")
                return
            
            try:
                line_bytes = self.serial_log.readline()
                if not line_bytes:
                    continue
                    
                line = line_bytes.decode('utf-8').strip()
                if not line:
                    continue
                
                self._parse_serial_line(line, data_found)

            except (UnicodeDecodeError, ValueError) as e:
                print(f"  ... skipping corrupt data: {e}")
        
        if not all(data_found.values()):
            print(f"Warning: Failed to get all initial data. Using defaults.")
            print(f"  > Missing: {[k for k, v in data_found.items() if not v]}")
            print(f"  > Using: {self.current_data}")


    def _parse_serial_line(self, line, data_found_dict=None):
        """
        Parses a single line of labeled data (e.g., "Kappa: 10.0")
        and updates the self.current_data dictionary.
        """
        try:
            if ":" in line:
                parts = line.split(":", 1)
                key_str = parts[0].strip().lower()
                value_str = parts[1].strip()
                
                if key_str == 'kappa':
                    self.current_data['kappa'] = float(value_str)
                    if data_found_dict: data_found_dict['kappa'] = True
                elif key_str == 'phi':
                    self.current_data['phi'] = float(value_str)
                    if data_found_dict: data_found_dict['phi'] = True
                elif key_str == 'x':
                    self.current_data['x'] = float(value_str)
                    if data_found_dict: data_found_dict['x'] = True
                elif key_str == 'y':
                    self.current_data['y'] = float(value_str)
                    if data_found_dict: data_found_dict['y'] = True
                elif key_str == 'z':
                    self.current_data['z'] = float(value_str)
                    if data_found_dict: data_found_dict['z'] = True

        except ValueError:
            print(f"Warning: Could not parse value from line: {line}")
        except Exception as e:
            print(f"Warning: Error parsing line '{line}': {e}")


    def _read_serial_data(self):
        """
        Reads *all* available lines from the serial buffer and updates
        the self.current_data dictionary.
        Returns the most up-to-date (kappa, phi, x, y, z) tuple.
        """
        try:
            while self.serial_log.in_waiting > 0:
                line = self.serial_log.readline().decode('utf-8').strip()
                if line:
                    self._parse_serial_line(line)
        
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            self.plotter.close()
        except (UnicodeDecodeError, ValueError) as e:
            print(f"Warning: Skipping corrupt data ({e})")
            
        return (
            self.current_data['kappa'],
            self.current_data['phi'],
            self.current_data['x'],
            self.current_data['y'],
            self.current_data['z']
        )


    def setup_scene(self):
        """
        Set up the PyVista scene. (Unchanged)
        """
        self.plotter.background_color = "black"
        s_xy = self.env_size / 2.0
        
        self.plotter.show_grid(
            color='white', 
            grid='back', 
            location='outer',
            bounds=[-s_xy, s_xy, -s_xy, s_xy, 0, self.z_max]
        )
        
        axes_actor = vtk.vtkAxesActor()
        axes_actor.SetTotalLength(s_xy * 0.4, s_xy * 0.4, s_xy * 0.4) 
        for axis_prop in [axes_actor.GetXAxisShaftProperty(),
                          axes_actor.GetYAxisShaftProperty(),
                          axes_actor.GetZAxisShaftProperty(),
                          axes_actor.GetXAxisTipProperty(),
                          axes_actor.GetYAxisTipProperty(),
                          axes_actor.GetZAxisTipProperty()]:
            axis_prop.SetColor(1, 1, 1)
        self.plotter.add_actor(axes_actor)
        
        plane = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), 
                         i_size=self.env_size, j_size=self.env_size)
        self.plotter.add_mesh(plane, color='gray', ambient=0.2, name='ground')
        
        goal = pv.Sphere(radius=self.goal_radius, center=self.goal_center, 
                         theta_resolution=60, phi_resolution=60)
        self.plotter.add_mesh(goal, color='green', name='goal', specular=0.6, ambient=0.2)
        
        view_up = (0, 0, 1)
        focal_point = (0, 0, self.robot_length * 1)
        cam_pos = (-s_xy*2, -s_xy*2, self.z_max)
        
        self.plotter.camera_position = [cam_pos, focal_point, view_up]
        self.plotter.enable_parallel_projection()
        self.plotter.camera.parallel_scale = s_xy * 1.1

    def update_plot(self):
        """
        This function is called in a loop.
        It reads new data, LOGS IT, and updates the actors.
        """
        kappa1, phi1, goal_x, goal_y, goal_z = self._read_serial_data()
        current_goal_pos = (goal_x, goal_y, goal_z)
        
        # --- NEW: Save data to CSV ---
        try:
            log_time = time.time() # Get current timestamp (float)
            self.csv_writer.writerow([log_time, kappa1, phi1, goal_x, goal_y, goal_z])
        except Exception as e:
            # Don't crash the animation if logging fails
            print(f"Warning: Could not write to CSV: {e}")
        # -----------------------------

        # --- (Rest of the function is unchanged) ---
        seg1 = self.robot1.get_segment_points(kappa1, phi1)
        tube1 = self.robot1.get_tube_mesh(seg1)
        shadow1_mesh = self.robot1.get_shadow_mesh(seg1)

        if self.robot2:
            kappa1_r2 = kappa1
            phi1_r2 = phi1
            seg1_robot2 = self.robot2.get_segment_points(kappa1_r2, phi1_r2)
            tube1_robot2 = self.robot2.get_tube_mesh(seg1_robot2)
            shadow1_mesh_robot2 = self.robot2.get_shadow_mesh(seg1_robot2)

        for name in ['tube1', 'shadow1', 'tube1_robot2', 'shadow1_robot2', 'goal']:
            self.plotter.remove_actor(name, reset_camera=False)
        self.plotter.remove_legend()

        if tube1:
            self.plotter.add_mesh(tube1, name='tube1', texture=self.robot1.texture,
                                  color='red' if self.robot1.texture is None else None,
                                  ambient=0.1, specular=0.5,
                                  label=f'Robot 1: k={kappa1:.1f}, p={np.degrees(phi1):.0f}°')
        if shadow1_mesh:
            self.plotter.add_mesh(shadow1_mesh, name='shadow1', color='black', opacity=0.3)

        if self.robot2:
            if tube1_robot2:
                self.plotter.add_mesh(tube1_robot2, name='tube1_robot2', texture=self.robot2.texture,
                                      color='blue' if self.robot2.texture is None else None,
                                      ambient=0.1, specular=0.5,
                                      label=f'Robot 2: k={kappa1_r2:.1f}, p={np.degrees(phi1_r2):.0f}°')
            if shadow1_mesh_robot2:
                self.plotter.add_mesh(shadow1_mesh_robot2, name='shadow1_robot2', color='black', opacity=0.3)

        goal = pv.Sphere(radius=self.goal_radius, center=current_goal_pos,
                         theta_resolution=60, phi_resolution=60)
        self.plotter.add_mesh(goal, color='green', name='goal', specular=0.6, ambient=0.2)
        
        self.plotter.add_legend(bcolor=(0.9, 0.9, 0.9), size=[0.22, 0.15])


    def animate(self):
        """
        Run the LIVE animation loop and handle cleanup.
        """
        print("Starting live animation... Press 'q' in the window to quit.")
        
        self.plotter.show(interactive=False, auto_close=False)
        
        while self.plotter.renderer and self.plotter.renderer.GetRenderWindow():
            try:
                self.update_plot()
                self.plotter.render()
                self.plotter.iren.process_events()
                time.sleep(0.016) # ~60 FPS

            except Exception as e:
                print(f"Render loop exiting (window closed or error): {e}")
                break # Exit the while loop
        
        # --- Cleanup after the window is closed ---
        if self.serial_log and self.serial_log.is_open:
            self.serial_log.close()
            print("Serial port closed.")

        # --- NEW: Close CSV file ---
        if self.csv_file:
            self.csv_file.close()
            print(f"Data saved to {self.csv_filename}")
        # ---------------------------

        print("Animation stopped.")


# --- Main execution block ---

# Parameters for the robots and animation
robot_length = 0.14  # Single segment length
robot_radius = 0.03  # Robot's radius
base_height = 0.0

# --- Define Serial Port and Baud Rate ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200 
# --------------------------------------------

# Instantiate robot
robot1 = ContinuumRobot(
    length=robot_length, 
    radius=robot_radius,
    base_height=base_height, 
    base_pos=(0, 0),
)

# Create and run the animation
try:
    # This needs to be in scope for the 'except' block
    animation = ContinuumAnimation(
        robot1,
        port=SERIAL_PORT,
        baudrate=BAUD_RATE
    )
    animation.animate()
    
except Exception as e:
    print(f"An error occurred: {e}")
    # --- MODIFIED: Cleanup on error ---
    if 'animation' in locals():
        if animation.serial_log and animation.serial_log.is_open:
            animation.serial_log.close()
            print("Serial port closed due to error.")
        
        # Also close the CSV file on error
        if hasattr(animation, 'csv_file') and animation.csv_file:
            animation.csv_file.close()
            print(f"Data (partially) saved to {animation.csv_filename}")
    # ----------------------------------
    print("Exiting.")