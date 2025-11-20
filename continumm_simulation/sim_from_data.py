"""
continuum_sim_from_serial.py

This module provides an object-oriented simulation and animation of a single-segment
continuum robot using PyVista, driven by LIVE data from a serial port.

It renders the robot moving in 3D space, including its shadow and a
dynamic goal sphere whose position is also read from the serial data.

Classes:
    ContinuumSegment:   Represents a single continuum robot segment.
    ContinuumRobot:     Represents a robot composed of one continuum segment.
    ContinuumAnimation: Handles scene setup and live animation from serial data.

Example:
    robot1 = ContinuumRobot(length=0.4, radius=0.03, base_pos=(0, 0))
    # NOTE: Check your device's BAUD_RATE
    animation = ContinuumAnimation(robot1, port='/dev/ttyACM0', baudrate=115200)
    animation.animate()

Dependencies:
- numpy
- pyvista
- vtk
- os
- pandas (No longer needed for main operation, but kept for ContinuumSegment)
- pyserial (NEW dependency: pip install pyserial)

Author: Motaz Elsaman
Date: 2024-04-28
(Modified for live serial data by Gemini: 2025-11-13)
"""

import numpy as np
import pyvista as pv
import vtk
import os
import pandas as pd # Keep for np/pd compatibility if needed, though not strictly for CSV
import serial       # --- NEW: Import pyserial
import time         # --- NEW: For serial connection handling

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
    based on data from a SERIAL PORT.

    Args:
        robot1 (ContinuumRobot): The main robot to animate.
        port (str): The serial port name (e.g., '/dev/ttyACM0' or 'COM3').
        baudrate (int): The serial baud rate (e.g., 9600, 115200).
        robot2 (ContinuumRobot, optional): A second robot for comparison.
    """
    def __init__(self, robot1, port, baudrate=115200, robot2=None):
        
        self.robot1 = robot1
        self.robot2 = robot2
        
        # --- NEW: Serial Port Setup ---
        self.port = port
        self.baudrate = baudrate
        self.serial_log = None
        try:
            # Set a timeout so readline() doesn't block forever
            self.serial_log = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connecting to serial port {self.port} at {self.baudrate}...")
            # Give the connection a moment to establish
            time.sleep(2) 
            print("Connection established.")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {self.port}: {e}")
            print("Please check port name, permissions, and ensure no other program is using it.")
            raise
        # -------------------------------

        # --- NEW: Get initial data to set up scene ---
        # We need to read one good line of data to know where to place the initial goal.
        # This helper will try to get the first valid data, flushing any
        # startup garbage from the serial buffer.
        
        # Default data (straight up, goal at tip) in case of read failure
        default_data = (0.0, 0.0, 0.0, 0.0, self.robot1.segment.length)
        self.current_data = self._get_initial_data(default_data)
        
        kappa, phi, goal_x, goal_y, goal_z = self.current_data
        # -----------------------------------------------

        # --- Dynamic Environment Scaling (Unchanged) ---
        self.robot_length = robot1.segment.length
        self.robot_radius = robot1.segment.radius
        radius_scale_factor = 80.0 
        self.env_size = self.robot_radius * radius_scale_factor
        self.z_max = self.robot_length * 1.5
        
        # --- MODIFIED: Set initial goal from *serial data* ---
        self.goal_center = (goal_x, goal_y, goal_z)
        self.goal_radius = self.robot_radius 

        self.plotter = pv.Plotter(notebook=False, window_size=[1024, 768])
        self.setup_scene()


    def _get_initial_data(self, default_data):
        """
        NEW: Helper to read the serial port until one valid line is parsed.
        This clears any garbage data sent on connection.
        """
        print("Flushing serial buffer and waiting for initial data...")
        self.serial_log.flushInput()
        start_time = time.time()
        
        # Try for 5 seconds to get a valid line
        while time.time() - start_time < 5.0:
            try:
                line_bytes = self.serial_log.readline()
                if not line_bytes:
                    continue # Timeout, try again
                    
                line = line_bytes.decode('utf-8').strip()
                if not line:
                    continue # Empty line
                
                parts = line.split(',')
                if len(parts) == 5:
                    data = tuple(float(p) for p in parts)
                    print(f"Initial data acquired: {data}")
                    return data
                else:
                    print(f"  ... skipping malformed line: {line}")

            except (UnicodeDecodeError, ValueError) as e:
                print(f"  ... skipping corrupt data: {e}")
        
        print(f"Warning: Failed to get initial data. Using default: {default_data}")
        return default_data

    def _read_serial_data(self):
        """
        NEW: Helper function to read and parse one line of serial data.
        Returns the last valid data if the new line is bad.
        """
        # Check if there's data waiting, otherwise just return the old data
        if self.serial_log.in_waiting == 0:
            return self.current_data

        try:
            line = self.serial_log.readline().decode('utf-8').strip()
            
            if not line:
                # Got an empty string, probably a timeout
                return self.current_data

            parts = line.split(',')
            if len(parts) == 5:
                # Try to convert all parts to float
                data = tuple(float(p) for p in parts)
                # If successful, update and return the new data
                self.current_data = data
                return self.current_data
            else:
                # Wrong number of parts, line is malformed
                print(f"Warning: Incomplete data: {line}")
                return self.current_data # Return the last good data

        except (UnicodeDecodeError, ValueError) as e:
            # Data was corrupt or not a number
            print(f"Warning: Malformed data: {line} ({e})")
            return self.current_data # Return the last good data
            
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            self.plotter.close() # Close the plotter if serial fails
            return self.current_data

    def setup_scene(self):
        """
        Set up the PyVista scene.
        """
        self.plotter.background_color = "black"
        
        # --- MODIFIED: We are no longer writing a GIF by default ---
        # This is a LIVE plot.
        # self.plotter.open_gif(self.output_gif, fps=60) 
        # --------------------------------------------------------

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
        NEW: This function is the callback.
        It reads new data and updates the actors in the scene.
        This is the code that was *inside* the old animate() loop.
        """
        # --- Get parameters from *live serial data* ---
        kappa1, phi1, goal_x, goal_y, goal_z = self._read_serial_data()
        current_goal_pos = (goal_x, goal_y, goal_z)
        # -----------------------------------------------
        
        # Get segment points and meshes for robot1
        seg1 = self.robot1.get_segment_points(kappa1, phi1)
        tube1 = self.robot1.get_tube_mesh(seg1)
        shadow1_mesh = self.robot1.get_shadow_mesh(seg1)

        # If a second robot is present
        if self.robot2:
            kappa1_r2 = kappa1 # (Define how robot2 gets its data)
            phi1_r2 = phi1
            seg1_robot2 = self.robot2.get_segment_points(kappa1_r2, phi1_r2)
            tube1_robot2 = self.robot2.get_tube_mesh(seg1_robot2)
            shadow1_mesh_robot2 = self.robot2.get_shadow_mesh(seg1_robot2)

        # --- Remove previous actors for clean frame ---
        for name in ['tube1', 'shadow1', 'tube1_robot2', 'shadow1_robot2', 'goal']:
            self.plotter.remove_actor(name, reset_camera=False)
        self.plotter.remove_legend()

        # --- Add robot1 meshes ---
        if tube1:
            self.plotter.add_mesh(tube1, name='tube1', texture=self.robot1.texture,
                                  color='red' if self.robot1.texture is None else None,
                                  ambient=0.1, specular=0.5,
                                  label=f'Robot 1: k={kappa1:.1f}, p={np.degrees(phi1):.0f}°')
        if shadow1_mesh:
            self.plotter.add_mesh(shadow1_mesh, name='shadow1', color='black', opacity=0.3)

        # --- Add robot2 meshes if present ---
        if self.robot2:
            if tube1_robot2:
                self.plotter.add_mesh(tube1_robot2, name='tube1_robot2', texture=self.robot2.texture,
                                      color='blue' if self.robot2.texture is None else None,
                                      ambient=0.1, specular=0.5,
                                      label=f'Robot 2: k={kappa1_r2:.1f}, p={np.degrees(phi1_r2):.0f}°')
            if shadow1_mesh_robot2:
                self.plotter.add_mesh(shadow1_mesh_robot2, name='shadow1_robot2', color='black', opacity=0.3)

        # --- Add the updated goal sphere ---
        goal = pv.Sphere(radius=self.goal_radius, center=current_goal_pos,
                         theta_resolution=60, phi_resolution=60)
        self.plotter.add_mesh(goal, color='green', name='goal', specular=0.6, ambient=0.2)
        
        self.plotter.add_legend(bcolor=(0.9, 0.9, 0.9), size=[0.22, 0.15])
        
        # --- MODIFIED: No longer writing to GIF frame by frame ---
        # self.plotter.write_frame() 
        # The plotter handles rendering automatically in a live loop.
        
    def animate(self):
        """
        --- MODIFIED: Run the LIVE animation loop. ---
        """
        print("Starting live animation... Press 'q' in the window to quit.")
        
        # Register the 'update_plot' function as a callback.
        # PyVista will call this function repeatedly.
        # 'interval' is in milliseconds. 16ms is ~60 FPS.
        self.plotter.add_callback(self.update_plot, interval=16) 
        
        # Show the plotter. This is a blocking call.
        # It will run until the user closes the window (or presses 'q').
        # 'auto_close=False' keeps the window open until 'q' is pressed.
        self.plotter.show(interactive=True, auto_close=False)
        
        # --- NEW: Cleanup after the window is closed ---
        if self.serial_log and self.serial_log.is_open:
            self.serial_log.close()
            print("Serial port closed.")
        print("Animation stopped.")
        # ---------------------------------------------


# --- Main execution block ---

# Parameters for the robots and animation
robot_length = 0.14  # Single segment length
robot_radius = 0.03  # Robot's radius
base_height = 0.0

# --- NEW: Define Serial Port and Baud Rate ---
SERIAL_PORT = '/dev/ttyACM0'
# IMPORTANT: Make sure this baud rate matches your device's setting
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
    animation = ContinuumAnimation(
        robot1,
        port=SERIAL_PORT,
        baudrate=BAUD_RATE
    )
    animation.animate()
    
except Exception as e:
    print(f"An error occurred: {e}")
    print("Exiting.")