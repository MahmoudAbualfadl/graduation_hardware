import os
import gymnasium as gym
import numpy as np
import pandas as pd
import torch
import pyvista as pv
import vtk
# from stable_baselines3 import SAC
# from stable_baselines3.common.callbacks import BaseCallback, EvalCallback, CheckpointCallback

class ContinuumRobot:
    """
    A class representing a continuum (flexible) robot segment, providing methods for kinematic modeling,
    curve sampling, and visualization utilities.
    (Updated with kinematics from continuum_sim.py for consistency)

    Attributes:
        length (float): The length of the robot segment.
        radius (float): The radius of the robot segment (default: 0.03).
        num_points (int): Number of points to sample along the robot's backbone (default: 100).
    """
    def __init__(self, length, radius=0.03, num_points=100):
        self.length = length
        self.radius = radius
        self.num_points = num_points
        if length <= 0:
            raise ValueError("Segment length must be positive.")
        if num_points < 2:
            raise ValueError("num_points must be at least 2.")

    @staticmethod
    def homogeneous_transform(s, kappa, phi):
        """
        Compute the homogeneous transformation matrix for a segment at arc length s.
        (Updated from continuum_sim.py)
        """
        T = np.eye(4)
        if kappa > 1e-9: # Use a small epsilon to avoid division by zero
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
            # Straight segment (kappa is effectively zero)
            T[2, 3] = s
        return T

    def sample_curve(self, kappa, phi):
        s_vals = np.linspace(0, self.length, self.num_points)
        pts = np.zeros((self.num_points, 3))
        for i, s in enumerate(s_vals):
            T = self.homogeneous_transform(s, kappa, phi)
            pts[i, :] = T[:3, 3]
        return pts

    @staticmethod
    def apply_global_transform(pts, x=0.0, y=0.0, z=0.0):
        """
        Apply a global translation to the points.
        (Updated from continuum_sim.py to be Z-up)
        """
        return pts + np.array([x, y, z])

    @staticmethod
    def generate_shadow(pts):
        shadow = pts.copy()
        shadow[:, 2] = 0.001 # Slightly above 0 to avoid z-fighting
        return shadow

    @staticmethod
    def create_shadow_mesh(shadow_pts, radius=0.02):
        if shadow_pts.shape[0] < 2 or np.allclose(shadow_pts, shadow_pts[0]):
            return None
        return pv.lines_from_points(shadow_pts).tube(radius=radius)

class ContinuumEnv(gym.Env):
    """
    ContinuumEnv is a custom OpenAI Gym environment for simulating and controlling
    a *single-segment* continuum robot in a 3D space.

    (Refactored to be a functional single-segment environment with dynamic scaling)
    """
    metadata = {"render_modes": ["human", None]}

    def __init__(self, positions_file: str = 'venv/Utiles/new_final_positions.csv', switch_threshold: int = 10, render_mode: str = "human"):
        super().__init__()
        
        # --- Environment Parameters ---
        self.render_mode = render_mode
        self.seg_length = 0.14
        self.robot_radius = 0.03
        self.base_pos = (0.0, 0.0)
        self.base_height = 0.0
        self.threshold = 0.05
        self.d_max = self.threshold
        self.switch_threshold=1
        self.max_steps = 250
        self.current_config = np.array([0.001, 0.0], dtype=np.float32)
        # --- Robot Model ---
        self.segment = ContinuumRobot(self.seg_length, radius=self.robot_radius)
        
        # --- State Variables ---
        self.current_step = 0
        self.success_count = 0
        self.reached_goal_this_episode = False

        
        # --- Visualization (Initialize plotter *before* goal loading) ---
        # <-- FIX: MOVED THESE LINES UP
        self._actor_names = ['tube1', 'shadow1']
        self.plotter = None
        

        
    
        self.goal =np.array([0.004016,0.082906,0.098545])
        # --- DYNAMIC RENDER SETUP (from continuum_sim.py) ---
        if self.render_mode == "human":
            # --- Dynamic Environment Scaling ---
            self.robot_length = self.seg_length
            
            # (e.g., 0.03 * 80 = 2.4)
            radius_scale_factor = 80.0 
            self.env_size = self.robot_radius * radius_scale_factor
            
            self.z_max = self.robot_length * 1.5
            self.goal_radius = self.robot_radius # Goal sphere radius
            # -----------------------------------

            self.plotter = pv.Plotter(notebook=False, off_screen=False, window_size=[1024, 768])
            self.plotter.background_color = "black"

            # --- Use scaled environment variables ---
            s_xy = self.env_size / 2.0
            
            self.plotter.show_grid(
                color='white', 
                grid='back', 
                location='outer',
                bounds=[-s_xy, s_xy, -s_xy, s_xy, 0, self.z_max] # Scaled bounds
            )
            
            # Add axes
            axes_actor = vtk.vtkAxesActor()
            # Scale axes to be visible (scaled by s_xy)
            axes_actor.SetTotalLength(s_xy * 0.4, s_xy * 0.4, s_xy * 0.4) 
            for axis_prop in [axes_actor.GetXAxisShaftProperty(),
                              axes_actor.GetYAxisShaftProperty(),
                              axes_actor.GetZAxisShaftProperty(),
                              axes_actor.GetXAxisTipProperty(),
                              axes_actor.GetYAxisTipProperty(),
                              axes_actor.GetZAxisTipProperty()]:
                axis_prop.SetColor(1, 1, 1)
            self.plotter.add_actor(axes_actor)
            
            # Add ground plane (scaled by env_size)
            plane = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), 
                             i_size=self.env_size, j_size=self.env_size)
            self.plotter.add_mesh(plane, color='gray', ambient=0.2, name='ground')
            
            # Add goal sphere (scaled)
            self._draw_goal_sphere() # Draw the initial goal (now works)
            
            # --- Set dynamic camera position (User-specified) ---
            view_up = (0, 0, 1)
            focal_point = (0, 0, self.robot_length * 1)
            cam_pos = (-s_xy*2, -s_xy*2, self.z_max)
            
            self.plotter.camera_position = [cam_pos, focal_point, view_up]
            
            # --- NEW CAMERA SCALING (User-specified) ---
            self.plotter.enable_parallel_projection()
            self.plotter.camera.parallel_scale = s_xy * 1.1
            
            # Show the plotter window
            self.plotter.show(auto_close=False, interactive_update=True)
    
        # Actions: [delta_kappa, delta_phi]
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )
        
        # --- START: UPDATED OBSERVATION SPACE ---
        # Observations: [tip_x, tip_y, tip_z, goal_x, goal_y, goal_z, kappa, phi]
        # Total 8 elements
        
        # Bounds for tip position (from user)
        tip_low = np.array([0, 0, 0])
        tip_high = np.array([0.14, 0.14, 0.14])
        
        # Bounds for goal (assuming goal can be anywhere in the workspace)
        # We use the same bounds as the tip position
        goal_low = tip_low
        goal_high = tip_high
        
        # Bounds for configuration (from step method clipping)
        config_low = np.array([0.001, 0.0])
        config_high = np.array([10.0, np.pi])
        
        # Concatenate all bounds for the 8-element observation space
        low_obs = np.concatenate([tip_low, goal_low, config_low]).astype(np.float32)
        high_obs = np.concatenate([tip_high, goal_high, config_high]).astype(np.float32)

        self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32)
        # --- 🔴 END: UPDATED OBSERVATION SPACE ---

  
 

    def _draw_goal_sphere(self):
        """Draws the goal sphere in the PyVista plotter."""
        if self.plotter is None:
            return
        try:
            self.plotter.remove_actor('goal')
        except Exception:
            pass
        # Use the dynamically scaled goal_radius
        sphere = pv.Sphere(radius=self.goal_radius, center=self.goal, theta_resolution=60, phi_resolution=60)
        # Use 'blue' color from sim
        self.plotter.add_mesh(sphere, color='blue', ambient=0.2, specular=0.6, name='goal')

    def _forward_tip(self, k1, phi1):
        """
        Computes the robot tip position and segment points.
        (Refactored for 1 segment)
        """
        seg1_pts = self.segment.sample_curve(k1, phi1)
        # Apply Z-up transform
        seg1_global = ContinuumRobot.apply_global_transform(seg1_pts, *self.base_pos, self.base_height)
        
        return seg1_global[-1], seg1_global

    def step(self, action):
            action = np.clip(action, self.action_space.low, self.action_space.high)
            
            # Update 2-element config
            self.current_config += action
            self.current_config[0] = np.clip(self.current_config[0], 0.001, 10.0) # kappa
            # --- 🔴 
            self.current_config[1] = np.clip(self.current_config[1], 0, np.pi)   # phi
            
            # Get tip position
            tip, _ = self._forward_tip(*self.current_config)
        
            dist = np.linalg.norm(tip - self.goal)
            
            if dist <= self.d_max:
                reward = 1000.0
                self.reached_goal_this_episode = True
                print("Goal reached in this episode")
            else:
                # Reward is negative, approaching 0 as dist -> d_max
                reward = 1 - (np.log1p(dist) / np.log1p(self.d_max)) 
            
            self.current_step += 1
            terminated = bool(dist <= self.threshold)
            

            truncated = bool(self.current_step >= self.max_steps)
            
            # --- 🔴 START: UPDATED OBSERVATION ---
            # Create 8-element observation [tip, goal, config]
            obs = np.concatenate([tip, self.goal, self.current_config]).astype(np.float32)
            obs = np.clip(obs, self.observation_space.low, self.observation_space.high)
            # --- 🔴 END: UPDATED OBSERVATION ---
            
            return obs, reward, terminated, truncated, {'distance': dist}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        
        if self.reached_goal_this_episode:
            self.success_count += 1
            print(f"Success count: {self.success_count}")
        else:
            self.success_count = 0
            print("Success count reset to 0")
            
        self.reached_goal_this_episode = False
        
        if self.success_count >= self.switch_threshold:
            self.success_count = 0
    
        self.current_step = 0
        self.current_config = np.array([0.001, 0.0], dtype=np.float32) 
        
        tip, _ = self._forward_tip(*self.current_config)
        # This already correctly returns the 8-element observation
        obs = np.concatenate([tip, self.goal, self.current_config]).astype(np.float32)
        
        return obs, {}

    def render(self, mode='human'):
        if self.render_mode != "human" or self.plotter is None:
            return
            
        k1, phi1 = self.current_config
        tip, seg1_pts = self._forward_tip(k1, phi1)
        
        # Create robot mesh
        tube1 = pv.lines_from_points(seg1_pts).tube(self.segment.radius)
        
        # Create shadow mesh
        shadow1_pts = ContinuumRobot.generate_shadow(seg1_pts)
        shadow1_mesh = ContinuumRobot.create_shadow_mesh(shadow1_pts, radius=self.segment.radius * 1.05)
        
        # Remove old actors
        for name in self._actor_names:
            try:
                self.plotter.remove_actor(name)
            except Exception:
                pass
                
        # Add new actors
        # Use 'green' color from sim
        self.plotter.add_mesh(tube1, name='tube1', color='green', ambient=0.1, specular=0.5)
        
        if shadow1_mesh:
            self.plotter.add_mesh(shadow1_mesh, name='shadow1', color='black', opacity=0.3)

        self.plotter.render()
        self.plotter.update()

    def close(self):
        if self.plotter is not None:
            self.plotter.close()
            self.plotter = None

