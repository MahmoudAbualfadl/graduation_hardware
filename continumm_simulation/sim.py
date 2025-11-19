# """
# continuum_sim.py

# This module provides an object-oriented simulation and animation of a two-segment continuum robot using PyVista.
# It supports rendering two robots (optionally), each composed of two serially connected continuum segments.
# The animation visualizes the robot(s) moving in 3D space, including their shadows and an optional goal sphere.
# Texture mapping is supported for realistic visualization.

# Classes:
#     ContinuumSegment: Represents a single continuum robot segment, with geometry and transformation utilities.
#     ContinuumRobot:   Represents a robot composed of two continuum segments.
#     ContinuumAnimation: Handles scene setup and animation of one or two robots.

# Example:
#     robot1 = ContinuumRobot([0.4, 0.4], base_height=1.0)
#     robot2 = ContinuumRobot([0.4, 0.4], base_height=1.0)
#     animation = ContinuumAnimation(robot1, robot2=robot2)
#     animation.animate()

# Dependencies:
# - numpy
# - pyvista
# - vtk
# - os

# Author: Motaz Elsaman
# Date: 2024-04-28
# """

# import numpy as np
# import pyvista as pv
# import vtk
# import os

# class ContinuumSegment:
#     """
#     Represents a single segment of a continuum robot.

#     Args:
#         length (float): Length of the segment.
#         radius (float): Tube radius for visualization.
#         num_points (int): Number of points sampled along the curve.
#     """
#     def __init__(self, length, radius=0.03, num_points=300):
#         self.length = length
#         self.radius = radius
#         self.num_points = num_points

#     @staticmethod
#     def homogeneous_transform(s, kappa, phi):
#         """
#         Compute the homogeneous transformation matrix for a segment at arc length s.

#         Args:
#             s (float): Arc length along the segment.
#             kappa (float): Curvature.
#             phi (float): Orientation angle.

#         Returns:
#             np.ndarray: 4x4 transformation matrix.
#         """
#         T = np.eye(4)
#         if kappa != 0:
#             c = np.cos(kappa * s)
#             s_val = np.sin(kappa * s)
#             one_minus_c = 1 - c

#             T[0, 0] = np.cos(phi)**2 * c + np.sin(phi)**2
#             T[0, 1] = np.cos(phi) * np.sin(phi) * (c - 1)
#             T[0, 2] = np.cos(phi) * s_val

#             T[1, 0] = np.cos(phi) * np.sin(phi) * (c - 1)
#             T[1, 1] = np.sin(phi)**2 * c + np.cos(phi)**2
#             T[1, 2] = np.sin(phi) * s_val

#             T[2, 0] = -np.cos(phi) * s_val
#             T[2, 1] = -np.sin(phi) * s_val
#             T[2, 2] = c

#             T[0, 3] = np.cos(phi) * one_minus_c / kappa
#             T[1, 3] = np.sin(phi) * one_minus_c / kappa
#             T[2, 3] = s_val / kappa
#         else:
#             T[2, 3] = s
#         return T

#     def sample_curve(self, kappa, phi):
#         """
#         Sample points along the segment curve.

#         Args:
#             kappa (float): Curvature.
#             phi (float): Orientation angle.

#         Returns:
#             np.ndarray: Array of 3D points along the segment.
#         """
#         s_vals = np.linspace(0, self.length, self.num_points)
#         points = []
#         for s in s_vals:
#             T = self.homogeneous_transform(s, kappa, phi)
#             points.append(T[:3, 3])
#         return np.array(points)

#     @staticmethod
#     def apply_global_transform(points, x=0, y=0, z=0):
#         """
#         Apply a global transformation (rotation and translation) to the points.

#         Args:
#             points (np.ndarray): Points to transform.
#             x, y, z (float): Base position.

#         Returns:
#             np.ndarray: Transformed points.
#         """
#         # 180-degree rotation about Y-axis and translation
#         rot_y = np.array([
#             [1, 0, 0],
#             [0, 1, 0],
#             [0, 0, 1]
#         ])
#         return (rot_y @ points.T).T + np.array([x, y, z])

#     @staticmethod
#     def generate_shadow(points):
#         """
#         Project points onto the ground plane (z=0) to create a shadow.

#         Args:
#             points (np.ndarray): Points to project.

#         Returns:
#             np.ndarray: Shadow points.
#         """
#         shadow = points.copy()
#         shadow[:, 2] = 0
#         return shadow

#     @staticmethod
#     def create_shadow_mesh(shadow_points, radius=0.031):
#         """
#         Create a tube mesh for the shadow.

#         Args:
#             shadow_points (np.ndarray): Shadow points.
#             radius (float): Tube radius.

#         Returns:
#             pyvista.PolyData or None: Tube mesh or None if not enough points.
#         """
#         if shadow_points.shape[0] < 2 or np.allclose(shadow_points, shadow_points[0]):
#             return None
#         return pv.lines_from_points(shadow_points).tube(radius=radius)

# class ContinuumRobot:
#     """
#     Represents a continuum robot composed of two segments.

#     Args:
#         seg_lengths (list): List of segment lengths.
#         base_height (float): Height of the robot base.
#         base_pos (tuple): (x, y) base position.
#         texture (pyvista.Texture): Optional texture for visualization.
#     """
#     def __init__(self, seg_lengths, base_height=0.0, base_pos=(0, 0), texture=None):
#         self.segments = [ContinuumSegment(l) for l in seg_lengths]
#         self.base_height = base_height
#         self.base_pos = base_pos
#         self.texture = texture

#     def get_segments(self, kappa1, phi1, kappa2, phi2):
#         """
#         Compute the 3D points for both robot segments.

#         Returns:
#             tuple: (seg1_points, seg2_points)
#         """
#         seg1 = self.segments[0].sample_curve(kappa1, phi1)
#         #seg2_local = self.segments[1].sample_curve(kappa2, phi2)
#         # Transform segment 2 into the frame of segment 1
#         T1 = ContinuumSegment.homogeneous_transform(self.segments[0].length, kappa1, phi1)
#         #seg2 = (T1[:3, :3] @ seg2_local.T).T + T1[:3, 3]
#         # Apply global transformation (base position and height)
#         seg1 = ContinuumSegment.apply_global_transform(seg1, *self.base_pos, self.base_height)
#         #seg2 = ContinuumSegment.apply_global_transform(seg2, *self.base_pos, self.base_height)
#         return seg1 #,seg2

#     def get_shadow_meshes(self, seg1):
#         """
#         Generate shadow meshes for both segments.

#         Returns:
#             tuple: (shadow1_mesh, shadow2_mesh)
#         """
#         shadow1 = ContinuumSegment.generate_shadow(seg1)
#         #shadow2 = ContinuumSegment.generate_shadow(seg2)
#         shadow1_mesh = ContinuumSegment.create_shadow_mesh(shadow1)
#         #shadow2_mesh = ContinuumSegment.create_shadow_mesh(shadow2)
#         return shadow1_mesh #, shadow2_mesh

#     def get_tube_meshes(self, seg1 ):
#         """
#         Generate tube meshes for both segments.

#         Returns:
#             tuple: (tube1, tube2)
#         """
#         tube1 = pv.lines_from_points(seg1).tube(self.segments[0].radius).texture_map_to_plane()
#         #tube2 = pv.lines_from_points(seg2).tube(self.segments[1].radius).texture_map_to_plane()
#         return tube1 #,  tube2

# class ContinuumAnimation:
#     """
#     Handles the animation and visualization of one or two continuum robots.

#     Args:
#         robot1 (ContinuumRobot): The main robot to animate.
#         robot2 (ContinuumRobot, optional): A second robot for comparison.
#         num_frames (int): Number of animation frames.
#         kappa_range (tuple): Range of curvature values.
#         phi_range (tuple): Range of orientation angles.
#         output_gif (str): Output GIF filename.
#     """
#     def __init__(self, robot1, robot2=None, num_frames=900, kappa_range=(0.001, 2.0), phi_range=(-np.pi, np.pi), output_gif="inverted_continuum_robot.gif"):
#         self.robot1 = robot1
#         self.robot2 = robot2
#         self.num_frames = num_frames
#         self.kappa_range = kappa_range
#         self.phi_range = phi_range
#         self.output_gif = output_gif
#         self.plotter = pv.Plotter(notebook=False)
#         self.setup_scene()

#     def setup_scene(self):
#         """
#         Set up the PyVista scene: background, grid, axes, ground, and goal.
#         """
#         self.plotter.background_color = "black"
#         self.plotter.open_gif(self.output_gif, fps=60)
#         self.plotter.show_grid(color='white', grid='back', location='outer')
#         # Add axes
#         axes_actor = vtk.vtkAxesActor()
#         for axis_prop in [axes_actor.GetXAxisShaftProperty(),
#                           axes_actor.GetYAxisShaftProperty(),
#                           axes_actor.GetZAxisShaftProperty(),
#                           axes_actor.GetXAxisTipProperty(),
#                           axes_actor.GetYAxisTipProperty(),
#                           axes_actor.GetZAxisTipProperty()]:
#             axis_prop.SetColor(1, 1, 1)
#         self.plotter.add_actor(axes_actor)
#         # Add ground plane
#         plane = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=5, j_size=5)
#         self.plotter.add_mesh(plane, color='gray', ambient=0.2, name='ground')
#         # Add goal sphere
#         goal_center = (0.25, 0.25, 0.5)
#         goal_radius = 0.09
#         goal = pv.Sphere(radius=goal_radius, center=goal_center, theta_resolution=60, phi_resolution=60)
#         self.plotter.add_mesh(goal, color='green', name='goal', specular=0.6, ambient=0.2)

#     def animate(self):
#         """
#         Run the animation loop, updating robot(s) and writing frames to the GIF.
#         """
#         for frame in range(self.num_frames):
#             t = frame / self.num_frames
#             # Animate curvature and orientation using sinusoidal functions
#             kappa1 = self.kappa_range[0] + (self.kappa_range[1] - self.kappa_range[0]) * np.abs(np.sin(2 * np.pi * t))
#             phi1 = self.phi_range[0] + (self.phi_range[1] - self.phi_range[0]) * t
#             kappa2 = self.kappa_range[1] * (np.sin(4 * np.pi * t) ** 2)
#             phi2 = phi1	
#             #phi2 = phi1 + np.pi / 2
# 	        # this is to make the pending plane the same for two segments
	        
#             # Get segment points and meshes for robot1
#             seg1   = self.robot1.get_segments(kappa1, phi1, kappa2, phi2)
#             tube1 = self.robot1.get_tube_meshes(seg1)
#             shadow1_mesh= self.robot1.get_shadow_meshes(seg1)

#             # If a second robot is present, offset it along y-axis and get its meshes
#             if self.robot2:
#                 seg1_robot2 = seg1 + np.array([0, 0.5, 0])
#                 #seg2_robot2 = seg2 + np.array([0, 0.5, 0])
#                 tube1_robot2, tube2_robot2 = self.robot2.get_tube_meshes(seg1_robot2)
#                 shadow1_mesh_robot2, shadow2_mesh_robot2 = self.robot2.get_shadow_meshes(seg1_robot2)

#             # Remove previous actors for clean frame
#             for name in ['tube1', 'tube2', 'shadow1', 'shadow2', 'tube1_robot2', 'tube2_robot2', 'shadow1_robot2', 'shadow2_robot2']:
#                 self.plotter.remove_actor(name, reset_camera=False)
#             self.plotter.remove_legend()

#             # Add robot1 meshes
#             self.plotter.add_mesh(tube1, name='tube1', texture=self.robot1.texture,
#                                   color='red' if self.robot1.texture is None else None,
#                                   ambient=0.1, specular=0.5,
#                                   label=f'Seg1_1: kappa1_1= {kappa1:.1f}, phi1_1= {np.degrees(phi1):.0f}°')
#             # self.plotter.add_mesh(tube2, name='tube2', texture=self.robot1.texture,
#                                 #  color='blue' if self.robot1.texture is None else None,
#                                 ##  ambient=0.1, specular=0.5,
#                                   #label=f'Seg1_2: kappa1_2= {kappa2:.1f}, phi1_2= {np.degrees(phi2):.0f}°')
#             if shadow1_mesh:
#                 self.plotter.add_mesh(shadow1_mesh, name='shadow1', color='black', opacity=0.3)
#             # if shadow2_mesh:
#               #  self.plotter.add_mesh(shadow2_mesh, name='shadow2', color='black', opacity=0.3)

#             # Add robot2 meshes if present
#             if self.robot2:
#                 self.plotter.add_mesh(tube1_robot2, name='tube1_robot2', texture=self.robot2.texture,
#                                       color='red' if self.robot2.texture is None else None,
#                                       ambient=0.1, specular=0.5,
#                                       label=f'Seg2_1 (Robot 2): kappa2_1 ={kappa1:.1f}, phi2_1= {np.degrees(phi1):.0f}°')
#                 self.plotter.add_mesh(tube2_robot2, name='tube2_robot2', texture=self.robot2.texture,
#                                       color='blue' if self.robot2.texture is None else None,
#                                       ambient=0.1, specular=0.5,
#                                       label=f'Seg2_2 (Robot 2): kappa2_2= {kappa2:.1f}, phi2_2= {np.degrees(phi2):.0f}°')
#                 if shadow1_mesh_robot2:
#                     self.plotter.add_mesh(shadow1_mesh_robot2, name='shadow1_robot2', color='black', opacity=0.3)
#                 if shadow2_mesh_robot2:
#                     self.plotter.add_mesh(shadow2_mesh_robot2, name='shadow2_robot2', color='black', opacity=0.3)

#             # Add legend and write frame to GIF
#             self.plotter.add_legend(bcolor=(0.9, 0.9, 0.9), size=[0.22, 0.22])
#             self.plotter.write_frame()
#         self.plotter.close()
#         print("Animation saved.")

# # --- Usage Example ---

# # Parameters for the robots and animation
# seg_lengths = [0.14, 0.4]
# kappa_range = (0.001, 10.0)
# phi_range = (0, np.pi)
# num_frames = 1200
# base_height = 0.0

# # Load texture for visualization if available
# texture_path = "/home/motaz/Projects/graduation_project/venv/metal_texture.jpg"
# texture = None
# if os.path.exists(texture_path):
#     try:
#         texture = pv.read_texture(texture_path)
#     except Exception as e:
#         print(f"Error loading texture: {e}")
# else:
#     print(f"Texture file not found: {texture_path}")

# # Instantiate robots
# robot1 = ContinuumRobot(seg_lengths, base_height=base_height, base_pos=(0, 0), texture=texture)
# robot2 = ContinuumRobot(seg_lengths, base_height=base_height, base_pos=(0, 0), texture=texture)  # Optional second robot

# # Create and run the animation
# animation = ContinuumAnimation(robot1, num_frames=num_frames, kappa_range=kappa_range, phi_range=phi_range)
# animation.animate()


####################################################################################################################################################################################################################
"""
continuum_sim.py

This module provides an object-oriented simulation and animation of a single-segment continuum robot using PyVista.
It supports rendering two robots (optionally) for comparison.
The animation visualizes the robot(s) moving in 3D space, including their shadows and a dynamic goal sphere.
Texture mapping is supported for realistic visualization.

Classes:
    ContinuumSegment:   Represents a single continuum robot segment.
    ContinuumRobot:     Represents a robot composed of one continuum segment.
    ContinuumAnimation: Handles scene setup and animation of one or two robots.

Example:
    robot1 = ContinuumRobot(length=0.4, radius=0.03, base_pos=(0, -0.25))
    robot2 = ContinuumRobot(length=0.4, radius=0.03, base_pos=(0, 0.25))
    animation = ContinuumAnimation(robot1, robot2=robot2)
    animation.animate()

Dependencies:
- numpy
- pyvista
- vtk
- os

Author: Motaz Elsaman
Date: 2024-04-28
(Improved by Gemini: 2025-11-09)
"""

import numpy as np
import pyvista as pv
import vtk
import os

class ContinuumSegment:
    """
    Represents a single segment of a continuum robot.

    Args:
        length (float): Length of the segment.
        radius (float): Tube radius for visualization.
        num_points (int): Number of points sampled along the curve.
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
        """
        Compute the homogeneous transformation matrix for a segment at arc length s.

        Args:
            s (float): Arc length along the segment.
            kappa (float): Curvature.
            phi (float): Orientation angle.

        Returns:
            np.ndarray: 4x4 transformation matrix.
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
        """
        Sample points along the segment curve.

        Args:
            kappa (float): Curvature.
            phi (float): Orientation angle.

        Returns:
            np.ndarray: Array of 3D points along the segment.
        """
        s_vals = np.linspace(0, self.length, self.num_points)
        points = np.zeros((self.num_points, 3))
        
        for i, s in enumerate(s_vals):
            T = self.homogeneous_transform(s, kappa, phi)
            points[i, :] = T[:3, 3]
            
        return points

    @staticmethod
    def apply_global_transform(points, x=0, y=0, z=0):
        """
        Apply a global translation to the points.

        Args:
            points (np.ndarray): Points to transform.
            x, y, z (float): Base position.

        Returns:
            np.ndarray: Transformed points.
        """
        return points + np.array([x, y, z])


    @staticmethod
    def generate_shadow(points):
        """
        Project points onto the ground plane (z=0) to create a shadow.

        Args:
            points (np.ndarray): Points to project.

        Returns:
            np.ndarray: Shadow points.
        """
        shadow = points.copy()
        shadow[:, 2] = 0.001  # Slightly above 0 to avoid z-fighting with the plane
        return shadow

    @staticmethod
    def create_shadow_mesh(shadow_points, radius=0.031):
        """
        Create a tube mesh for the shadow.

        Args:
            shadow_points (np.ndarray): Shadow points.
            radius (float): Tube radius.

        Returns:
            pyvista.PolyData or None: Tube mesh or None if not enough points.
        """
        if shadow_points.shape[0] < 2 or np.allclose(shadow_points, shadow_points[0]):
            return None
        return pv.lines_from_points(shadow_points).tube(radius=radius)

class ContinuumRobot:
    """
    Represents a continuum robot composed of a *single* segment.

    Args:
        length (float): Segment length.
        radius (float): Segment radius for visualization.
        base_height (float): Height of the robot base (z-offset).
        base_pos (tuple): (x, y) base position.
        texture (pyvista.Texture): Optional texture for visualization.
    """
    def __init__(self, length, radius=0.03, base_height=0.0, base_pos=(0, 0), texture=None):
        self.segment = ContinuumSegment(length, radius=radius)
        self.base_height = base_height
        self.base_pos = base_pos
        self.texture = texture

    def get_segment_points(self, kappa, phi):
        """
        Compute the 3D points for the robot segment.

        Returns:
            np.ndarray: 3D points of the segment.
        """
        seg_points = self.segment.sample_curve(kappa, phi)
        
        # Apply global transformation (base position and height)
        seg_points = ContinuumSegment.apply_global_transform(
            seg_points, *self.base_pos, self.base_height
        )
        return seg_points

    def get_shadow_mesh(self, seg_points):
        """
        Generate shadow mesh for the segment.

        Returns:
            pyvista.PolyData or None: Shadow mesh.
        """
        shadow = ContinuumSegment.generate_shadow(seg_points)
        # Shadow radius is scaled from the segment's actual radius
        shadow_mesh = ContinuumSegment.create_shadow_mesh(shadow, radius=self.segment.radius * 1.05)
        return shadow_mesh

    def get_tube_mesh(self, seg_points):
        """
        Generate tube mesh for the segment.

        Returns:
            pyvista.PolyData: Tube mesh.
        """
        if seg_points.shape[0] < 2:
            return None
        # Tube mesh uses the segment's actual radius
        tube = pv.lines_from_points(seg_points).tube(self.segment.radius)
        if self.texture:
            tube.texture_map_to_plane(inplace=True)
        return tube

class ContinuumAnimation:
    """
    Handles the animation and visualization of one or two continuum robots.

    Args:
        robot1 (ContinuumRobot): The main robot to animate.
        robot2 (ContinuumRobot, optional): A second robot for comparison.
        num_frames (int): Number of animation frames.
        kappa_range (tuple): Range of curvature values.
        phi_range (tuple): Range of orientation angles.
        output_gif (str): Output GIF filename.
    """
    def __init__(self, robot1, robot2=None, num_frames=300, 
                 kappa_range=(0.001, 10.0), phi_range=(0, np.pi), 
                 output_gif="continuum_robot_1seg.gif"):
        
        self.robot1 = robot1
        self.robot2 = robot2
        self.num_frames = num_frames
        self.kappa_range = kappa_range
        self.phi_range = phi_range
        self.output_gif = output_gif
        
        # --- Dynamic Environment Scaling ---
        self.robot_length = robot1.segment.length
        self.robot_radius = robot1.segment.radius
        
        # Scale environment size based on robot length
        # We define a factor to scale the radius to an appropriate env size
        # (e.g., 0.03 * 20 = 0.6, which is similar to 0.14 * 4.0 = 0.56)
        radius_scale_factor = 80.0 
        self.env_size = self.robot_radius * radius_scale_factor  # <-- UPDATED SCALING
        
        # Z (height) and goal position are still best scaled by length
        self.z_max = self.robot_length * 1.5
        self.goal_center = (0.004016,0.082906,0.098545)
        
        # Goal radius scales with robot radius
        self.goal_radius = self.robot_radius 
        # -----------------------------------

        self.plotter = pv.Plotter(notebook=False, window_size=[1024, 768])
        self.setup_scene()

    def setup_scene(self):
        """
        Set up the PyVista scene: background, grid, axes, ground, and goal.
        (Using the user-provided parameters)
        """
        self.plotter.background_color = "black"
        self.plotter.open_gif(self.output_gif, fps=60)

        # --- Use scaled environment variables ---
        # s_xy is now scaled by robot_radius via env_size
        s_xy = self.env_size / 2.0 # <-- UPDATED SCALING
        
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
        goal = pv.Sphere(radius=self.goal_radius, center=self.goal_center, 
                         theta_resolution=60, phi_resolution=60)
        self.plotter.add_mesh(goal, color='blue', name='goal', specular=0.6, ambient=0.2)
        
        # --- Set dynamic camera position (User-specified) ---
        view_up = (0, 0, 1)
        # Focal point should still be based on robot length
        focal_point = (0, 0, self.robot_length * 1)
        # Camera position is now scaled by new s_xy (from radius)
        cam_pos = (-s_xy*2, -s_xy*2, self.z_max) # <-- UPDATED SCALING
        
        self.plotter.camera_position = [cam_pos, focal_point, view_up]
        
        # --- NEW CAMERA SCALING (User-specified) ---
        # Enable parallel projection for consistent scaling
        self.plotter.enable_parallel_projection()
        
        # Set the parallel scale (half-height of the viewport)
        # This is now also scaled by s_xy (from radius)
        self.plotter.camera.parallel_scale = s_xy * 1.1 # <-- UPDATED SCALING
        # ---

    def animate(self):
        """
        Run the animation loop, updating robot(s) and writing frames to the GIF.
        """
        print(f"Starting animation... saving to {self.output_gif}")
        for frame in range(self.num_frames):
            t = frame / self.num_frames
            
            # --- Animate parameters for Robot 1 ---
            kappa1 = self.kappa_range[0] + (self.kappa_range[1] - self.kappa_range[0]) * np.abs(np.sin(2 * np.pi * t))
            phi1 = self.phi_range[0] + (self.phi_range[1] - self.phi_range[0]) * t
            
            # Get segment points and meshes for robot1
            seg1 = self.robot1.get_segment_points(kappa1, phi1)
            tube1 = self.robot1.get_tube_mesh(seg1)
            shadow1_mesh = self.robot1.get_shadow_mesh(seg1)

            # If a second robot is present, get its meshes
            if self.robot2:
                # --- Animate parameters for Robot 2 (e.g., with phase shift) ---
                kappa1_r2 = self.kappa_range[0] + (self.kappa_range[1] - self.kappa_range[0]) * np.abs(np.sin(2 * np.pi * t + np.pi/2))
                phi1_r2 = phi1
            
                seg1_robot2 = self.robot2.get_segment_points(kappa1_r2, phi1_r2)
                tube1_robot2 = self.robot2.get_tube_mesh(seg1_robot2)
                shadow1_mesh_robot2 = self.robot2.get_shadow_mesh(seg1_robot2)


            # Remove previous actors for clean frame
            for name in ['tube1', 'shadow1', 'tube1_robot2', 'shadow1_robot2']:
                self.plotter.remove_actor(name, reset_camera=False)
            self.plotter.remove_legend()

            # --- Add robot1 meshes ---
            if tube1:
                self.plotter.add_mesh(tube1, name='tube1', texture=self.robot1.texture,
                                      color='green' if self.robot1.texture is None else None,
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

            # Add legend and write frame to GIF
            self.plotter.add_legend(bcolor=(0.9, 0.9, 0.9), size=[0.22, 0.15])
            self.plotter.write_frame()
            
        self.plotter.close()
        print("Animation saved.")


# Parameters for the robots and animation
robot_length = 0.14  # Single segment length
robot_radius = 0.03  # <-- Robot's radius
kappa_range = (0.001, 10)
phi_range = (0,  np.pi) # Animate a half circle
num_frames = 360 # 6 seconds at 60fps
base_height = 0.0




# Instantiate robots
robot1 = ContinuumRobot(
    length=robot_length, 
    radius=robot_radius, # <-- Pass the radius
    base_height=base_height, 
    base_pos=(0, 0),
)


# Create and run the animation
animation = ContinuumAnimation(
    robot1, 
    # robot2=robot2,  # <-- Uncomment this to add the second robot
    num_frames=num_frames, 
    kappa_range=kappa_range, 
    phi_range=phi_range
)
animation.animate()