# kcl_project/utils/docking.py

import carb
import omni.usd
import numpy as np
import random
import math
from pxr import Gf, UsdGeom, Sdf
from isaacsim.core.utils.prims import get_prim_at_path
from ..config import DOCKING_MARKER_CONFIG

class SimpleDockingController:
    """
    Simple docking controller that moves DragonX to ISS position using invisible marker cubes.
    """
    
    def __init__(self):
        self.dragon_prim_path = "/World/DragonX"
        self.iss_prim_path = "/World/ISS"
        self.dragon_imu_path = "/World/DragonX/dragon_primary_imu"
        self.iss_imu_path = "/World/ISS/iss_docking_port_imu"
        self.dragon_marker_path = "/World/DragonX/docking_marker"
        self.iss_marker_path = "/World/ISS/docking_marker"
        self.docking_active = False
        self.approach_speed = 0.35  # Approach speed (increased for faster docking)
        self.final_speed = 0.25     # Final docking speed (increased for faster completion)
        self.target_position = None
        self.target_orientation = None
        self.start_position = None
        self.start_orientation = None
        self.docking_progress = 0.0
        self.markers_created = False
        self.preserve_original_orientation = True  # Flag to prevent unwanted rotations
        
    def get_prim_transform(self, prim_path):
        """
        Get the world transform of a prim.
        
        Args:
            prim_path: Path to the prim
            
        Returns:
            tuple: (position, orientation) or (None, None) if failed
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return None, None
                
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return None, None
                
            # Get the world transform
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                return None, None
                
            # Get transform matrix (use current time)
            transform_matrix = xformable.ComputeLocalToWorldTransform(0.0)
            
            # Extract position
            position = transform_matrix.ExtractTranslation()
            
            # Extract rotation as quaternion
            rotation_matrix = transform_matrix.ExtractRotationMatrix()
            quat = rotation_matrix.ExtractRotation().GetQuaternion()
            orientation = (quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2])
            
            return (position[0], position[1], position[2]), orientation
            
        except Exception as e:
            carb.log_error(f"[Docking] Error getting prim transform for {prim_path}: {str(e)}")
            return None, None
    
    def set_prim_transform(self, prim_path, position, orientation=None, preserve_scale=True):
        """
        Set the world transform of a prim.
        
        Args:
            prim_path: Path to the prim
            position: (x, y, z) position
            orientation: (w, x, y, z) quaternion orientation (optional)
            preserve_scale: Whether to preserve existing scale values (default: True)
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return False
                
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False
                
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                return False
            
            # Get existing transform ops to preserve their types and values
            existing_ops = xformable.GetOrderedXformOps()
            existing_scale = None
            existing_rotation_op = None
            existing_rotation_value = None
            
            # Extract existing values and operation types
            for op in existing_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    existing_scale = op.Get()
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    existing_rotation_op = op
                    existing_rotation_value = op.Get()
                elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    existing_rotation_op = op
                    existing_rotation_value = op.Get()
                    
            # If no scale found, try to extract from transform matrix
            if existing_scale is None and preserve_scale:
                try:
                    transform_matrix = xformable.ComputeLocalToWorldTransform(0.0)
                    scale_x = transform_matrix.GetColumn(0).GetLength()
                    scale_y = transform_matrix.GetColumn(1).GetLength()
                    scale_z = transform_matrix.GetColumn(2).GetLength()
                    existing_scale = Gf.Vec3d(scale_x, scale_y, scale_z)
                    
                    if existing_scale is None or (existing_scale[0] == 0 and existing_scale[1] == 0 and existing_scale[2] == 0):
                        existing_scale = Gf.Vec3d(0.1, 0.1, 0.1) if "DragonX" in prim_path else Gf.Vec3d(1.0, 1.0, 1.0)
                        carb.log_warn(f"[Docking] Using fallback scale for {prim_path}: {existing_scale}")
                except Exception as e:
                    carb.log_error(f"[Docking] Could not extract scale: {str(e)}")
                    existing_scale = Gf.Vec3d(0.1, 0.1, 0.1) if "DragonX" in prim_path else Gf.Vec3d(1.0, 1.0, 1.0)
            
            # Clear existing transform ops
            xformable.ClearXformOpOrder()
            
            # Set translation
            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
            
            # Handle rotation - preserve the original operation type when possible
            if orientation:
                # New orientation provided - use quaternion
                rotate_op = xformable.AddOrientOp()
                quat = Gf.Quatf(float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3]))
                rotate_op.Set(quat)
            elif existing_rotation_op:
                # Preserve existing rotation using the same operation type
                if existing_rotation_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = xformable.AddRotateXYZOp()
                    rotate_op.Set(existing_rotation_value)
                elif existing_rotation_op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    rotate_op = xformable.AddOrientOp()
                    rotate_op.Set(existing_rotation_value)
            
            # Restore scale if preserved
            if preserve_scale and existing_scale:
                scale_op = xformable.AddScaleOp()
                scale_op.Set(existing_scale)
            
            return True
            
        except Exception as e:
            carb.log_error(f"[Docking] Error setting prim transform for {prim_path}: {str(e)}")
            return False
    
    def create_invisible_marker_cubes(self):
        """
        Create invisible marker cubes with manually defined positions and orientations for docking alignment.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                carb.log_error("[Docking] No USD stage available")
                return False
            
            # Use marker configurations from extension
            dragon_marker_config = DOCKING_MARKER_CONFIG["dragon_marker"]
            iss_marker_config = DOCKING_MARKER_CONFIG["iss_marker"]

            # Create DragonX marker cube
            dragon_marker = UsdGeom.Cube.Define(stage, self.dragon_marker_path)
            dragon_marker.GetSizeAttr().Set(1.0)  # Small cube

            # Set DragonX marker position and orientation
            dragon_xform = UsdGeom.Xformable(dragon_marker)
            dragon_xform.ClearXformOpOrder()
            translate_op = dragon_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(dragon_marker_config["position"][0], dragon_marker_config["position"][1], dragon_marker_config["position"][2]))

            # Set orientation for DragonX marker
            orient_op = dragon_xform.AddOrientOp()
            quat = Gf.Quatf(dragon_marker_config["orientation"][0], dragon_marker_config["orientation"][1], dragon_marker_config["orientation"][2], dragon_marker_config["orientation"][3])
            orient_op.Set(quat)
            
            # Make invisible
            dragon_marker.MakeInvisible()
            
            # Create ISS marker cube
            iss_marker = UsdGeom.Cube.Define(stage, self.iss_marker_path)
            iss_marker.GetSizeAttr().Set(1.0)  # Small cube
            
            # Set ISS marker position and orientation
            iss_xform = UsdGeom.Xformable(iss_marker)
            iss_xform.ClearXformOpOrder()
            translate_op = iss_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(iss_marker_config["position"][0], iss_marker_config["position"][1], iss_marker_config["position"][2]))

            # Set orientation for ISS marker
            orient_op = iss_xform.AddOrientOp()
            quat = Gf.Quatf(iss_marker_config["orientation"][0], iss_marker_config["orientation"][1], iss_marker_config["orientation"][2], iss_marker_config["orientation"][3])
            orient_op.Set(quat)
            
            # Make invisible
            iss_marker.MakeInvisible()
            
            self.markers_created = True
            carb.log_info("[Docking] Created invisible marker cubes for docking alignment")
            carb.log_info(f"[Docking] DragonX marker at: {dragon_marker_config['position']} with orientation: {dragon_marker_config['orientation']}")
            carb.log_info(f"[Docking] ISS marker at: {iss_marker_config['position']} with orientation: {iss_marker_config['orientation']}")
            
            return True
            
        except Exception as e:
            carb.log_error(f"[Docking] Error creating marker cubes: {str(e)}")
            return False
    
    def get_marker_positions(self):
        """
        Get the current positions and orientations of the marker cubes.
        
        Returns:
            tuple: (dragon_marker_pos, dragon_marker_orient, iss_marker_pos, iss_marker_orient)
        """
        try:
            # Get DragonX marker position
            dragon_marker_pos, dragon_marker_orient = self.get_prim_transform(self.dragon_marker_path)
            
            # Get ISS marker position
            iss_marker_pos, iss_marker_orient = self.get_prim_transform(self.iss_marker_path)
            
            return dragon_marker_pos, dragon_marker_orient, iss_marker_pos, iss_marker_orient
            
        except Exception as e:
            carb.log_error(f"[Docking] Error getting marker positions: {str(e)}")
            return None, None, None, None
    
    def get_imu_positions(self):
        """
        Get the current positions of DragonX and ISS IMUs.
        
        Returns:
            tuple: (dragon_pos, dragon_orient, iss_pos, iss_orient)
        """
        try:
            # Get DragonX IMU position
            dragon_pos, dragon_orient = self.get_prim_transform(self.dragon_imu_path)
            
            # Get ISS IMU docking port position
            iss_pos, iss_orient = self.get_prim_transform(self.iss_imu_path)
            
            return dragon_pos, dragon_orient, iss_pos, iss_orient
            
        except Exception as e:
            carb.log_error(f"[Docking] Error getting IMU positions: {str(e)}")
            return None, None, None, None
    
    def randomize_dragon_position(self):
        """
        Randomize DragonX position and orientation.
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Get current ISS position as reference
            _, _, iss_pos, _ = self.get_imu_positions()
            if not iss_pos:
                carb.log_error("[Docking] Could not get ISS position for randomization")
                return False
            
            # Randomize position within +/- 100 units of ISS
            random_x = iss_pos[0] + random.uniform(-450, -350)
            random_y = iss_pos[1] + random.uniform(150, 250)
            random_z = iss_pos[2] + random.uniform(-100, -10)
            
            # Randomize orientation using Euler angles
            random_roll = random.uniform(0, 2 * math.pi)
            random_pitch = random.uniform(0, 2 * math.pi)
            random_yaw = random.uniform(0, 2 * math.pi)
            
            # Convert Euler angles to quaternion
            cz = math.cos(random_yaw * 0.5)
            sz = math.sin(random_yaw * 0.5)
            cy = math.cos(random_pitch * 0.5)
            sy = math.sin(random_pitch * 0.5)
            cx = math.cos(random_roll * 0.5)
            sx = math.sin(random_roll * 0.5)

            random_w = cx * cy * cz + sx * sy * sz
            random_x_rot = sx * cy * cz - cx * sy * sz
            random_y_rot = cx * sy * cz + sx * cy * sz
            random_z_rot = cx * cy * sz - sx * sy * cz
            
            # Apply randomized transform
            success = self.set_prim_transform(
                self.dragon_prim_path,
                (random_x, random_y, random_z),
                (random_w, random_x_rot, random_y_rot, random_z_rot)
            )
            
            if success:
                carb.log_info(f"[Docking] Randomized DragonX position to ({random_x:.2f}, {random_y:.2f}, {random_z:.2f})")
                return True
            else:
                carb.log_error("[Docking] Failed to apply randomized transform")
                return False
                
        except Exception as e:
            carb.log_error(f"[Docking] Error randomizing DragonX position: {str(e)}")
            return False
    
    def start_simple_docking(self):
        """
        Start simple docking procedure using invisible marker cubes for alignment.
        
        Returns:
            bool: True if docking started successfully, False otherwise
        """
        try:
            # Create invisible marker cubes if not already created
            if not self.markers_created:
                if not self.create_invisible_marker_cubes():
                    carb.log_error("[Docking] Failed to create marker cubes")
                    return False
            
            # Get current positions of DragonX prim and marker cubes
            dragon_pos, dragon_orient = self.get_prim_transform(self.dragon_prim_path)
            dragon_marker_pos, dragon_marker_orient, iss_marker_pos, iss_marker_orient = self.get_marker_positions()
            
            if not dragon_pos or not dragon_marker_pos or not iss_marker_pos:
                carb.log_error("[Docking] Could not get current positions")
                return False
            
            # Phase 1: Approach phase - Keep original DragonX orientation to avoid unwanted rotations
            # Only apply specific docking orientation if absolutely necessary
            self.approach_target_orientation = dragon_orient if dragon_orient else (1.0, 0.0, 0.0, 0.0)
            
            # Calculate the offset between DragonX prim and its marker
            self.dragon_to_marker_offset = (
                dragon_marker_pos[0] - dragon_pos[0],
                dragon_marker_pos[1] - dragon_pos[1],
                dragon_marker_pos[2] - dragon_pos[2]
            )
            
            # Calculate final target position for DragonX prim such that its marker will align with ISS marker
            # We want: dragon_pos + dragon_to_marker_offset = iss_marker_pos (at docking completion)
            # Therefore: target_dragon_pos = iss_marker_pos - dragon_to_marker_offset
            final_target_pos = (
                iss_marker_pos[0] - self.dragon_to_marker_offset[0],
                iss_marker_pos[1] - self.dragon_to_marker_offset[1],
                iss_marker_pos[2] - self.dragon_to_marker_offset[2]
            )
            
            # Set intermediate target position (approach phase) - 10 units away from final position
            approach_distance = 10.0
            # Calculate approach direction (from current position toward final position)
            direction_x = final_target_pos[0] - dragon_pos[0]
            direction_y = final_target_pos[1] - dragon_pos[1]
            direction_z = final_target_pos[2] - dragon_pos[2]
            
            # Normalize direction
            direction_length = math.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
            if direction_length > 0:
                direction_x /= direction_length
                direction_y /= direction_length
                direction_z /= direction_length
            
            # Set approach target position
            self.target_position = (
                final_target_pos[0] - (direction_x * approach_distance),
                final_target_pos[1] - (direction_y * approach_distance),
                final_target_pos[2] - (direction_z * approach_distance)
            )
            
            # Store final docking position for completion phase
            self.final_target_position = final_target_pos
            
            # For approach phase, transition to proper docking orientation
            self.target_orientation = self.approach_target_orientation
            self.start_position = dragon_pos
            self.start_orientation = dragon_orient if dragon_orient else (1.0, 0.0, 0.0, 0.0)
            
            # Store final docking orientation - maintain DragonX at 90, 0, 0 Euler angles (90 degrees around X-axis)
            # Convert 90 degrees X rotation to quaternion: (cos(45°), sin(45°), 0, 0) = (0.707, 0.707, 0, 0)
            self.final_target_orientation = (0.7071068, 0.7071068, 0.0, 0.0)  # 90 degrees X rotation
            self.docking_progress = 0.0
            self.docking_active = True
            self.docking_phase = "approach"  # "approach" or "docking"
            self.docking_speed = self.approach_speed
            
            carb.log_info("[Docking] Marker-based docking started")
            carb.log_info(f"[Docking] Current DragonX position: {dragon_pos}")
            carb.log_info(f"[Docking] Current DragonX marker position: {dragon_marker_pos}")
            carb.log_info(f"[Docking] Target ISS marker position: {iss_marker_pos}")
            carb.log_info(f"[Docking] Marker offset: {self.dragon_to_marker_offset}")
            carb.log_info(f"[Docking] Approach target position: {self.target_position}")
            carb.log_info(f"[Docking] Final target position: {self.final_target_position}")
            carb.log_info(f"[Docking] Target orientation: {self.target_orientation}")
            
            return True
            
        except Exception as e:
            carb.log_error(f"[Docking] Error starting marker-based docking: {str(e)}")
            return False
    
    def update_docking(self, dt):
        """
        Update docking progress with two-phase approach (approach + final docking).
        
        Args:
            dt: Delta time in seconds
            
        Returns:
            bool: True if docking is still active, False if completed
        """
        if not self.docking_active:
            return False
            
        try:
            # Calculate progress
            self.docking_progress += self.docking_speed * dt
            
            if self.docking_progress >= 1.0:
                if self.docking_phase == "approach":
                    # Transition to final docking phase
                    carb.log_info("[Docking] Approach phase completed, starting final docking")
                    self.docking_phase = "final"
                    self.docking_progress = 0.0
                    
                    # Update targets for final docking phase
                    self.start_position = self.target_position
                    self.target_position = self.final_target_position
                    
                    # For final phase, ensure DragonX aligns to the proper 90,0,0 orientation
                    # No need to query current orientation - just use the target directly
                    self.start_orientation = self.target_orientation
                    self.target_orientation = self.final_target_orientation  # Force 90,0,0 orientation
                    
                    # Much slower speed for final docking precision
                    self.docking_speed = self.final_speed
                    
                    carb.log_info(f"[Docking] Final phase - moving to precise docking position: {self.target_position}")
                    carb.log_info(f"[Docking] Final phase - aligning orientation for docking")
                    
                elif self.docking_phase == "final":
                    # Docking completed
                    self.docking_progress = 1.0
                    self.docking_active = False
                    carb.log_info("[Docking] Final docking completed - markers aligned")
                    
                    # Set final position and orientation for perfect docking alignment
                    self.set_prim_transform(
                        self.dragon_prim_path,
                        self.target_position,
                        orientation=self.final_target_orientation  # Apply final docking orientation
                    )
                    
                    # Verify final marker alignment
                    dragon_marker_pos, _, iss_marker_pos, _ = self.get_marker_positions()
                    if dragon_marker_pos and iss_marker_pos:
                        final_marker_distance = math.sqrt(
                            (dragon_marker_pos[0] - iss_marker_pos[0])**2 +
                            (dragon_marker_pos[1] - iss_marker_pos[1])**2 +
                            (dragon_marker_pos[2] - iss_marker_pos[2])**2
                        )
                        carb.log_info(f"[Docking] Final marker alignment distance: {final_marker_distance:.3f} units")
                    
                    # Also verify IMU alignment for reference
                    dragon_imu_pos, _, iss_imu_pos, _ = self.get_imu_positions()
                    if dragon_imu_pos and iss_imu_pos:
                        final_imu_distance = math.sqrt(
                            (dragon_imu_pos[0] - iss_imu_pos[0])**2 +
                            (dragon_imu_pos[1] - iss_imu_pos[1])**2 +
                            (dragon_imu_pos[2] - iss_imu_pos[2])**2
                        )
                        carb.log_info(f"[Docking] Final IMU alignment distance: {final_imu_distance:.3f} units")
                    
                    return False
            
            # Interpolate position based on current phase
            current_pos = (
                self.start_position[0] + (self.target_position[0] - self.start_position[0]) * self.docking_progress,
                self.start_position[1] + (self.target_position[1] - self.start_position[1]) * self.docking_progress,
                self.start_position[2] + (self.target_position[2] - self.start_position[2]) * self.docking_progress
            )
            
            # Orientation handling - preserve original orientation during approach, force correct orientation during final phase
            if self.preserve_original_orientation and self.docking_phase == "approach":
                # Keep the original DragonX orientation during approach phase to prevent unwanted rotations
                current_orient = None  # Let set_prim_transform preserve the existing rotation
                # carb.log_info("[Docking] Preserving original DragonX orientation during approach phase")
            elif self.docking_phase == "final":
                # During final phase, directly apply the target 90,0,0 orientation without any interpolation
                current_orient = self.final_target_orientation
                # Only log once per phase transition, not every frame
                # carb.log_info(f"[Docking] Final phase - applying target orientation directly: {self.final_target_orientation}")
            else:
                current_orient = None  # Fallback to preserving existing orientation
            
            # Apply current transform with position and interpolated orientation
            self.set_prim_transform(
                self.dragon_prim_path,
                current_pos,
                orientation=current_orient  # Apply interpolated orientation for proper docking alignment
            )
            
            # Log progress for each phase
            if self.docking_progress % 0.2 < 0.1:  # Log every 20% progress
                phase_name = "Approach" if self.docking_phase == "approach" else "Final docking"
                carb.log_info(f"[Docking] {phase_name} progress: {self.docking_progress * 100:.1f}%")
            
            return True
            
        except Exception as e:
            carb.log_error(f"[Docking] Error updating docking: {str(e)}")
            self.docking_active = False
            return False
    
    def set_approach_speed(self, speed):
        """
        Set the approach speed for docking.
        
        Args:
            speed (float): Approach speed (0.05 to 2.0)
        """
        self.approach_speed = max(0.05, min(2.0, speed))
        carb.log_info(f"[Docking] Approach speed set to {self.approach_speed:.3f}")
    
    def set_final_speed(self, speed):
        """
        Set the final docking speed.
        
        Args:
            speed (float): Final docking speed (0.02 to 0.3)
        """
        self.final_speed = max(0.02, min(0.3, speed))
        carb.log_info(f"[Docking] Final speed set to {self.final_speed:.3f}")
        
        # Update current docking speed if in final phase
        if hasattr(self, 'docking_phase') and self.docking_phase == "final" and self.docking_active:
            self.docking_speed = self.final_speed
    
    def get_speed_settings(self):
        """
        Get current speed settings.
        
        Returns:
            dict: Speed settings
        """
        return {
            "approach_speed": self.approach_speed,
            "final_speed": self.final_speed,
            "current_speed": getattr(self, 'docking_speed', 0.0)
        }
    
    def stop_docking(self):
        """
        Stop docking procedure.
        """
        self.docking_active = False
        carb.log_info("[Docking] Docking stopped")
    
    def clear_markers(self):
        """
        Clear the invisible marker cubes from the scene.
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return
            
            # Remove DragonX marker
            dragon_marker_prim = stage.GetPrimAtPath(self.dragon_marker_path)
            if dragon_marker_prim and dragon_marker_prim.IsValid():
                stage.RemovePrim(self.dragon_marker_path)
                carb.log_info("[Docking] Removed DragonX marker cube")
            
            # Remove ISS marker
            iss_marker_prim = stage.GetPrimAtPath(self.iss_marker_path)
            if iss_marker_prim and iss_marker_prim.IsValid():
                stage.RemovePrim(self.iss_marker_path)
                carb.log_info("[Docking] Removed ISS marker cube")
            
            self.markers_created = False
            carb.log_info("[Docking] Cleared marker cubes")
            
        except Exception as e:
            carb.log_error(f"[Docking] Error clearing markers: {str(e)}")
    
    def get_docking_status(self):
        """
        Get current docking status.
        
        Returns:
            dict: Docking status information
        """
        return {
            "active": self.docking_active,
            "progress": self.docking_progress,
            "target_position": self.target_position,
            "start_position": self.start_position
        }

class AdvancedDockingController(SimpleDockingController):
    """
    Advanced docking controller that updates all UI aspects during approach and docking.
    Extends SimpleDockingController with continuous UI feedback and enhanced monitoring.
    """
    
    def __init__(self, ui_update_callback=None):
        super().__init__()
        self.ui_update_callback = ui_update_callback
        self.docking_metrics = {
            "distance_to_target": 0.0,
            "approach_velocity": 0.0,
            "alignment_error": 0.0,
            "phase_progress": 0.0,
            "estimated_time_remaining": 0.0,
            "last_update_time": 0.0
        }
        self.previous_position = None
        self.velocity_history = []
        self.max_velocity_history = 10
        
    def set_ui_update_callback(self, callback):
        """Set callback function for UI updates."""
        self.ui_update_callback = callback
        
    def calculate_approach_velocity(self, current_pos, dt):
        """Calculate current approach velocity."""
        if self.previous_position is None:
            self.previous_position = current_pos
            return 0.0
            
        # Calculate velocity vector
        velocity = (
            (current_pos[0] - self.previous_position[0]) / dt,
            (current_pos[1] - self.previous_position[1]) / dt,
            (current_pos[2] - self.previous_position[2]) / dt
        )
        
        # Calculate magnitude
        velocity_magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        
        # Update history for smoothing
        self.velocity_history.append(velocity_magnitude)
        if len(self.velocity_history) > self.max_velocity_history:
            self.velocity_history.pop(0)
            
        # Return smoothed velocity
        return sum(self.velocity_history) / len(self.velocity_history)
        
    def calculate_alignment_error(self, current_pos):
        """Calculate alignment error from ideal trajectory."""
        if not self.target_position or not self.start_position:
            return 0.0
            
        # Calculate ideal position on straight line trajectory
        ideal_pos = (
            self.start_position[0] + (self.target_position[0] - self.start_position[0]) * self.docking_progress,
            self.start_position[1] + (self.target_position[1] - self.start_position[1]) * self.docking_progress,
            self.start_position[2] + (self.target_position[2] - self.start_position[2]) * self.docking_progress
        )
        
        # Calculate deviation from ideal trajectory
        error = math.sqrt(
            (current_pos[0] - ideal_pos[0])**2 +
            (current_pos[1] - ideal_pos[1])**2 +
            (current_pos[2] - ideal_pos[2])**2
        )
        
        return error
        
    def estimate_time_remaining(self):
        """Estimate time remaining for docking completion."""
        if self.docking_progress >= 1.0:
            return 0.0
            
        remaining_progress = 1.0 - self.docking_progress
        if self.docking_speed > 0:
            return remaining_progress / self.docking_speed
        return 0.0
        
    def update_docking_metrics(self, current_pos, dt):
        """Update all docking metrics for UI display."""
        import time
        
        # Distance to target
        if self.target_position:
            self.docking_metrics["distance_to_target"] = math.sqrt(
                (current_pos[0] - self.target_position[0])**2 +
                (current_pos[1] - self.target_position[1])**2 +
                (current_pos[2] - self.target_position[2])**2
            )
        
        # Approach velocity
        self.docking_metrics["approach_velocity"] = self.calculate_approach_velocity(current_pos, dt)
        
        # Alignment error
        self.docking_metrics["alignment_error"] = self.calculate_alignment_error(current_pos)
        
        # Phase progress
        self.docking_metrics["phase_progress"] = self.docking_progress
        
        # Estimated time remaining
        self.docking_metrics["estimated_time_remaining"] = self.estimate_time_remaining()
        
        # Update timestamp
        self.docking_metrics["last_update_time"] = time.time()
        
        # Store current position for next iteration
        self.previous_position = current_pos
        
    def update_ui_display(self):
        """Update UI with current docking metrics."""
        if self.ui_update_callback:
            # Get current position for real-time updates
            current_pos, _ = self.get_prim_transform(self.dragon_prim_path)
            
            ui_data = {
                "phase": self.docking_phase if hasattr(self, 'docking_phase') else "inactive",
                "active": self.docking_active,
                "progress": self.docking_progress * 100,
                "distance": self.docking_metrics["distance_to_target"],
                "velocity": self.docking_metrics["approach_velocity"],
                "alignment_error": self.docking_metrics["alignment_error"],
                "time_remaining": self.docking_metrics["estimated_time_remaining"],
                "speed_setting": self.docking_speed,
                "current_position": current_pos if current_pos else (0, 0, 0),
                "target_position": self.target_position if self.target_position else (0, 0, 0),
                "metrics": self.docking_metrics.copy()
            }
            
            try:
                self.ui_update_callback(ui_data)
            except Exception as e:
                carb.log_error(f"[Advanced Docking] Error updating UI: {str(e)}")
                
    def start_advanced_docking(self):
        """Start advanced docking with enhanced monitoring."""
        # Use the same logic as simple docking but with enhanced tracking
        success = self.start_simple_docking()
        if success:
            # Initialize tracking variables
            self.previous_position = None
            self.velocity_history = []
            carb.log_info("[Advanced Docking] Started with enhanced UI monitoring")
            
            # Initial UI update
            self.update_ui_display()
        return success
        
    def update_docking(self, dt):
        """Enhanced docking update with continuous UI updates."""
        if not self.docking_active:
            return False
            
        try:
            # Get current position for metrics calculation
            current_pos, _ = self.get_prim_transform(self.dragon_prim_path)
            if current_pos:
                # Update metrics before position change
                self.update_docking_metrics(current_pos, dt)
            
            # Call parent update method
            still_active = super().update_docking(dt)
            
            # Update UI with latest metrics
            self.update_ui_display()
            
            # Enhanced logging for advanced mode
            if self.docking_progress > 0 and int(self.docking_progress * 100) % 10 == 0:
                phase_name = "Approach" if self.docking_phase == "approach" else "Final docking"
                carb.log_info(
                    f"[Advanced Docking] {phase_name} - Progress: {self.docking_progress * 100:.1f}%, "
                    f"Distance: {self.docking_metrics['distance_to_target']:.2f}, "
                    f"Velocity: {self.docking_metrics['approach_velocity']:.3f}, "
                    f"Alignment Error: {self.docking_metrics['alignment_error']:.3f}, "
                    f"ETA: {self.docking_metrics['estimated_time_remaining']:.1f}s"
                )
            
            return still_active
            
        except Exception as e:
            carb.log_error(f"[Advanced Docking] Error updating docking: {str(e)}")
            self.docking_active = False
            return False
            
    def stop_docking(self):
        """Stop docking with final UI update."""
        super().stop_docking()
        
        # Final UI update
        self.update_ui_display()
        
        # Reset tracking variables
        self.previous_position = None
        self.velocity_history = []
        self.docking_metrics = {
            "distance_to_target": 0.0,
            "approach_velocity": 0.0,
            "alignment_error": 0.0,
            "phase_progress": 0.0,
            "estimated_time_remaining": 0.0,
            "last_update_time": 0.0
        }
        
        carb.log_info("[Advanced Docking] Stopped with metrics reset")
        
    def get_advanced_docking_status(self):
        """Get comprehensive docking status for advanced mode."""
        base_status = self.get_docking_status()
        base_status.update({
            "mode": "Advanced",
            "phase": getattr(self, 'docking_phase', 'inactive'),
            "metrics": self.docking_metrics.copy(),
            "ui_active": self.ui_update_callback is not None
        })
        return base_status

# Utility functions for docking operations
def get_dragon_iss_distance():
    """
    Get the distance between DragonX and ISS.
    
    Returns:
        float: Distance in units, or None if failed
    """
    try:
        controller = SimpleDockingController()
        dragon_pos, _, iss_pos, _ = controller.get_imu_positions()
        
        if not dragon_pos or not iss_pos:
            return None
            
        distance = np.sqrt(
            (dragon_pos[0] - iss_pos[0])**2 +
            (dragon_pos[1] - iss_pos[1])**2 +
            (dragon_pos[2] - iss_pos[2])**2
        )
        
        return distance
        
    except Exception as e:
        carb.log_error(f"[Docking] Error calculating distance: {str(e)}")
        return None

def get_docking_alignment():
    """
    Get alignment information between DragonX and ISS.
    
    Returns:
        dict: Alignment information
    """
    try:
        controller = SimpleDockingController()
        dragon_pos, dragon_orient, iss_pos, iss_orient = controller.get_imu_positions()
        
        if not dragon_pos or not iss_pos:
            return {}
            
        # Calculate relative position
        relative_pos = (
            iss_pos[0] - dragon_pos[0],
            iss_pos[1] - dragon_pos[1],
            iss_pos[2] - dragon_pos[2]
        )
        
        # Calculate distance
        distance = np.sqrt(relative_pos[0]**2 + relative_pos[1]**2 + relative_pos[2]**2)
        
        alignment_info = {
            "distance": distance,
            "relative_position": relative_pos,
            "dragon_position": dragon_pos,
            "iss_position": iss_pos,
            "aligned": distance < 10.0  # Consider aligned if within 10 units
        }
        
        return alignment_info
        
    except Exception as e:
        carb.log_error(f"[Docking] Error calculating alignment: {str(e)}")
        return {}
