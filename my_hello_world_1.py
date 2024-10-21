# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation

import numpy as np
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot  # manage robots
import carb  # logging

class MyHelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.target_height = 5.0  # Target hover height
        self.target_orientation = np.array([0.0, 0.0, 0.0])  # Target orientation (roll, pitch, yaw)
        self.pid_height = PID(kp=2.0, ki=0.0, kd=0.3)  # Initialize PID for height control
        self.pid_roll = PID(kp=0.5, ki=0.1, kd=0.3)  # PID for roll
        self.pid_pitch = PID(kp=1.5, ki=0.0, kd=0.1)  # PID for pitch
        self.pid_yaw = PID(kp=0.2, ki=0.01, kd=0.1)  # PID for yaw control

        # 111
        self.current_thrust = 0.0  # 初始化当前推力
        self.max_thrust = 5.0  # 最大推力上限，用于控制逐渐增加的速度
        self.thrust_increment = 0.0001  # 每次迭代增加的推力量
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error('could not find path!')

        asset_path = '/home/hk/drone/hummingbird.usd'
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/hummingbird")
        drone_robot = world.scene.add(Robot(prim_path="/World/hummingbird", name="fancy_drone"))

        prim = Articulation(prim_path="/World/hummingbird", name="fancy_drone")
        prim.set_enabled_self_collisions(True)
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._drone = self._world.scene.get_object('fancy_drone')
        print('degrees of freedom' + str(self._drone.num_dof))

        self._drone_articulation_controller = self._drone.get_articulation_controller()
        self._world.add_physics_callback('sending_actions', callback_fn=self.send_hummingbird_actions)
        return

    def send_hummingbird_actions(self, step_size):
        # Get current position and orientation
        position, orientation = self._drone.get_world_pose()
        roll, pitch, yaw = self.get_euler_from_quaternion(orientation)  # Convert quaternion to roll, pitch, yaw
        x, y, z = position

        # Calculate control efforts for height and orientation
        height_error = self.target_height - z
        roll_error = self.target_orientation[0] - roll
        pitch_error = self.target_orientation[1] - pitch
        yaw_error = self.target_orientation[2] - yaw

        thrust = self.pid_height.update(height_error) * 1

        # 111
        if self.current_thrust < thrust:
            self.current_thrust += self.thrust_increment  # 缓慢增加推力
            self.current_thrust = min(self.current_thrust, self.max_thrust)  # 限制推力的最大值
        else:
            self.current_thrust = thrust

        roll_control = self.pid_roll.update(roll_error)
        pitch_control = self.pid_pitch.update(pitch_error)
        yaw_control = self.pid_yaw.update(yaw_error)

        # 222：调整偏航控制
        # 调整推力分配，避免绕 z 轴旋转
        yaw_factor = 0.25

        motor1 = self.current_thrust + roll_control - yaw_control * yaw_factor
        motor2 = self.current_thrust - roll_control + yaw_control * yaw_factor
        motor3 = self.current_thrust + pitch_control - yaw_control * yaw_factor
        motor4 = self.current_thrust - pitch_control + yaw_control * yaw_factor

        # 333
        # 限制每个电机的推力范围，避免推力过大
        motor1 = np.clip(motor1, 0, self.max_thrust)
        motor2 = np.clip(motor2, 0, self.max_thrust)
        motor3 = np.clip(motor3, 0, self.max_thrust)
        motor4 = np.clip(motor4, 0, self.max_thrust)


        # Combine control efforts into a force/effort vector applied to the drone
        # joint_efforts = np.array([motor1, motor2, motor3, motor4])
        joint_efforts = np.array([0, 0, 0.01, 0])

        # Apply the control effort to the drone's motors
        self._drone_articulation_controller.apply_action(ArticulationAction(
            joint_positions=None,
            joint_efforts=joint_efforts,
            joint_velocities=None
        ))

        # Print debugging information
        print(f"Current height: {z}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        print(f"Motor thrusts: motor1={motor1}, motor2={motor2}, motor3={motor3}, motor4={motor4}")
        print(f"Roll control: {roll_control}, Pitch control: {pitch_control}, Yaw control: {yaw_control}")

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return

    def get_euler_from_quaternion(self, quat):
        """Convert quaternion to roll, pitch, yaw (Euler angles)"""
        x, y, z, w = quat
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

class PID:
    """PID Controller class"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt=0.01):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
