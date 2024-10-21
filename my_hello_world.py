# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# PPO中的Actor-Critic网络
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.actor = nn.Linear(128, action_dim)
        self.critic = nn.Linear(128, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        action_probs = torch.tanh(self.actor(x))  # 输出推力控制 [-1, 1]
        state_value = self.critic(x)
        return action_probs, state_value

# PPO强化学习算法
class PPOAgent:
    def __init__(self, state_dim, action_dim):
        self.model = ActorCritic(state_dim, action_dim)
        self.optimizer = optim.Adam(self.model.parameters(), lr=1e-4)
        self.gamma = 0.99  # 折扣因子
        self.eps_clip = 0.2  # PPO中的剪切参数
        self.memory = []
        self.min_thrust = 0.0  # 电机的最小推力
        self.max_thrust = 98.0  # 电机的最大推力

    def select_action(self, state):
        state = torch.tensor(state, dtype=torch.float32)
        action_probs, _ = self.model(state)

        # 将推力从 [-1, 1] 映射到 [min_thrust, max_thrust]
        action = self.min_thrust + (action_probs + 1) * (self.max_thrust - self.min_thrust) / 2
        return action.detach().numpy()

    def store_transition(self, transition):
        self.memory.append(transition)

    def update(self):
        states, actions, rewards, next_states, dones = zip(*self.memory)
        states = torch.tensor(states, dtype=torch.float32)
        actions = torch.tensor(actions, dtype=torch.float32)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        dones = torch.tensor(dones, dtype=torch.float32)

        # 计算优势函数 (GAE-Lambda)()
        _, values = self.model(states)
        _, next_values = self.model(torch.tensor(next_states, dtype=torch.float32))
        td_target = rewards + self.gamma * next_values * (1 - dones)
        delta = td_target - values
        advantage = delta.detach()

        # 更新策略
        for _ in range(10):  # PPO多次更新
            old_action_probs, _ = self.model(states)
            new_action_probs, _ = self.model(states)
            ratio = torch.exp(new_action_probs - old_action_probs)

            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1 - self.eps_clip, 1 + self.eps_clip) * advantage
            actor_loss = -torch.min(surr1, surr2).mean()

            critic_loss = nn.functional.mse_loss(values, td_target.detach())
            loss = actor_loss + critic_loss

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        self.memory = []


class MyHelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.target_height = 5.0  # 目标悬停高度
        self.agent = PPOAgent(state_dim=6, action_dim=4)  # 状态维度（位置和姿态），动作维度（四个电机推力）
        self.current_step = 0
        self.episode_rewards = []

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        asset_path = '/home/hk/drone/hummingbird.usd'
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/hummingbird")
        drone_robot = world.scene.add(Robot(prim_path="/World/hummingbird", name="fancy_drone"))

        self._drone = world.scene.get_object('fancy_drone')
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._drone = self._world.scene.get_object('fancy_drone')
        self._drone_articulation_controller = self._drone.get_articulation_controller()
        self._world.add_physics_callback('sending_actions', callback_fn=self.send_hummingbird_actions)
        return

    def send_hummingbird_actions(self, step_size):
        position, orientation = self._drone.get_world_pose()
        roll, pitch, yaw = self.get_euler_from_quaternion(orientation)  
        x, y, z = position
        state = [x, y, z, roll, pitch, yaw]

        # 选择动作（推力）
        action = self.agent.select_action(state)
        motor1, motor2, motor3, motor4 = action  # 四个电机推力分配

        joint_efforts = np.array([motor1, motor2, motor3, motor4])

        self._drone_articulation_controller.apply_action(ArticulationAction(
            joint_positions=None,
            joint_efforts=joint_efforts,
            joint_velocities=None
        ))

        # 奖励函数
        height_error = np.abs(self.target_height - z)
        reward = -height_error * 10  # 奖励为负的高度误差

        done = z < 0.1  # 无人机碰撞地面
        self.agent.store_transition((state, action, reward, state, done))

        # 每步打印信息
        print(f"Current height: {z}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        print(f"Motor thrusts: motor1={motor1}, motor2={motor2}, motor3={motor3}, motor4={motor4}")
        print(f"Reward: {reward}")

        # 每隔一定步数更新策略
        self.current_step += 1
        if self.current_step % 10 == 0:
            self.agent.update()

        if done:
            print("Collision detected. Resetting environment...")
            self.reset_training()

    def reset_training(self):
        self._drone.set_world_pose([0, 0, 0.2], [0, 0, 0, 1])  # 重置无人机位置
        self.agent.memory = []  # 清空经验池
        self.current_step = 0

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

