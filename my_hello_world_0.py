# # Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
# #
# # NVIDIA CORPORATION and its licensors retain all intellectual property
# # and proprietary rights in and to this software, related documentation
# # and any modifications thereto. Any use, reproduction, disclosure or
# # distribution of this software and related documentation without an express
# # license agreement from NVIDIA CORPORATION is strictly prohibited.
# #
# from omni.isaac.examples.base_sample import BaseSample
# from omni.isaac.core.utils.types import ArticulationAction
# from omni.isaac.core.articulations import Articulation
# from omni.isaac.core.articulations import ArticulationView

# import numpy as np
# from omni.isaac.core.objects import DynamicCuboid

# from omni.isaac.core.utils.nucleus import get_assets_root_path  # get the root path of the assets
# from omni.isaac.core.utils.stage import add_reference_to_stage  # 
# from pxr import UsdPhysics
# from omni.isaac.core.robots import Robot  # manage robots
# import carb  #logging

# from omni.isaac.examples.user_examples.control import PIDController
# import time
# # # Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


# class MyHelloWorld(BaseSample):
#     def __init__(self) -> None:
#         self.prims = None
#         super().__init__()
#         return

#     def setup_scene(self):

#         world = self.get_world()
#         world.scene.add_default_ground_plane()

#         assets_root_path = get_assets_root_path()  # get the path
#         if assets_root_path is None:
#             # logging
#             carb.log_error('could not find path!')
        
#         asset_path = '/home/hk/drone/hummingbird.usd'
#         add_reference_to_stage(usd_path=asset_path, prim_path="/World/hummingbird")
#         drone_robot = world.scene.add(Robot(prim_path="/World/hummingbird", 
#                                              name="fancy_drone"))
#         #----------------------------------------------------------------------------------
#         self.prim = Articulation(prim_path="/World/hummingbird",       # set collision
#                                              name="fancy_drone")
#         self.prim.set_enabled_self_collisions(True)
#         #----------------------------------------------------------------------------------
#         self.prims = ArticulationView(prim_paths_expr="/World/hummingbird",
#                                              name="fancy_drone")
#         if self.prims is None:
#             raise ValueError("Failed to initialize ArticulationView")
        

#         # fancy_cube = world.scene.add(
#         #     DynamicCuboid(
#         #         prim_path='/World/random_cube',
#         #         name='fancy_cube',
#         #         position=np.array([0, 0, 1.0]),
#         #         scale=np.array([0.5015, 0.5015, 0.5015]),
#         #         color=np.array([0, 0, 1.0])
#         #     )
#         # ) 
        
#         return

#     async def setup_post_load(self):
#         self._world = self.get_world()
#         self._drone = self._world.scene.get_object('fancy_drone')
#         print('degrees of freedom'+ str(self._drone.num_dof))
#         self._drone_articulation_controller = self._drone.get_articulation_controller()
#         # self._world.add_physics_callback('sending_actions', callback_fn=self.send_hummingbird_actions)
#         self._world.add_physics_callback('sending_actions', callback_fn=self.PID_control_fly)

#         return
    
        
#     def send_hummingbird_actions(self, step_size):
#         self._drone_articulation_controller.apply_action(ArticulationAction(
#             joint_positions=None,
#             joint_efforts=43.868*np.array([1, 1, 1, 1]),
#             # joint_velocities=100 * np.random.rand(2,)
#             # joint_velocities= 40000000*np.array([1, 0, 0, 0]),
#             joint_velocities= None
#         ))
#         #----------------------------------------------------------------------------------
#         position, orientation = self._drone.get_world_pose()  # get position and orientation
#         x, y, z = position
#         a, b, c, d = orientation
#         print("position_z = " + str(z))
#         print("orientation = " + str(a))
#         #----------------------------------------------------------------------------------
        
#     def PID_control_fly(self):
#         Kp, Ki, Kd = 1.0, 0.01, 0.1  # PID 增益参数
#         pid = PIDController(Kp, Ki, Kd)

#         drone_sim = DroneSimulator(self.prims)
#         target_height = 10.0
#         drone_sim.set_target_height(target_height)

#         current_height = drone_sim.get_current_height()
#         thrust = pid.compute(drone_sim.target_height, current_height, step_size)
#         thrust = np.clip(thrust, 0, 50.0)  # 假设最大推力是50N

#         # 应用计算的推力
#         drone_sim.apply_thrust(thrust)
        
#         # 打印当前高度和推力
#         print(f"Height: {current_height:.2f}m, Thrust: {thrust:.2f}N")
#         # Kp, Ki, Kd = 1.0, 0.01, 0.1  # PID 增益参数，可以根据需要调整
#         # pid = PIDController(Kp, Ki, Kd)

#         # drone_sim = DroneSimulator(self.prims)   #111
#         # if drone_sim is None:
#         #     raise ValueError("Failed to initialize drone_sim")

#         # target_height = 10.0  # 设定无人机的目标高度
#         # drone_sim.set_target_height(target_height)

#         # dt = 0.01
#         # max_thrust = 50.0

#         # for t in np.arange(0, 10, dt):  # 模拟10秒的时间
#         #     current_height = drone_sim.get_current_height()  # 获取当前高度
#         #     thrust = pid.compute(drone_sim.target_height, current_height, dt)  # 计算PID输出的推力
            
#         #     # 将推力限制在最大推力范围内
#         #     thrust = np.clip(thrust, 0, max_thrust)
            
#         #     # 应用推力控制无人机
#         #     drone_sim.apply_thrust(thrust)
            
#         #     # 打印当前高度和推力信息
#         #     print(f"Time: {t:.2f}s, Height: {current_height:.2f}m, Thrust: {thrust:.2f}N")
            
#         #     # 模拟时间流逝
#         #     time.sleep(dt)
    

#     async def setup_pre_reset(self):
#         return

#     async def setup_post_reset(self):
#         return

#     def world_cleanup(self):
#         return


# class DroneSimulator:
#     def __init__(self, drone_articulation_view):   # 111
#         self.drone = drone_articulation_view
#         if self.drone is None:
#             raise ValueError("DroneSimulator initialized with None drone")
#         self.current_height = 0.0
#         self.target_height = 0.0
    
#     def set_target_height(self, height):
#         self.target_height = height
    
#     def get_current_height(self):
#         if not self.drone:
#             raise AttributeError("Drone is not initialized")
#         # 获取无人机当前高度，假设高度由世界坐标系中的z值决定
#         positions, _ = self.drone.get_world_poses()
#         self.current_height = positions[0][2]  # 假设无人机的z坐标是高度
#         return self.current_height
    
#     def apply_thrust(self, thrust):
#         # # 将计算出的推力应用到无人机（在真实场景中，需要通过物理引擎来实现）
#         # # 这里只是一个简单的示例
#         # self.drone.apply_force(np.array([0.0, 0.0, thrust]))
#         action = ArticulationAction(
#             joint_positions=None,  # 这里不设置位置
#             joint_velocities=None,  # 这里不设置速度
#             joint_efforts=np.array([thrust, thrust, thrust, thrust])  # 施加推力到四个推进器
#         )
#         self.drone.apply_action(action)