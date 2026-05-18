# 机器人系统工程师岗位技术考核任务说明

## 1.1 任务概述
开发一个基于Web的机械臂仿真控制系统，实现以下核心功能：
通过PyBullet加载并仿真机械臂模型
支持关节空间和任务空间的运动控制
实时获取并显示机械臂末端相机的RGB/深度图像
通过Web界面进行交互式控制

## 1.2 技术栈要求

后端技术栈（Python）
技术组件   要求说明   版本建议
仿真引擎   PyBullet（核心仿真）   ≥3.2.5

Web框架   FastAPI   ≥0.100.0

WebSocket   FastAPI内置WebSocket   -

运动学库   Pinocchio 或 PyBullet内置功能   -

图像处理   OpenCV、NumPy、PIL   -

异步处理   asyncio   -

前端技术栈（JavaScript）
技术组件   要求说明
核心技术   HTML5、CSS3、JavaScript (ES6+)

图像显示   Canvas 或 img标签

UI框架   原生或主流框架（Vue.js/React）

WebSocket客户端   原生WebSocket API

## 1.3 PyBullet核心功能说明

### 1.3.1 PyBullet图像仿真能力
PyBullet完全支持高质量的图像仿真，包括：
✅ RGB图像渲染：支持彩色图像生成
✅ 深度图像渲染：支持深度信息获取

### 1.3.2 关键API参考
```python
# 相机图像获取
width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    width=640,
    height=480,
    viewMatrix=view_matrix,
    projectionMatrix=proj_matrix,
    renderer=p.ER_BULLET_HARDWARE_OPENGL
)

# 运动控制
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=joint_idx,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle
)
```

## 2.1 详细任务分解

## 2.1.1：技术学习

学习内容：
PyBullet基础
   物理引擎原理与PyBullet架构
   仿真环境初始化（GUI/DIRECT模式）
   基本物理参数设置（重力、时间步长）

机器人运动学
   正向运动学（FK）原理
   逆向运动学（IK）原理
   关节空间与任务空间概念

FastAPI与WebSocket
   RESTful API设计
   WebSocket实时通信原理
   异步编程基础

## 2.1.2：环境搭建与URDF加载

实例代码：
```python
import pybullet as p
import pybullet_data
import time

连接仿真环境
physicsClient = p.connect(p.GUI)  # 或 p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1./240.)

加载机器人URDF
robot_id = p.loadURDF("provided_robot.urdf", [0, 0, 0.5])

验证加载成功
num_joints = p.getNumJoints(robot_id)
print(f"机器人关节数量: {num_joints}")

运行简单仿真
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)
```

## 2.2：运动学控制模块

功能要求：

关节空间控制
   实现各关节独立角度控制
   支持关节角度范围限制
   实时反馈当前关节状态

任务空间控制
   实现末端执行器XYZ坐标控制
   集成逆向运动学求解器
   处理奇异点和关节限位

参考实现：
```python
import pybullet as p
import numpy as np
from ikpy.chain import Chain

class RobotController:
    def init(self, robot_id, urdf_path):
        self.robot_id = robot_id
        self.chain = Chain.from_urdf_file(urdf_path)
        
    def set_joint_angles(self, joint_angles):
        """关节空间控制"""
        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle,
                force=100
            )
    
    def move_to_position(self, target_position):
        """任务空间控制"""
        # 使用IK求解
        joint_angles = self.chain.inverse_kinematics(target_position)
        
        # 应用到机器人
        self.set_joint_angles(joint_angles[1:])  # 跳过base_link
        
        return joint_angles
    
    def get_end_effector_pose(self):
        """获取末端执行器位姿"""
        end_effector_state = p.getLinkState(self.robot_id, self.end_effector_link)
        position = end_effector_state[0]
        orientation = end_effector_state[1]
        return position, orientation
```

## 2.3：相机图像获取与传输

功能要求：

相机配置
   假设在机器人End Effector上安装了一个虚拟摄像头
   配置相机参数（FOV、near/far plane）

图像获取
   实时获取RGB图像
   实时获取深度图像

图像编码与传输
   Base64编码
   传输效率优化
   帧率控制

## 2.4: Web界面与系统集成

### 2.4.1：后端API开发

功能要求：

RESTful API
   机器人状态查询接口
   控制指令接收接口
   系统信息接口

WebSocket服务
   控制指令WebSocket端点
   图像流WebSocket端点
   状态更新WebSocket端点

### 2.4.2：前端Web界面开发

功能要求：

控制面板
   关节空间控制：滑块组控制各关节角度
   任务空间控制：XYZ roll yaw pitch按钮调节机械臂末端
   控制模式切换（关节/任务空间）

图像显示区
   实时显示RGB图像和深度图像

状态监控面板
   当前关节角度实时显示
   末端执行器位置显示

