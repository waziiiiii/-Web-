# 机械臂仿真控制系统

基于 PyBullet + FastAPI + Three.js 的 Web 端机械臂仿真控制系统，支持关节空间/任务空间控制、末端相机渲染、碰撞检测等功能。

---

## 目录

- [快速开始]
- [1. 系统架构]()
- [2. 技术栈]()
- [3. 项目结构]()
- [4. 功能实现方式]()
- [5. 配置管理]()
- [6. API 接口]()
- [7. 后期迭代接口]()
- [8. 关键设计决策]()

---

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 启动服务
python main.py
# 或双击 start.bat

# 打开浏览器
# http://localhost:8000
```

### 启动后的默认状态

启动服务器后，系统默认加载以下状态：

- **控制模式**：精确模式（`precise`）— 可通过顶部状态栏的下拉菜单实时切换为物理仿真模式
- **机械臂**：Home 位姿 `[0.0, 0.5, 0.0, -0.5, 0.0, 0.3, 0.0]`（7 个关节初始角度）
- **工作空间物体**：4 个球体 + 4 个方块，散落在机械臂周围（距离底座 >0.15m）
- **碰撞检测**：已启用（物体碰撞 + 关节自碰撞 + 地面碰撞）
- **仿真引擎**：PyBullet `p.DIRECT` 模式（无 GUI 窗口，纯后台物理计算）

### 控制模式说明

系统提供**两种控制模式**，可通过顶部状态栏的下拉菜单实时切换（无需重启服务器）：

#### 1. 精确模式（`precise`）

- **核心原理**：调用 `p.resetJointState()` 直接写入关节角度值，**跳过物理求解**
- **运动方式**：瞬移到目标位置，无加速/减速过程
- **物理特性**：❌ 不受重力、惯性、阻尼影响；关节不会因重力下垂
- **精度**：✅ 零误差，设置的值即最终值
- **碰撞检测**：✅ 支持（物体碰撞 + 自碰撞 + 地面碰撞）
- **适用场景**：
  - 精确控制验证（如 IK 求解结果验证）
  - 末端相机拍摄（需要精确到达指定位姿）
  - 调试和测试（快速定位到目标关节角度）
  - 场景搭建（精确定位机械臂姿态后拖拽物体）

#### 2. 物理仿真模式（`motor`）— 默认模式

- **核心原理**：调用 `p.setJointMotorControl2()` + `POSITION_CONTROL`，通过电机控制器驱动关节
- **运动方式**：平滑运动到目标位置，有加速/减速过程，存在运动延迟
- **物理特性**：✅ 完整物理仿真——重力（-9.81 m/s²）、惯性、阻尼均生效；松开的关节会因重力下垂
- **精度**：⚠️ 存在微小稳态误差（由 kp/kd 增益参数决定）
- **电机参数**：最大力矩 5000N、位置增益 kp=0.3、阻尼增益 kd=1.0
- **碰撞检测**：✅ 支持（物体碰撞 + 自碰撞 + 地面碰撞）
- **适用场景**：
  - 物理仿真演示（展示真实电机控制行为）
  - 运动学/动力学验证（观察关节响应、超调、振荡）
  - 碰撞效果展示（物体碰撞后产生物理反馈力）
  - 抓取策略评估（考虑重力对物体和机械臂的影响）

#### 模式对比表

| 特性 | 精确模式（`precise`） | 物理仿真模式（`motor`） |
|:--|:--|:--|
| **API 参数** | `"mode": "precise"` | `"mode": "motor"` |
| **PyBullet API** | `p.resetJointState()` | `p.setJointMotorControl2()` |
| **运动方式** | 瞬移到位 | 平滑运动（有延迟） |
| **物理特性** | ❌ 无重力/惯性影响 | ✅ 重力、惯性、阻尼均生效 |
| **精度** | ✅ 精确到位（零误差） | ⚠️ 有微小稳态误差 |
| **电机参数** | 无 | force=5000N, kp=0.3, kd=1.0 |
| **碰撞检测** | ✅ 支持 | ✅ 支持 |
| **默认状态** | ✅ **默认启用** | 需手动切换 |
| **适用场景** | 精确控制、IK 验证、相机拍摄 | 物理仿真演示、动力学验证 |

> **切换方式**：两种模式共用同一套前端 UI 和 API，切换时无需重启服务器。默认为精确模式以保证控制精度和开箱即用体验。

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        浏览器前端 (Three.js)                         │
│                                                                     │
│  ┌──────────────┐  ┌──────────────────┐  ┌───────────────────────┐ │
│  │  控制面板      │  │  3D 实时可视化     │  │  相机 2x2 网格         │ │
│  │  - 关节滑块    │  │  - STL 网格渲染    │  │  - PyBullet RGB       │ │
│  │  - IK 控制    │  │  - 轨道控制       │  │  - PyBullet 深度       │ │
│  │  - 模式切换    │  │  - 拖拽物体       │  │  - Three.js 末端 RGB  │ │
│  │  - 碰撞指示    │  │  - 碰撞高亮       │  │  - Three.js 末端深度   │ │
│  └──────┬───────┘  └────────┬─────────┘  └───────────┬───────────┘ │
│         │                   │                        │             │
└─────────┼───────────────────┼────────────────────────┼─────────────┘
          │ WebSocket/REST    │ WebSocket (60FPS)      │ REST + WS
          ▼                   ▼                        ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      FastAPI 后端 (main.py)                          │
│                                                                     │
│  ┌───────────────┐  ┌─────────────────┐  ┌───────────────────────┐ │
│  │  /ws/control   │  │  /ws/stream      │  │  REST API             │ │
│  │  控制命令通道   │  │  60FPS 状态推送   │  │  /api/joint/*         │ │
│  └───────┬───────┘  └────────┬────────┘  │  /api/ik              │ │
│          │                   │            │  /api/camera/*        │ │
│          ▼                   ▼            │  /api/collision       │ │
│  ┌──────────────────────────────────────┐ │  /api/control_mode    │ │
│  │       RobotController 核心控制器      │ │  /api/workspace/*     │ │
│  │                                      │ └───────────────────────┘ │
│  │  ┌─────────────────────────────────┐ │                          │
│  │  │ PyBullet 物理仿真 (p.DIRECT)     │ │                          │
│  │  │ - 7轴机械臂 URDF                │ │                          │
│  │  │ - 60FPS 后台仿真循环             │ │                          │
│  │  │ - 双控制模式 (精确/电机)         │ │                          │
│  │  │ - 碰撞检测 (物体/自碰撞/地面)    │ │                          │
│  │  └─────────────────────────────────┘ │                          │
│  │  ┌─────────────────────────────────┐ │                          │
│  │  │ 按需相机 (独立 PyBullet 实例)     │ │                          │
│  │  │ - 160×120 渲染(可在config.py中修改) │ │                        │
│  │  │ - OpenCV JPEG 编码               │ │                          │
│  │  │ - 深度图 JET 热力图              │ │                          │
│  │  │ - 自动保存到 captures/           │ │                          │
│  │  └─────────────────────────────────┘ │                          │
│  └──────────────────────────────────────┘                          │
└─────────────────────────────────────────────────────────────────────┘
```

### 数据流

```
┌─────────┐    set_joint    ┌──────────┐    cmd_queue    ┌──────────────┐
│  前端    │ ──WebSocket──▶ │ FastAPI  │ ──────────────▶ │ RobotControl │
│  滑块   │                │ /ws/ctrl │                 │   _loop()    │
└─────────┘                └──────────┘                 └──────┬───────┘
     ▲                                                        │
     │              60FPS state stream                        ▼
     │         ◀── /ws/stream ──────◀──── _capture_state()
     │
     │   拍摄请求 POST /api/camera/capture
     │ ──────────────────────────────────────▶  capture_single_frame()
     │                                            │
     │   轮询 GET /api/camera/status             ▼
     │ ◀──────────────────────────────  独立 PyBullet 渲染线程
     │                                            │
     │   获取图像 GET /api/camera                ▼
     │ ◀──────────────────────────────  保存到 captures/
     │
     │   碰撞查询 GET /api/collision
     │ ◀──────────────────────────────  _detect_collisions()
```

---

## 2. 技术栈

### 后端 (Python)

| 组件 | 技术 | 版本 | 说明 |
|------|------|------|------|
| 仿真引擎 | PyBullet | ≥3.2.5 | 物理仿真 + 碰撞检测 + IK 求解 + 相机渲染 |
| Web 框架 | FastAPI | ≥0.100.0 | REST API + WebSocket 端点 |
| 异步服务器 | Uvicorn | ≥0.23.0 | ASGI 服务器 |
| 图像处理 | OpenCV (cv2) | ≥4.8.0 | JPEG 编码 + 深度图热力图着色 |
| 数值计算 | NumPy | ≥1.24.0 | 数组操作 + 深度图转换 |

### 前端 (JavaScript)

| 组件 | 技术 | 说明 |
|------|------|------|
| 3D 渲染 | Three.js r128 | 主场景 + 末端相机 + 深度相机 |
| 网格加载 | STLLoader | 加载 8 个 STL 机械臂 link 网格 |
| 轨道控制 | OrbitControls | 鼠标旋转/缩放/平移 |
| 拖拽控制 | DragControls | 拖拽工作空间物体 |
| 实时通信 | 原生 WebSocket API | 控制指令 + 60FPS 状态流 |
| UI 框架 | 原生 HTML/CSS/JS | GitHub Dark 主题 |

---

## 3. 项目结构

```
EvaluateTask/
├── config.py                   # 集中配置（所有可调参数）
├── main.py                     # FastAPI 后端入口
├── robot_controller.py         # PyBullet 机器人控制器（核心）
├── requirements.txt            # Python 依赖
├── start.bat                   # Windows 一键启动
├── autolife_desktop/
│   ├── urdfs/
│   │   └── robot_v0_1.urdf     # 7 轴机械臂 URDF 模型
│   └── meshes/
│       └── robot_v0_1/         # 8 个 STL 网格文件
├── captures/                   # 拍照输出（按时间戳分目录）
│   └── YYYYMMDD_HHMMSS/
│       ├── rgb.jpg
│       └── depth.jpg
└── static/
    ├── index.html              # 单页 HTML（三栏布局）
    ├── style.css               # 样式（GitHub Dark）
    └── app.js                  # 前端逻辑（Three.js + WebSocket）
```

---

## 4. 功能实现方式

### 4.1 关节空间控制

**实现方式**：前端滑块 → WebSocket `/ws/control` → 命令队列 → 60FPS 仿真循环

```python
# robot_controller.py - 双控制模式
def _apply_pose(self, angles):        # 精确模式：resetJointState（瞬移定位）
def _apply_motor_control(self, angles): # 物理模式：setJointMotorControl2（电机控制）
```

- **精确模式（默认）**：使用 `resetJointState` 直接设置关节角度，无物理惯性，精确定位
- **物理仿真模式**：使用 `setJointMotorControl2` + `POSITION_CONTROL`，模拟真实电机行为
- 关节角度范围限制（来自 URDF `limit` 标签）
- 命令队列模式保证线程安全

### 4.2 任务空间控制（IK 逆运动学）

**实现方式**：PyBullet 内置 `calculateInverseKinematics`（阻尼最小二乘法）

```python
# robot_controller.py
def solve_ik(self, target_x, target_y, target_z, target_roll, target_pitch, target_yaw):
    joint_poses = p.calculateInverseKinematics(
        bodyUniqueId=self._robot_id,
        endEffectorLinkIndex=ee_link,
        targetPosition=[x, y, z],
        targetOrientation=quaternion,  # 可选
        lowerLimits=ll, upperLimits=ul,
        maxNumIterations=500,
        residualThreshold=1e-5,
    )
```

- 支持位置（XYZ）+ 姿态（Roll/Pitch/Yaw）约束
- 误差验证：应用结果前自动检查 IK 误差，阈值 10mm
- 失败保护：误差过大时不移动机器人

### 4.3 正向运动学（FK）与 3D 可视化

**实现方式**：每帧计算所有 link 的世界坐标位姿，通过 WebSocket 推送到前端

```python
# robot_controller.py - 每帧计算 8 个 link 的位姿
for idx in range(num_joints):
    ls = p.getLinkState(self._robot_id, idx, computeForwardKinematics=True)
    links.append({"px": wpos[0], "py": wpos[1], "pz": wpos[2], "qx": ..., "qy": ..., "qz": ..., "qw": ...})
```

- Three.js 通过 `STLLoader` 加载 8 个 STL 网格
- 每帧根据 PyBullet 返回的 `position + quaternion` 更新 Three.js 网格位置
- 60FPS WebSocket `/ws/stream` 推送关节状态 + link 位姿 + 末端位姿

### 4.4 末端相机（RGB + 深度）

**实现方式**：按钮触发 → 一次性后台 PyBullet 实例 → 渲染一帧 → 保存 + 返回 base64

```python
# robot_controller.py - 按需拍摄
def capture_single_frame(self):
    # 1. 创建独立 PyBullet 实例（p.DIRECT）
    # 2. 加载机器人 + 地面 + 工作空间物体
    # 3. 应用当前关节角度
    # 4. p.getCameraImage() 渲染 RGB + 深度
    # 5. OpenCV 编码 JPEG + 深度热力图
    # 6. 保存到 captures/ + 返回内存数据
    # 7. 断开连接，线程退出
```

- 相机位置：末端执行器 + 15cm 沿局部 X 轴偏移
- RGB：`cv2.imencode('.jpg', rgb_bgr)`
- 深度：`cv2.applyColorMap(depth_gray, cv2.COLORMAP_JET)` → JET 热力图
- 前端通过轮询 `/api/camera/status` + `/api/camera` 获取图像

### 4.5 碰撞检测

**实现方式**：PyBullet `getContactPoints()` 每帧检测三种碰撞类型

```python
# robot_controller.py
def _detect_collisions(self):
    # 1. 机器人 vs 工作空间物体
    contacts = p.getContactPoints(bodyA=robot_id, bodyB=object_id)
    # 2. 机器人 vs 地面
    contacts = p.getContactPoints(bodyA=robot_id, bodyB=ground_id)
    # 3. 机器人自碰撞（link vs link）
    contacts = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)
    # 过滤：排除相邻 link、配置忽略对、力阈值
```

- **URDF 自碰撞标志**：`URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT`
- **碰撞体**：URDF 中 STL 网格（PyBullet 自动转凸包）+ 工作空间物体 `createCollisionShape`
- **性能优化**：使用 `resetBasePositionAndOrientation` 更新物体位置，而非重建
- **可配置忽略**：`COLLISION_IGNORE_LINK_PAIRS`（如 Forearm↔Wrist_Lower）
- 碰撞信息包含：类型、link 名称、碰撞物体名称、力大小、碰撞位置

### 4.6 工作空间物体（可拖拽）

**实现方式**：Three.js 前端创建 → 拖拽更新位置 → 同步到 PyBullet 后端

- 4 个球体 + 4 个方块，定义在 `config.py` 的 `WORKSPACE_OBJECTS`
- Three.js `DragControls` 实现拖拽
- 拖拽结束时自动调用 `POST /api/workspace/objects` 同步位置
- PyBullet 中同时更新视觉体和碰撞体位置

---

## 5. 配置管理 (`config.py`)

所有可调参数集中在 `config.py` 中，修改无需改动核心逻辑：

| 分类 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| 仿真 | `GRAVITY` | -9.81 | 重力加速度 |
| 仿真 | `TIME_STEP` | 1/240 | 仿真时间步长 |
| 仿真 | `SIM_STEPS_PER_FRAME` | 4 | 每帧仿真步数 |
| 仿真 | `TARGET_FPS` | 60 | 控制循环频率 |
| 控制 | `DEFAULT_CONTROL_MODE` | "precise" | 默认控制模式（"precise" 或 "motor"） |
| 电机 | `MOTOR_FORCE` | 5000 | 电机最大力矩 (N) |
| 电机 | `MOTOR_POSITION_GAIN` | 0.3 | 位置增益 kp |
| 电机 | `MOTOR_VELOCITY_GAIN` | 1.0 | 阻尼增益 kd |
| 相机 | `CAM_WIDTH` / `CAM_HEIGHT` | 160 / 120 | 相机分辨率 |
| 相机 | `CAM_FOV` | 60 | 视场角 (度) |
| 相机 | `CAM_OFFSET` | 0.15 | 末端偏移距离 (m) |
| IK | `IK_MAX_ITERATIONS` | 500 | 最大迭代次数 |
| IK | `IK_ERROR_THRESHOLD` | 0.01 | 误差阈值 (m) |
| 碰撞 | `COLLISION_DETECTION_ENABLED` | True | 启用碰撞检测 |
| 碰撞 | `COLLISION_FORCE_THRESHOLD` | 0.0 | 力阈值 (N) |
| 碰撞 | `COLLISION_IGNORE_LINK_PAIRS` | [(4,6),(6,4)] | 忽略的 link 对 |
| 服务器 | `HOST` / `PORT` | 0.0.0.0 / 8000 | 服务器地址 |

---

## 6. API 接口

### 6.1 REST API

| 方法 | 路径 | 说明 | 请求体 |
|------|------|------|--------|
| `GET` | `/api/info` | 获取关节信息（名称、限位） | - |
| `GET` | `/api/state` | 获取最新机器人状态（关节角 + link 位姿 + 末端位姿） | - |
| `POST` | `/api/joint/{index}` | 设置单个关节角度 | `{"angle": 1.57}` |
| `POST` | `/api/ik` | 求解 IK 并移动机器人 | `{"x":0.3, "y":0, "z":0.3, "roll":0, "pitch":0, "yaw":0}` |
| `POST` | `/api/ik_solve` | 仅求解 IK（不移动） | 同上 |
| `GET` | `/api/control_mode` | 获取当前控制模式 | - |
| `POST` | `/api/control_mode` | 切换控制模式 | `{"mode": "precise"}` 或 `{"mode": "motor"}` |
| `GET` | `/api/collision` | 获取碰撞状态 | - |
| `GET` | `/api/collision/debug` | 调试：所有原始自碰撞接触点 | - |
| `POST` | `/api/workspace/objects` | 更新工作空间物体位置 | `{"objects": [{"pos":[x,y,z]}, ...]}` |
| `POST` | `/api/camera/capture` | 触发单帧拍摄 | `{"objects": [...]}` (可选) |
| `GET` | `/api/camera/status` | 查询拍摄状态 + 保存路径 | - |
| `GET` | `/api/camera` | 获取最新拍摄图像 (base64) | - |

### 6.2 WebSocket API

| 路径 | 说明 | 方向 | 数据格式 |
|------|------|------|----------|
| `/ws/control` | 控制指令通道 | 双向 JSON | `{"type":"set_joint", "index":0, "angle":1.57}` |
| `/ws/stream` | 60FPS 状态流 | 服务端→客户端 | `{"joints":[...], "links":[...], "ee_pose":{...}}` |
| `/ws/camera` | 相机图像流（已弃用） | 服务端→客户端 | 二进制帧（前缀 0x00=RGB, 0x01=Depth） |

### 6.3 WebSocket 控制消息类型

| type | 说明 | 参数 |
|------|------|------|
| `set_joint` | 设置单个关节 | `index`, `angle` |
| `set_joints` | 设置所有关节 | `angles[]` |
| `ik` | IK 求解并移动 | `x`, `y`, `z`, `roll`, `pitch`, `yaw` |
| `get_state` | 请求当前状态 | - |

### 6.4 碰撞数据结构

```json
{
  "detected": true,
  "count": 2,
  "collisions": [
    {
      "type": "self",           // 自碰撞
      "link_a": 4, "link_b": 6,
      "link_a_name": "Forearm",
      "link_b_name": "Wrist_Lower",
      "force": 138.744,
      "position": [0.0642, 0.1488, 0.3735]
    },
    {
      "type": "object",         // 物体碰撞
      "robot_link": 3,
      "robot_link_name": "Elbow",
      "object_body": 5,
      "object_name": "sphere_0",
      "force": 25.5,
      "position": [0.3, 0.15, 0.05]
    },
    {
      "type": "ground",         // 地面碰撞
      "robot_link": 0,
      "robot_link_name": "Shoulder_Inner",
      "force": 50.2,
      "position": [0.0, 0.0, 0.05]
    }
  ]
}
```

---

## 7. 后期迭代接口

### 7.1 新增功能模块

| 功能 | 接口建议 | 说明 |
|------|---------|------|
| **轨迹规划** | `POST /api/trajectory` | 输入路径点列表，输出时间-关节角序列 |
| **抓取规划** | `POST /api/grasp` | 输入物体位置，自动规划抓取姿态 |
| **多机器人** | `POST /api/robot/add` | 动态加载多个 URDF 实例 |
| **场景编辑** | `POST /api/scene/objects` | 动态添加/删除工作空间物体 |
| **录制回放** | `POST /api/recording/start` / `GET /api/recording/play` | 录制关节轨迹并回放 |
| **物理参数** | `PUT /api/physics` | 动态调整重力、摩擦系数等 |
| **传感器** | `GET /api/sensor/force` | 力/力矩传感器数据 |
| **仿真控制** | `POST /api/sim/pause` / `POST /api/sim/step` | 仿真暂停/单步执行 |

### 7.2 前端扩展接口

| 功能 | 实现方式 | 说明 |
|------|---------|------|
| **自定义相机视角** | `POST /api/camera/preset` | 保存/加载多个相机预设 |
| **路径可视化** | Three.js `Line` | 在 3D 场景中绘制运动轨迹 |
| **关节限位可视化** | Three.js `Arc` | 显示关节活动范围 |
| **碰撞高亮** | Three.js 材质切换 | 碰撞时 link 变红 |
| **多视图** | 多个 Three.js 渲染器 | 自由视角 + 俯视 + 侧视 |

### 7.3 已知技术债

| 项目 | 说明 | 改进方向 |
|------|------|---------|
| 相机为按需拍摄 | 非实时流 | 可恢复持续相机线程 |
| 碰撞体用 STL 凸包 | 精度有限 | 可改为简化几何体（圆柱/球） |
| 无 ROS 集成 | 独立系统 | 可添加 ROS Bridge |
| 无数据库 | 无历史记录 | 可添加 PostgreSQL 存储轨迹数据 |

---

## 8. 关键设计决策

| 决策 | 原因 |
|------|------|
| PyBullet `p.DIRECT` 模式 | 无 GUI 窗口，适合服务器部署 |
| 双 PyBullet 实例（主仿真 + 按需相机） | 避免相机渲染阻塞 60FPS 仿真循环 |
| 命令队列模式 | 前端指令通过队列传递，避免线程竞争 |
| `setJointMotorControl2` 可选 | 满足技术栈要求，默认用精确模式保证体验 |
| OpenCV 替代 PIL | 性能更好 + 满足技术栈要求 |
| 配置外部化 (`config.py`) | 关注点分离，调参不改核心逻辑 |