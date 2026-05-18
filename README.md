一个基于Web的机械臂仿真控制系统，实现以下核心功能：
通过PyBullet加载并仿真机械臂模型
支持关节空间和任务空间的运动控制
实时获取并显示机械臂末端相机的RGB/深度图像
通过Web界面进行交互式控制


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
