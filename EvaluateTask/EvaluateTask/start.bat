@echo off
chcp 65001 >nul 2>&1
echo ========================================
echo   机械臂仿真控制系统 - 一键启动
echo ========================================
echo.

:: 检查Python是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到Python，请先安装Python 3.10+
    pause
    exit /b 1
)

:: 安装依赖
echo [1/2] 检查并安装依赖...
pip install -r requirements.txt -q
if errorlevel 1 (
    echo [警告] 部分依赖安装失败，尝试继续启动...
)

:: 启动服务
echo [2/2] 启动服务器...
echo.
echo ========================================
echo   服务已启动！请在浏览器中打开:
echo   http://localhost:8000
echo   按 Ctrl+C 停止服务
echo ========================================
echo.

:: 自动打开浏览器
start "" http://localhost:8000

:: 启动FastAPI服务
python main.py

pause