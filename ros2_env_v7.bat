@echo off
setlocal EnableDelayedExpansion
chcp 65001 >nul
title ROS 2 Humble 镜像持久化开发环境 V7
:: ============================================================================
::  ros2_env_v7.bat (改写版 V7)
::  自动化启动 ROS 2 Humble 镜像持久化 Docker 容器
:: ============================================================================

:: ---------------------------------------------------------------------------
:: (1) 用户配置区域
:: ---------------------------------------------------------------------------
set "ContainerName=ros2_humble_persistent"
set "ImageName=minzi0921/ros2_humble_persistent_image:latest"
set "HostWS=C:\ros2_workspaces"
set "VcXsrvPath=C:\Users\99741\Desktop\ros2.xlaunch"
set "DockerPath=C:\Program Files\Docker\Docker\Docker Desktop.exe"
set "VSCodePath=C:\Users\99741\AppData\Local\Programs\Microsoft VS Code\Code.exe"
set "DisplayIP=172.24.96.1:0.0"          &rem 按需修改

:: 主机侧 ROS2 个性配置文件（自动挂载并注入）
set "RosBashrcHost=%HostWS%\.ros2_bashrc"
set "RosBashrcContainer=/ros2_workspaces/.ros2_bashrc"

:: ---------------------------------------------------------------------------
:: (2) 启动依赖程序 (无变动)
:: ---------------------------------------------------------------------------
echo.
echo [Step 1/10] Starting background applications...
if not exist "%DockerPath%" (
    echo    - ERROR: Docker Desktop not found at "%DockerPath%". Check path.
    pause & exit /b
)
echo    - Starting Docker Desktop...
start "" "%DockerPath%"

if exist "%VcXsrvPath%" (
    echo    - Starting VcXsrv ...
    start "" "%VcXsrvPath%"
) else (
    echo    - WARNING: VcXsrv config not found. GUI may fail.
)

:: ---------------------------------------------------------------------------
:: (3) 等待 Docker 就绪 (无变动)
:: ---------------------------------------------------------------------------
echo.
echo [Step 2/10] Waiting for Docker daemon ...
:waitDocker
docker info >nul 2>&1
if !ERRORLEVEL! equ 0 goto dockerOK
echo|set /p=.
ping -n 3 127.0.0.1 >nul
goto waitDocker
:dockerOK

:: ---------------------------------------------------------------------------
:: (4) 检查/创建主机工作区 & 个性配置文件 (无变动)
:: ---------------------------------------------------------------------------
if not exist "%HostWS%" mkdir "%HostWS%"
if not exist "%RosBashrcHost%" (
    echo # ROS 2 Humble user customisations > "%RosBashrcHost%"
    echo source /opt/ros/humble/setup.bash >> "%RosBashrcHost%"
    echo # test if it works >> "%RosBashrcHost%"
    echo export ROS_DOMAIN_ID=42 >> "%RosBashrcHost%"
)

:: ---------------------------------------------------------------------------
:: (5) 容器与镜像生命周期管理 (核心改动 - 启动部分)
:: ---------------------------------------------------------------------------
echo.
echo [Step 3/10] Managing container and image lifecycle...

:: 如果上次异常退出导致旧容器残留，先强制删除
docker inspect %ContainerName% >nul 2>&1
if !ERRORLEVEL! equ 0 (
    echo   - Found and removing leftover container from previous session...
    docker rm -f %ContainerName% >nul
)

:: <<<<<<<<<<< 新增拉取功能开始 >>>>>>>>>>>
echo.
echo [Step 4/10] Pulling latest image from Docker Hub...
docker pull %ImageName%
echo.
:: <<<<<<<<<<< 新增拉取功能结束 >>>>>>>>>>>

echo [Step5/10] Create and starte a new container '%ContainerName%'...

:: 1. 先设置一个默认值，即使用基础镜像
echo   - Setting base image 'osrf/ros:humble-desktop-full' as default...
set "ImageToUse=osrf/ros:humble-desktop-full"

:: 2. 检查持久化镜像是否存在
docker image inspect %ImageName% >nul 2>&1
if !ERRORLEVEL! equ 0 (
    :: 3. 如果持久化镜像存在，就用它的名字覆盖默认值
    echo   - Found persistent image '%ImageName%'. Overriding default.
    set "ImageToUse=%ImageName%"
) else (
    echo   - Persistent image not found. Will proceed with the base image.
    echo   - (Image '%ImageName%' will be created on exit)
)

:: 添加一步诊断输出，明确告知用户最终将使用哪个镜像
echo.
echo   - Final image to be used: !ImageToUse!
echo.


:: 以后台模式(-d)创建并启动容器
echo   - Creating and starting new container '%ContainerName%'...
docker run -itd ^
  --name %ContainerName% ^
  -p 50001:50001 ^
  -p 50002:50002 ^
  -p 50003:50003 ^
  -p 50004:50004 ^
  --privileged ^
  -e DISPLAY=%DisplayIP% ^
  -v "%HostWS%:/ros2_workspaces" ^
  !ImageToUse! >nul

:: 确认容器已成功启动
docker inspect %ContainerName% >nul 2>&1
if !ERRORLEVEL! neq 0 (
    echo   - ERROR: Failed to create or start the container. Aborting.
    pause & exit /b
) else (
    echo   - Container is running in the background.
)

:: ---------------------------------------------------------------------------
:: (6) 注入个性配置 (已删除)
:: ---------------------------------------------------------------------------

:: ---------------------------------------------------------------------------
:: (7) 启动 VSCode (无变动)
:: ---------------------------------------------------------------------------
echo.
echo [Step 6/10] Launching VSCode ...
start "" "%VSCodePath%" >NUL 2>&1

:: ---------------------------------------------------------------------------
:: (8) 进入容器交互 (无变动)
:: ---------------------------------------------------------------------------
echo.
echo [Step 7/10] Attaching to container (%ContainerName%) ...
echo   - Type 'exit' to leave and SAVE the container state to an image.
echo.
docker exec -it %ContainerName% bash

:: ---------------------------------------------------------------------------
:: (9) 退出后：清理、提交、停止并移除容器 (已优化)
:: ---------------------------------------------------------------------------
echo.
echo [Step 8/10] Container session ended. Cleaning up before saving state...

:: 清理APT缓存 (通常会省出几百MB)
echo   - Cleaning apt cache inside container...
docker exec %ContainerName% apt-get clean >nul

:: 清理ROS 2工作区中的编译缓存和日志 (能省出大量空间和文件数)
echo   - Cleaning colcon build artifacts and logs...
docker exec %ContainerName% rm -rf /ros2_workspaces/build /ros2_workspaces/log

echo.
echo [Step 9/10] Saving state to image...
echo   - Committing container '%ContainerName%' to image '%ImageName%'...
docker commit %ContainerName% %ImageName% >nul

echo.

echo   - Pushing image to Docker Hub... This may take a while.
docker push %ImageName%

echo [Step 10/10] Leaving...

echo   - Stopping container...
docker stop %ContainerName% >nul

echo   - Removing temporary container...
docker rm %ContainerName% >nul
echo   - Lifecycle complete. All changes saved in image '%ImageName%'.

:: ---------------------------------------------------------------------------
:: (10) 可选：关闭后台程序 (无变动)
:: ---------------------------------------------------------------------------
    echo    - Shutting down WSL and Docker ...
    wsl --shutdown
    taskkill /F /IM "Docker Desktop.exe" 2>nul
    taskkill /F /IM "com.docker.backend.exe" 2>nul
    taskkill /F /IM vcxsrv.exe 2>nul

echo.
echo ======================  All done. ======================