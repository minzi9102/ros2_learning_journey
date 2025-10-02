@echo off
setlocal EnableDelayedExpansion
chcp 65001 >nul
title ROS 2 Humble ����־û��������� V7
:: ============================================================================
::  ros2_env_v7.bat (��д�� V7)
::  �Զ������� ROS 2 Humble ����־û� Docker ����
:: ============================================================================

:: ---------------------------------------------------------------------------
:: (1) �û���������
:: ---------------------------------------------------------------------------
set "ContainerName=ros2_humble_persistent"
set "ImageName=minzi0921/ros2_humble_persistent_image:latest"
set "HostWS=C:\ros2_workspaces"
set "VcXsrvPath=C:\Users\99741\Desktop\ros2.xlaunch"
set "DockerPath=C:\Program Files\Docker\Docker\Docker Desktop.exe"
set "VSCodePath=C:\Users\99741\AppData\Local\Programs\Microsoft VS Code\Code.exe"
set "DisplayIP=172.24.96.1:0.0"          &rem �����޸�

:: ������ ROS2 ���������ļ����Զ����ز�ע�룩
set "RosBashrcHost=%HostWS%\.ros2_bashrc"
set "RosBashrcContainer=/ros2_workspaces/.ros2_bashrc"

:: ---------------------------------------------------------------------------
:: (2) ������������ (�ޱ䶯)
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
:: (3) �ȴ� Docker ���� (�ޱ䶯)
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
:: (4) ���/�������������� & ���������ļ� (�ޱ䶯)
:: ---------------------------------------------------------------------------
if not exist "%HostWS%" mkdir "%HostWS%"
if not exist "%RosBashrcHost%" (
    echo # ROS 2 Humble user customisations > "%RosBashrcHost%"
    echo source /opt/ros/humble/setup.bash >> "%RosBashrcHost%"
    echo # test if it works >> "%RosBashrcHost%"
    echo export ROS_DOMAIN_ID=42 >> "%RosBashrcHost%"
)

:: ---------------------------------------------------------------------------
:: (5) �����뾵���������ڹ��� (���ĸĶ� - ��������)
:: ---------------------------------------------------------------------------
echo.
echo [Step 3/10] Managing container and image lifecycle...

:: ����ϴ��쳣�˳����¾�������������ǿ��ɾ��
docker inspect %ContainerName% >nul 2>&1
if !ERRORLEVEL! equ 0 (
    echo   - Found and removing leftover container from previous session...
    docker rm -f %ContainerName% >nul
)

:: <<<<<<<<<<< ������ȡ���ܿ�ʼ >>>>>>>>>>>
echo.
echo [Step 4/10] Pulling latest image from Docker Hub...
docker pull %ImageName%
echo.
:: <<<<<<<<<<< ������ȡ���ܽ��� >>>>>>>>>>>

echo [Step5/10] Create and starte a new container '%ContainerName%'...

:: 1. ������һ��Ĭ��ֵ����ʹ�û�������
echo   - Setting base image 'osrf/ros:humble-desktop-full' as default...
set "ImageToUse=osrf/ros:humble-desktop-full"

:: 2. ���־û������Ƿ����
docker image inspect %ImageName% >nul 2>&1
if !ERRORLEVEL! equ 0 (
    :: 3. ����־û�������ڣ������������ָ���Ĭ��ֵ
    echo   - Found persistent image '%ImageName%'. Overriding default.
    set "ImageToUse=%ImageName%"
) else (
    echo   - Persistent image not found. Will proceed with the base image.
    echo   - (Image '%ImageName%' will be created on exit)
)

:: ���һ������������ȷ��֪�û����ս�ʹ���ĸ�����
echo.
echo   - Final image to be used: !ImageToUse!
echo.


:: �Ժ�̨ģʽ(-d)��������������
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

:: ȷ�������ѳɹ�����
docker inspect %ContainerName% >nul 2>&1
if !ERRORLEVEL! neq 0 (
    echo   - ERROR: Failed to create or start the container. Aborting.
    pause & exit /b
) else (
    echo   - Container is running in the background.
)

:: ---------------------------------------------------------------------------
:: (6) ע��������� (��ɾ��)
:: ---------------------------------------------------------------------------

:: ---------------------------------------------------------------------------
:: (7) ���� VSCode (�ޱ䶯)
:: ---------------------------------------------------------------------------
echo.
echo [Step 6/10] Launching VSCode ...
start "" "%VSCodePath%" >NUL 2>&1

:: ---------------------------------------------------------------------------
:: (8) ������������ (�ޱ䶯)
:: ---------------------------------------------------------------------------
echo.
echo [Step 7/10] Attaching to container (%ContainerName%) ...
echo   - Type 'exit' to leave and SAVE the container state to an image.
echo.
docker exec -it %ContainerName% bash

:: ---------------------------------------------------------------------------
:: (9) �˳��������ύ��ֹͣ���Ƴ����� (���Ż�)
:: ---------------------------------------------------------------------------
echo.
echo [Step 8/10] Container session ended. Cleaning up before saving state...

:: ����APT���� (ͨ����ʡ������MB)
echo   - Cleaning apt cache inside container...
docker exec %ContainerName% apt-get clean >nul

:: ����ROS 2�������еı��뻺�����־ (��ʡ�������ռ���ļ���)
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
:: (10) ��ѡ���رպ�̨���� (�ޱ䶯)
:: ---------------------------------------------------------------------------
    echo    - Shutting down WSL and Docker ...
    wsl --shutdown
    taskkill /F /IM "Docker Desktop.exe" 2>nul
    taskkill /F /IM "com.docker.backend.exe" 2>nul
    taskkill /F /IM vcxsrv.exe 2>nul

echo.
echo ======================  All done. ======================