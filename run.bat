@echo off
rem Quest 3 -> UR5e teleop bridge.  Examples:
rem   run.bat                                        (WebXR + dummy robot)
rem   run.bat --robot.type ur_rtde --robot.ip 192.168.1.10
rem   run.bat --source.type oculus_reader --robot.type ur_rtde --robot.ip 192.168.1.10
cd /d "%~dp0"
".venv\Scripts\python.exe" bridge\teleop_bridge.py %*
