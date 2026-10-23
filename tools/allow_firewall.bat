@echo off
rem Adds the Windows Firewall rule for the teleop bridge (TCP 8765). Asks for administrator rights (UAC).
powershell -NoProfile -Command "Start-Process powershell -Verb RunAs -Wait -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','%~dp0allow_firewall.ps1'"
type "%~dp0allow_firewall.log" 2>nul
pause
