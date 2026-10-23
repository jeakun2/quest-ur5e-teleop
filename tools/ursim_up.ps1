# Start URSim (UR5 e-series) in Docker, power the robot on and switch PolyScope to Remote Control.
# Requires Rancher Desktop / Docker Desktop running.  noVNC: http://localhost:6080
$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $root ".venv\Scripts\python.exe"

docker rm -f ursim 2>$null | Out-Null
docker run -d --name ursim -e ROBOT_MODEL=UR5 -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 universalrobots/ursim_e-series
Write-Host "waiting for URSim to boot (dashboard :29999) ..."
$deadline = (Get-Date).AddMinutes(3)
while ((Get-Date) -lt $deadline) {
    try { $c = New-Object Net.Sockets.TcpClient("127.0.0.1", 29999); $c.Close(); break } catch { Start-Sleep 3 }
}
Start-Sleep 5
& $py -c "import time
from dashboard_client import DashboardClient
d=DashboardClient('127.0.0.1'); d.connect(); d.powerOn(); d.brakeRelease()
for _ in range(40):
    time.sleep(1)
    if 'RUNNING' in d.robotmode(): break
print(d.robotmode())"
& $py (Join-Path $PSScriptRoot "ursim_enable_remote.py")
Write-Host "URSim ready. Run:  .venv\Scripts\python bridge\teleop_bridge.py --robot.type ur_rtde --robot.ip 127.0.0.1"
