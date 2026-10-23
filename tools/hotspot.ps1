# Windows Mobile Hotspot on/off/status (no admin needed). Fallback network for the Quest when the
# room router keeps dropping: the headset joins the PC's own hotspot and the bridge is reachable at
# the hotspot adapter's address (usually 192.168.137.1).
#   tools\hotspot.ps1 status | on | off      (off: also turns the Wi-Fi radio back on; set TELEOP_WIFI=<ssid> to auto-reconnect)
param([string]$Action = "status")
Add-Type -AssemblyName System.Runtime.WindowsRuntime
$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
function Await($WinRtTask, $ResultType) {
    $asTask = $asTaskGeneric.MakeGenericMethod($ResultType)
    $netTask = $asTask.Invoke($null, @($WinRtTask))
    $netTask.Wait(-1) | Out-Null
    $netTask.Result
}
$null = [Windows.Networking.Connectivity.NetworkInformation, Windows.Networking.Connectivity, ContentType = WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager, Windows.Networking.NetworkOperators, ContentType = WindowsRuntime]
$profile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
if (-not $profile) { Write-Output "no internet connection profile (hotspot needs one to share)"; exit 1 }
$mgr = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($profile)
$cfg = $mgr.GetCurrentAccessPointConfiguration()
switch ($Action) {
    "on" {
        $r = Await ($mgr.StartTetheringAsync()) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
        Write-Output ("start: " + $r.Status + " " + $r.AdditionalErrorMessage)
    }
    "off" {
        $r = Await ($mgr.StopTetheringAsync()) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
        Write-Output ("stop: " + $r.Status + " " + $r.AdditionalErrorMessage)
        # stopping the hotspot leaves the Wi-Fi radio powered off on this USB adapter -> turn it back on
        $null = [Windows.Devices.Radios.Radio, Windows.System.Devices, ContentType = WindowsRuntime]
        $null = Await ([Windows.Devices.Radios.Radio]::RequestAccessAsync()) ([Windows.Devices.Radios.RadioAccessStatus])
        $radios = Await ([Windows.Devices.Radios.Radio]::GetRadiosAsync()) ([System.Collections.Generic.IReadOnlyList[Windows.Devices.Radios.Radio]])
        foreach ($rd in $radios) { if ($rd.Kind -eq 'WiFi' -and $rd.State -ne 'On') { $null = Await ($rd.SetStateAsync('On')) ([Windows.Devices.Radios.RadioAccessStatus]); Write-Output "wifi radio: on" } }
        Start-Sleep -Seconds 3
        if ($env:TELEOP_WIFI) { netsh wlan connect name="$env:TELEOP_WIFI" | Out-Null; Write-Output ("reconnect: " + $env:TELEOP_WIFI) }
    }
}
Write-Output ("state: " + $mgr.TetheringOperationalState + "  ssid: " + $cfg.Ssid + "  pass: " + $cfg.Passphrase + "  clients: " + $mgr.ClientCount)
Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.IPAddress -like "192.168.137.*" } | ForEach-Object { Write-Output ("hotspot ip: " + $_.IPAddress) }
