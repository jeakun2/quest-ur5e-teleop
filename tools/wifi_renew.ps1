# Re-acquire a DHCP address on the Wi-Fi adapter (admin). Used when Windows falls back to 169.254.x.
$log = Join-Path $PSScriptRoot "wifi_renew.log"
try {
    ipconfig /release "Wi-Fi" | Out-Null
    Start-Sleep -Seconds 2
    ipconfig /renew "Wi-Fi" | Out-Null
    Start-Sleep -Seconds 3
    $ip = (Get-NetIPAddress -InterfaceAlias "Wi-Fi" -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -First 1).IPAddress
    if (-not $ip -or $ip -like "169.254.*") {
        Restart-NetAdapter -Name "Wi-Fi" -Confirm:$false
        Start-Sleep -Seconds 8
        $ip = (Get-NetIPAddress -InterfaceAlias "Wi-Fi" -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -First 1).IPAddress
    }
    "$(Get-Date -Format s) wifi ip = $ip" | Out-File $log -Append -Encoding utf8
    exit 0
} catch {
    "$(Get-Date -Format s) FAILED: $($_.Exception.Message)" | Out-File $log -Append -Encoding utf8
    exit 1
}
