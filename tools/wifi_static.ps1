# Give the Wi-Fi adapter a fixed LAN address when the router's DHCP does not answer (169.254.x fallback).
# Usage (admin): wifi_static.ps1 -Ip 192.168.0.52 -Gateway 192.168.0.1 [-Dhcp]  (-Dhcp restores DHCP)
param([string]$Ip = "192.168.0.52", [string]$Gateway = "192.168.0.1", [int]$Prefix = 24, [switch]$Dhcp)
$log = Join-Path $PSScriptRoot "wifi_static.log"
try {
    $if = Get-NetAdapter -Name "Wi-Fi"
    if ($Dhcp) {
        Set-NetIPInterface -InterfaceIndex $if.ifIndex -Dhcp Enabled
        Remove-NetRoute -InterfaceIndex $if.ifIndex -DestinationPrefix "0.0.0.0/0" -Confirm:$false -ErrorAction SilentlyContinue
        ipconfig /renew "Wi-Fi" | Out-Null
        "$(Get-Date -Format s) back to DHCP" | Out-File $log -Append -Encoding utf8
        exit 0
    }
    Set-NetIPInterface -InterfaceIndex $if.ifIndex -Dhcp Disabled
    Get-NetIPAddress -InterfaceIndex $if.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue | Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
    Remove-NetRoute -InterfaceIndex $if.ifIndex -DestinationPrefix "0.0.0.0/0" -Confirm:$false -ErrorAction SilentlyContinue
    New-NetIPAddress -InterfaceIndex $if.ifIndex -IPAddress $Ip -PrefixLength $Prefix -DefaultGateway $Gateway -ErrorAction Stop | Out-Null
    Set-DnsClientServerAddress -InterfaceIndex $if.ifIndex -ServerAddresses ($Gateway, "8.8.8.8")
    Start-Sleep -Seconds 2
    $ok = Test-Connection -ComputerName $Gateway -Count 2 -Quiet
    "$(Get-Date -Format s) static $Ip/$Prefix gw $Gateway ping-gw=$ok" | Out-File $log -Append -Encoding utf8
    exit 0
} catch {
    "$(Get-Date -Format s) FAILED: $($_.Exception.Message)" | Out-File $log -Append -Encoding utf8
    exit 1
}
