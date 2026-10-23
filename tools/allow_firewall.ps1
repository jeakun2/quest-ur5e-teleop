# Allow the Quest (and the PC monitor page) to reach the bridge: inbound TCP 8765 on every network profile.
# Run as administrator (right-click -> Run with PowerShell as admin), or:  tools\allow_firewall.bat
$log = Join-Path $PSScriptRoot "allow_firewall.log"
try {
    if (-not (Get-NetFirewallRule -DisplayName "quest-teleop 8765" -ErrorAction SilentlyContinue)) {
        New-NetFirewallRule -DisplayName "quest-teleop 8765" -Direction Inbound -Protocol TCP -LocalPort 8765 -Action Allow -Profile Any -ErrorAction Stop | Out-Null
        "$(Get-Date -Format s) rule added" | Out-File $log -Append -Encoding utf8
    } else {
        "$(Get-Date -Format s) rule already present" | Out-File $log -Append -Encoding utf8
    }
    exit 0
} catch {
    "$(Get-Date -Format s) FAILED: $($_.Exception.Message)" | Out-File $log -Append -Encoding utf8
    exit 1
}
