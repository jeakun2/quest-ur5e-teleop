"""Keep a Quest awake with the headset taken off (controller-only teleop).

The Quest sleeps a few seconds after the proximity sensor stops seeing a face;
tracking (and the browser / APK) stops with it.  With developer mode + adb this
tells the power manager that the sensor is covered:

    python tools/quest_keep_awake.py on      # pretend the headset is worn
    python tools/quest_keep_awake.py off     # back to normal
    python tools/quest_keep_awake.py status  # adb devices

Without developer mode: put a piece of opaque tape over the proximity sensor
(between the lenses, Quest 3: the small window above the nose bridge).  Either
way the headset must be placed so its cameras can see the controllers.
"""
from __future__ import annotations

import os
import subprocess
import sys

ADB = os.environ.get("ADB", r"C:\Users\USER\AppData\Local\Android\Sdk\platform-tools\adb.exe")
if not os.path.exists(ADB):
    ADB = "adb"

ON = ["shell", "am", "broadcast", "-a", "com.oculus.vrpowermanager.prox_close"]
OFF = ["shell", "am", "broadcast", "-a", "com.oculus.vrpowermanager.automation_disable"]


def adb(*args: str) -> int:
    print("$ adb", " ".join(args))
    return subprocess.call([ADB, *args])


def main(argv: list[str]) -> int:
    what = (argv[0] if argv else "status").lower()
    if what == "on":
        return adb(*ON)
    if what == "off":
        return adb(*OFF)
    return adb("devices", "-l")


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
