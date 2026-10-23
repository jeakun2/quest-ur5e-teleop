"""Headless-browser check of the WebXR page against a running bridge (no headset needed).

    python tools/web_check.py https://192.168.0.52:8765 [out_dir]

Opens the page in Chromium (Playwright), turns on "2D test", drags the mouse with the button held
(= grip + move) so the bridge engages, then saves a screenshot of the three.js view (ghost robot +
paths) and prints the page log (incl. the "ghost FK check" line) and any JS errors.
"""
import glob
import os
import sys
import time

from playwright.sync_api import sync_playwright

URL = sys.argv[1] if len(sys.argv) > 1 else "https://127.0.0.1:8765"
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs", "web_check")
os.makedirs(OUT, exist_ok=True)


def chromium_exe():
    base = os.environ.get("PLAYWRIGHT_BROWSERS_PATH") or os.path.join(os.environ.get("LOCALAPPDATA", ""), "ms-playwright")
    cands = sorted(glob.glob(os.path.join(base, "chromium-*", "chrome-win*", "chrome.exe")), reverse=True)
    return cands[0] if cands else None


def main() -> int:
    errors = []
    with sync_playwright() as pw:
        kw = {"headless": True, "args": ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
                                         "--ignore-gpu-blocklist"]}
        exe = chromium_exe()
        if exe:
            kw["executable_path"] = exe
        browser = pw.chromium.launch(**kw)
        ctx = browser.new_context(ignore_https_errors=True, viewport={"width": 1100, "height": 900})
        page = ctx.new_page()
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: errors.append("console.error: " + m.text) if m.type == "error" else None)
        page.goto(URL, wait_until="load", timeout=30000)
        page.wait_for_function("document.getElementById('ws-state').textContent.includes('connected')", timeout=15000)
        page.click("#btn-test")
        time.sleep(1.5)
        box = page.locator("main").bounding_box()
        # grip + move: button down, sweep right then down (~0.3 m in the 2D mapping), release
        x0, y0 = 300, 300
        page.mouse.move(x0, y0); time.sleep(0.3)
        page.mouse.down()
        for i in range(1, 41):
            page.mouse.move(x0 + i * 6, y0); time.sleep(0.04)
        for i in range(1, 31):
            page.mouse.move(x0 + 240, y0 + i * 6); time.sleep(0.04)
        time.sleep(0.5)
        page.mouse.up()
        time.sleep(1.5)
        page.locator("#view").screenshot(path=os.path.join(OUT, "view.png"))
        page.screenshot(path=os.path.join(OUT, "page.png"), full_page=True)
        log = page.locator("#log").inner_text()
        state = page.locator("#hud-state").inner_text()
        chips = page.locator("#hud-chips").inner_text()
        info = page.evaluate("""() => {
            const c = document.querySelector('#view canvas');
            return {canvas: !!c, w: c && c.width, h: c && c.height, three: !!window.THREE_LOADED};
        }""")
        browser.close()
    print("state:", state)
    print("chips:", chips)
    print("canvas:", info)
    print("--- page log ---")
    print(log[:1500])
    print("--- js errors ---")
    print("\n".join(errors) if errors else "(none)")
    print("screenshots in", OUT)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
