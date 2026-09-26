"""
scripts/verify_browser.py
Capture Analytics Drawer with live Chart.js charts.
"""

import subprocess
import time
import json
import base64
import httpx
import websockets
import asyncio
import os

CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ARTIFACT_DIR = r"C:\Users\kedia\.gemini\antigravity\brain\520a6b51-7d8b-4a86-80c6-ab15e9a57763"

async def main():
    print("[*] Launching Headless Chrome with Remote Debugging...")
    proc = subprocess.Popen([
        CHROME_PATH,
        "--headless=new",
        "--remote-debugging-port=9222",
        "--no-sandbox",
        "--disable-gpu",
        "--window-size=1920,1080",
        "http://127.0.0.1:8000/"
    ])
    
    try:
        page_ws_url = None
        for attempt in range(20):
            time.sleep(0.5)
            try:
                r = httpx.get("http://127.0.0.1:9222/json", timeout=2.0)
                if r.status_code == 200:
                    for p in r.json():
                        if p.get("type") == "page" and "8000" in p.get("url", ""):
                            page_ws_url = p.get("webSocketDebuggerUrl")
                            break
                    if page_ws_url:
                        break
            except Exception:
                pass

        if not page_ws_url:
            print("[-] Could not find page WebSocket URL.")
            return

        async with websockets.connect(page_ws_url) as ws:
            msg_id = 0
            async def send_cmd(method, params=None):
                nonlocal msg_id
                msg_id += 1
                payload = {"id": msg_id, "method": method}
                if params:
                    payload["params"] = params
                await ws.send(json.dumps(payload))
                while True:
                    m = json.loads(await ws.recv())
                    if m.get("id") == msg_id:
                        return m

            await send_cmd("Runtime.enable")
            await send_cmd("Page.enable")
            
            # Wait 4 seconds for simulation telemetry to stream and populate chart history
            print("[*] Waiting 4 seconds for telemetry and chart data points...")
            await asyncio.sleep(4.0)

            # Open Analytics Drawer
            print("[*] Opening Analytics Drawer (#btn-toggle-analytics)...")
            await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-toggle-analytics').click()"
            })
            await asyncio.sleep(1.5)

            # Capture Analytics Drawer Screenshot
            ss1 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_1 = ss1.get("result", {}).get("data")
            if b64_1:
                path1 = os.path.join(ARTIFACT_DIR, "cockpit_analytics_drawer_live.png")
                with open(path1, "wb") as f:
                    f.write(base64.b64decode(b64_1))
                print(f"[+] Saved Live Analytics Drawer view: {path1}")

    finally:
        proc.terminate()
        proc.wait()
        print("[*] Headless Chrome terminated.")

if __name__ == "__main__":
    asyncio.run(main())
