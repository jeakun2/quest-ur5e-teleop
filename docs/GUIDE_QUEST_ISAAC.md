# 가이드 A — Quest 3 + Isaac Sim 4.2 (시뮬레이터로 연습)

로봇 없이 PC 한 대와 Quest 3로 전체 조작을 익히는 순서. 인터넷은 필요 없다(PC·Quest가 같은 LAN이면 됨).
실제 로봇은 `GUIDE_QUEST_REAL_ROBOT.md`.

## 1. 준비 (처음 한 번)

- Quest 3 + 오른손 컨트롤러 충전. PC와 Quest를 **같은 Wi-Fi/핫스팟**에(폰 핫스팟은 데이터 꺼도 됨).
- Isaac Sim은 첫 실행 때 UR5e USD를 한 번 내려받으므로, 인터넷이 되는 곳에서 미리 한 번 띄워 둔다(아래 2번).
- 방화벽에서 8765 포트 허용(관리자 PowerShell, 한 번만):
  ```powershell
  New-NetFirewallRule -DisplayName "quest-teleop 8765" -Direction Inbound -Protocol TCP -LocalPort 8765 -Action Allow
  ```

## 2. Isaac Sim 띄우기

창이 뜨는 모드로 띄운다(`tools/isaac_sim.py`가 Isaac Sim용 파이썬 환경을 찾아 `isaac/ur5e_sim.py`를 실행한다,
환경 설정은 `ISAAC_SIM.md`):

```powershell
cd quest-ur5e-teleop
.venv\Scripts\python tools\isaac_sim.py --log isaac\logs\sim.log
```

- 1~3분 뒤 Isaac Sim 창에 파란 격자 위의 UR5e가 보이고, `isaac\logs\sim_*.log`에 `READY`, `listening on 127.0.0.1:5560`이 찍힌다.
- 창 없이(헤드리스) 돌리려면 끝에 `--headless`. 창 모드는 마우스 오른쪽 드래그로 시점을 돌릴 수 있다.
- 상태 확인: `.venv\Scripts\python tools\isaac_ctl.py status` → `tcp`, `hz`가 나오면 정상.

## 3. 브리지 띄우기

```powershell
.venv\Scripts\python bridge\teleop_bridge.py --robot.type isaac
```
`[isaac] connected …`, `webxr https://<PC-IP>:8765`, `running` 이 보이면 준비. PC IP는 `ipconfig | findstr IPv4`.

## 4. Quest에서 접속

1. 헤드셋 착용 → **브라우저** → `https://<PC-IP>:8765` (https 필수).
2. 인증서 경고 → **고급 → 계속**. (자체 서명, 외부 서버 없음)
3. **Start AR**(패스스루) 또는 **Start VR** → 권한 허용.
4. HUD에 `mode=hmd`, `holding (squeeze grip to engage)`가 뜨고 PC 터미널에 `clients=1`이 보이면 연결된 것.

PC 모니터에서 Isaac Sim 창을 보면서 조작하면 되고, 헤드셋을 쓴 채로는 패스스루로 모니터를 보면 된다.

## 5. 첫 보정과 연습

1. 썸스틱 **클릭** → 메뉴 → **튜토리얼**(A): 그립 이동·래칫·트리거·A·B·메뉴를 한 번씩 써 보면 자동으로 넘어간다.
2. 메뉴 → **보정**: ① 그립 쥔 채 Isaac 창 속 로봇이 있다고 생각하는 쪽으로 20 cm → ② 편한 위치에서 트리거 → ③ 그립 쥐고 10 cm 확인 → ④ A 저장.
   시뮬에서는 "로봇이 있는 방향"을 모니터 쪽으로 잡으면 된다. 저장값은 `bridge\config.yaml`에 들어가고 실기에서도 그대로 쓰인다.
3. 그립을 쥐고 움직여 본다. Isaac 창의 UR5e가 손을 따라오고, 놓으면 멈춘다.
4. 메뉴에서 **Z축 고정**(평면 작업), **회전 고정**, **이동 배율**, **녹화 시작/정지 → 마지막 녹화 재생**을 차례로 써 본다.

## 6. 끝내기

- 브리지 터미널 Ctrl+C. Isaac Sim은 창을 닫거나 `.venv\Scripts\python tools\isaac_ctl.py quit`.
- 녹화 파일은 `runs\rec_*.jsonl`. 실기에서도 같은 파일을 재생할 수 있다(시작점까지 속도 제한 안에서 접근한 뒤 재생).

## 7. 자주 걸리는 것

| 증상 | 확인 |
|---|---|
| Isaac 창이 안 뜸 / `READY`가 안 찍힘 | `isaac\logs\sim*.log` 끝부분. `No module named numpy`면 런처가 numpy·scipy가 있는 Isaac 파이썬 환경을 못 찾은 것(`--conda-env`/`--isaac`, `ISAAC_SIM.md`) |
| 브리지가 `cannot connect to the sim` | Isaac이 아직 `READY` 전(최대 2분 재시도함). 포트 5560 중복이면 `--port`로 바꾸고 `--robot.port` 동일하게 |
| 페이지가 안 열림 | 같은 Wi-Fi, `https://`, 방화벽, 브리지 실행 여부. PC 브라우저의 **2D test**로 브리지만 먼저 확인 |
| 로봇이 엉뚱한 방향 | 메뉴 → 보정 다시. 시뮬은 UR Base 프레임이라 실기와 같은 규약 |
| 끊김 | PC 터미널 Hz(브리지 120, 시뮬 창 모드 80~105). 다른 GPU 작업이 많으면 `--headless`로 |
