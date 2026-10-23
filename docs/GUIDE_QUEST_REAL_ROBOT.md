# 가이드 B — Quest 3 + 실제 UR5e

실기에서 처음 쓰는 날의 순서. 시뮬레이터(`GUIDE_QUEST_ISAAC.md`)로 조작을 먼저 익힌 뒤에 오는 것을 전제로 한다.
인터넷은 필요 없다. PC·Quest·로봇이 **같은 LAN**에 있으면 된다.

## 0. 안전 수칙 (매번)

- 조작자는 **로봇이 직접 보이는 자리**에 선다. 패스스루(Start AR)로 로봇을 보면서 조작한다.
- 티치펜던트의 **비상정지**를 손 닿는 곳에, 브리지 터미널(Ctrl+C = 즉시 servoStop)을 PC 앞에.
- 처음엔 느리게: `--robot.speed_slider 0.3 --mapping.pos_scale 0.5`. 작업공간 박스는 `bridge\config.yaml`의
  `workspace_min/max`, `max_reach`를 현장에 맞게 줄인다(기본 ±0.8 m, 반경 0.85 m).
- 그립을 놓거나 컨트롤러 추적이 끊기거나 0.3 s 동안 데이터가 없으면 로봇은 자동으로 멈춘다. B를 누르고 있어도 멈춘다.

## 1. 네트워크 (처음 한 번)

1. 로봇 IP: PolyScope → 설정 → 시스템 → 네트워크. 고정 IP를 권한다(예 192.168.1.10).
2. PC를 같은 서브넷에(유선 권장). `ping <로봇 IP>` 가 되어야 한다. 사용 포트: 30004(RTDE), 30002, 29999(Dashboard), 63352(Robotiq URCap).
3. Quest는 같은 공유기의 Wi-Fi에. PC에서 8765 포트 방화벽 허용(관리자 PowerShell, 한 번만):
   ```powershell
   New-NetFirewallRule -DisplayName "quest-teleop 8765" -Direction Inbound -Protocol TCP -LocalPort 8765 -Action Allow
   ```
   PC가 유선+무선 둘 다면 Quest에서 접속할 주소는 **무선 어댑터의 IP**다(`ipconfig | findstr IPv4`).

## 2. 로봇 쪽 설정

1. PolyScope: 설정 → 시스템 → **Remote Control → Enable**. 화면 오른쪽 위 스위치를 **Remote**로.
   (Local이면 브리지가 `Failed to start RTDE data synchronization`으로 멈춘다.)
2. Installation → **TCP**: 실제 그리퍼 길이로 설정. 브리지가 보는 TCP 포즈가 이 값을 기준으로 한다.
3. 실행 중인 프로그램이 없어야 한다(브리지가 자체 제어 스크립트를 올린다). 다른 RTDE 클라이언트(ROS 드라이버 등)도 끈다.
4. 전원 ON + 브레이크 해제. `--robot.auto_power true`를 주면 Dashboard로 자동 수행된다.
5. 그리퍼(Robotiq 2F, URCap): `--gripper.type robotiq_urcap --gripper.ip <로봇 IP>`. 트리거를 당긴 만큼 닫힌다.

## 3. 브리지 띄우기

```powershell
cd G:\Claude\quest-ur5e-teleop
.venv\Scripts\python bridge\teleop_bridge.py --robot.type ur_rtde --robot.ip <로봇 IP> ^
    --robot.auto_power true --robot.speed_slider 0.3 --mapping.pos_scale 0.5
```
`[ur] connected … TCP=[…]`, `running` 이 보이면 준비. 이 시점부터 로봇은 그립을 쥐기 전까지 움직이지 않는다.

## 4. Quest에서 접속

1. 헤드셋 착용 → **브라우저** → `https://<PC 무선 IP>:8765` → 인증서 경고 **고급 → 계속** → **Start AR**.
2. HUD에 `mode=hmd`, `holding (squeeze grip to engage)`, PC 터미널에 `clients=1`.
3. 아직 그립을 쥐지 말고, 먼저 보정(5번)을 한다.

## 5. 첫 보정 (자리를 옮기면 다시)

1. 로봇 베이스에서 1 m쯤 떨어져 로봇을 바라보고 선다.
2. 썸스틱 **클릭** → 메뉴 → **보정**.
   ① 그립 쥔 채 **로봇 쪽으로 20 cm** 손을 민다(로봇은 움직이지 않음) → 로봇 방향(yaw) 결정
   ② 편한 조작 위치에 손을 두고 **트리거** → 조작 중심 기록
   ③ 확인: 그립 쥐고 로봇 쪽으로 10 cm → **이때 로봇이 실제로 10 cm 움직인다**. 로봇 +X(베이스 기준)로 갔으면 통과, 아니면 ①로.
   ④ **A** 저장 → `bridge\config.yaml`에 기록, 다음부터 자동.
3. 처음 몇 번은 **Z축 고정**(평면)과 **회전 고정**을 켜고 위치만 움직여 본 뒤 풀어 간다.
4. 익숙해지면 `--robot.speed_slider`를 0.5 → 1.0, `--mapping.pos_scale`을 1.0으로 올린다(메뉴 "이동 배율"로도 됨).

시뮬에서 저장한 `config.yaml` 보정값은 로봇 좌표 규약이 같아 실기에서도 유효하지만, **선 자리가 다르면 ①만 다시** 하면 된다.

## 6. 헤드셋을 벗고 쓰기 (선택)

1. 헤드셋이 잠들지 않게: `python tools\quest_keep_awake.py on`(개발자 모드+USB) 또는 근접 센서(렌즈 사이)에 테이프.
2. 브라우저 세션을 켠 채 벗어서 **카메라가 컨트롤러·작업 공간을 향하도록** 올려둔다. 컨트롤러가 시야를 벗어나면 로봇이 멈춘다.
3. 3초 뒤 `mode=headless`가 되고 메뉴가 잠긴다. 축 고정·배율·녹화·재생은 PC의 `https://<PC-IP>:8765/monitor.html`에서.
4. 실기에서는 헤드셋을 벗은 상태에서도 **로봇이 보이는 자리**를 유지한다.

## 7. 끝내기

1. 그립을 놓고(로봇 정지) → PC 터미널 **Ctrl+C**(servoStop, 제어 스크립트 종료).
2. PolyScope 스위치를 **Remote → Local**. 필요하면 브레이크 잠금·전원 OFF.
3. 녹화 파일은 `runs\rec_*.jsonl`. 다음에 메뉴 "마지막 녹화 재생" 또는 모니터 페이지로 재생할 수 있다(시작점까지 속도 제한 안에서 접근 후 재생, B로 중단).

## 8. 문제가 생기면

| 증상 | 확인 |
|---|---|
| `Failed to start RTDE data synchronization` | Remote 모드 아님, 프로그램 실행 중, 다른 RTDE 클라이언트 접속 중 |
| `UNSAFE: PROTECTIVE STOP` / `EMERGENCY STOP` | 펜던트에서 해제 후 브리지 재시작(자동 복귀하지 않음) |
| `UNSAFE: robot powered off` | 전원·브레이크. `--robot.auto_power true` |
| 로봇이 반대/엉뚱한 방향 | 메뉴 → 보정 ①~③ 다시. 급하면 `--mapping.operator_yaw_deg 180` |
| 손보다 늦게 따라옴 | 정상(속도 제한 0.25 m/s, EMA 필터). `--mapping.pos_filter_alpha 0.5`, `--robot.lookahead 0.05`로 빠르게 |
| 떨림 | `--robot.lookahead 0.15`, `--mapping.pos_filter_alpha 0.25` |
| `state: stale` 가 자주 뜸 | Wi-Fi 혼잡. 공유기를 로봇 가까이, 5 GHz 사용 |
| 페이지가 안 열림 | 같은 Wi-Fi, `https://`, 방화벽 8765, 무선 IP로 접속했는지 |

세부 파라미터(servoL 주기·이득 등)는 `docs/UR5E_SETUP.md`.
