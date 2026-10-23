# UR5e 설정 (실기 · URSim)

## 실제 UR5e

1. **네트워크**: PC와 로봇을 같은 서브넷에. PolyScope → 설정 → 시스템 → 네트워크에서 로봇 IP 확인.
   `ping <로봇 IP>` 가 되어야 한다. 포트 30004(RTDE), 30002(URScript 업로드), 29999(Dashboard)를 쓴다.
2. **Remote Control**: 설정 → 시스템 → **Remote Control** → **Enable**. 그 뒤 화면 오른쪽 위의
   Local/Remote 스위치를 **Remote**로 바꾼다. (Local이면 ur_rtde가 `Failed to start RTDE data synchronization`으로 실패.)
3. 로봇 전원 ON + 브레이크 해제. (`--robot.auto_power true`를 주면 Dashboard로 자동 수행.)
4. 실행 중인 프로그램이 없어야 한다(ur_rtde가 자체 제어 스크립트를 올린다).
5. 안전: 처음엔 `--robot.speed_slider 0.3 --mapping.pos_scale 0.5`. TCP 설정(툴 길이)은 PolyScope의
   Installation → TCP에서 실제 그리퍼에 맞춘다. 작업공간 박스는 `config.yaml`의 `workspace_min/max`.
6. 그리퍼: Robotiq 2F를 URCap으로 쓰면 `--gripper.type robotiq_urcap --gripper.ip <로봇 IP>`.

```
.venv\Scripts\python bridge\teleop_bridge.py --robot.type ur_rtde --robot.ip 192.168.1.10 ^
    --robot.auto_power true --robot.speed_slider 0.3 --mapping.pos_scale 0.5
```

## URSim (도커, 로봇 없이 전체 경로 검증)

```
tools\ursim_up.ps1            # Rancher Desktop(도커) 필요. 이미지 universalrobots/ursim_e-series, ROBOT_MODEL=UR5
```
이 스크립트는 컨테이너를 띄우고, Dashboard로 전원을 켜고, PolyScope가 Remote 모드가 아니면
VNC(5900)로 클릭을 자동 수행한다(`tools/ursim_enable_remote.py`). 화면은 http://localhost:6080 (noVNC)로도 볼 수 있다.

수동으로 할 때의 순서: 안전설정 Confirm → Getting Started 닫기 → ≡ → Settings → System → Remote Control → Enable → Exit → 상단 Local → Remote Control.

검증:
```
.venv\Scripts\python tools\make_demo_recording.py runs\demo.jsonl
.venv\Scripts\python bridge\teleop_bridge.py --source.type replay --source.path runs\demo.jsonl --source.loop false ^
    --robot.type ur_rtde --robot.ip 127.0.0.1 --robot.auto_power true --duration 14
```

## servoL 파라미터

| 키 | 기본 | 의미 |
|---|---|---|
| `robot.frequency` | 125 | 제어 주기(Hz). e-series는 500까지. Wi-Fi 지연이 크면 125가 안정적 |
| `robot.lookahead` | 0.1 | servoL look-ahead(s). 크면 부드럽고 느림 (0.03~0.2) |
| `robot.gain` | 300 | servoL 비례 이득 (100~2000) |
| `mapping.pos_filter_alpha` | 0.35 | 위치 EMA. 1이면 필터 없음 |
| `mapping.max_lin_speed` | 0.25 | m/s 상한 |

## 문제 해결

- `Failed to start RTDE data synchronization` → Remote 모드가 아님(위 2번), 또는 다른 RTDE 클라이언트가 붙어 있음.
- 보호정지 후 → 펜던트에서 해제하고 브리지를 다시 실행(`UNSAFE` 상태에서 자동 복귀하지 않음).
- 로봇이 반대 방향으로 움직임 → `calibrate_frame.py`로 `r_map` 재보정하거나 `--mapping.operator_yaw_deg 180`.
