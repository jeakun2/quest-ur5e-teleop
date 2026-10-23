# Isaac Sim 4.2에서 UR5e 텔레오퍼레이션

브리지(`--robot.type isaac`)가 Isaac Sim 안의 UR5e를 URSim/실기와 같은 방식(TCP 포즈 서보)으로 움직인다.

```
Quest/WebXR ──ws──▶ bridge/teleop_bridge.py ──TCP 5560 (JSON lines)──▶ isaac/ur5e_sim.py (Isaac Sim 4.2, Lula RMPflow)
```

## 요구 사항

- Isaac Sim 4.2. 파이썬 환경에 numpy·scipy가 있어야 한다. Isaac Sim standalone의 `python.bat`에는 numpy가 없으므로
  Isaac Lab을 설치할 때 만든 conda 환경(기본 이름 `isaaclab`)을 쓴다. `tools/isaac_sim.py`가 `activate.bat <env>`의 환경변수를
  받아 그 python으로 `isaac/ur5e_sim.py`를 띄운다(`--conda-root`, `--conda-env`로 변경, conda 없이 쓰려면 `--isaac <Isaac 루트>`).
  Isaac Lab API는 쓰지 않는다.
- UR5e USD는 NVIDIA Nucleus 공개 자산(`Isaac/4.2/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd`)을 처음 한 번 내려받는다.
  RMPflow 설정은 Isaac Sim에 들어 있는 `motion_policy_configs/universal_robots/ur5e`를 쓴다.
- GPU를 다른 작업과 나눠 쓰는 PC라면 각자의 대기열/스케줄러로 아래 명령을 감싸 띄우면 된다(약 5 GB VRAM).

```
.venv\Scripts\python tools\isaac_sim.py [--headless] [--log isaac\logs\sim.log]
```

## 실행

1. 시뮬레이터: 위 명령(창을 보려면 `--headless` 생략). 로그 `isaac/logs/sim*.log`에 `READY`와 `listening on 127.0.0.1:5560`이 찍히면 준비.
   첫 실행은 USD 다운로드·셰이더 컴파일로 1~3분 걸릴 수 있다.
2. 브리지: `.venv\Scripts\python bridge\teleop_bridge.py --robot.type isaac` (+ `--source.*` 옵션은 그대로).
   시뮬레이터가 아직 안 떠 있으면 최대 2분 동안 접속을 재시도한다.
3. 조작·메뉴·보정·녹화/재생은 `docs/OPERATION.md`와 동일. 스모크 테스트도 그대로: `tools/smoke_test.py <port>`.

옵션(`tools/isaac_sim.py` 뒤에): `--port 5560`, `--hz 120`(물리 주기), `--stale 0.3`(서보가 끊기면 정지),
`--home q1,…,q6`(시작 관절각, 기본 `0,-90°,90°,-90°,-90°,0`), `--selftest`(브리지 없이 10 cm 사각형 추종 후 종료,
결과 `isaac/logs/selftest.json`), `--shot path.png`(창 모드에서 기동 8초 후 스크린샷), `--duration s`,
`--render-hz 60`(창 모드 렌더 주기; 물리 120 Hz는 그 사이에 2회 서브스텝), `--no-gripper`, `--no-cube`.

실행 중 제어: `python tools/isaac_ctl.py status | shot <png> | quit | servo x y z rx ry rz`.
창 모드 스크린샷은 프로세스당 첫 장만 믿을 수 있다(이후 캡처가 검게 나오는 경우가 많음, Isaac 4.2 뷰포트 캡처 문제).

## 그리퍼 정렬·TCP

- 조립 직후 Robotiq USD의 손가락 메시 바운딩박스(USD 기본 시각)로 손끝 중심과 벌림 축을 재서, 손가락이 `tool0` +Z, 벌림이 +X가 되도록
  고정 조인트 회전을 `set_fixed_joint_transform`으로 맞춘다(두 후보 회전 중 점수가 높은 쪽). 링크 원점은 모두 그리퍼 베이스에
  겹쳐 있어 원점이나 물리 포즈로는 잴 수 없다(메시 바운딩박스만 유효).
- **TCP = tool0 + 0.121 m(손끝 중심)**. 보고·명령 포즈 모두 TCP 기준이며, RMPflow 목표는 내부에서 tool0로 환산한다. `--tcp-offset`로 바꿀 수 있다.
- 홀로그램(`web/ghost.js`)은 `ui.tcp_offset`을 받아 그리퍼 시각을 같은 길이로 스케일하고 TCP 좌표축을 손끝에 둔다.

## 오버레이 카메라 / 시점

- 천장(`/World/cams/ceiling`), 사이드(`/World/cams/side`), EE(`/World/ur5e/tool0/ee_cam`, 손끝 방향) 카메라. 켜질 때만
  Replicator 렌더 프로덕트(320×240)를 만들고 끄면 없앤다. JPEG은 `http://127.0.0.1:5561/cam/<name>.jpg`.
- `{"view": "default|top|side|ee"}`로 메인 뷰포트 카메라 프리셋을 바꾼다(ee는 손끝을 따라감).
- 렌더가 느린 동안(공유 GPU)에도 로봇이 느려지지 않도록, 렌더 프레임 뒤에 물리 전용 스텝으로 **시뮬 시간을 벽시계에 맞춘다**
  (`sim_lag` 로그). 창은 10~20 Hz로 갱신돼도 로봇 추종은 실시간이다.

## 그리퍼 조립 메모

- `omni.isaac.robot_assembler`는 python 앱에 기본 로드되지 않아 `enable_extension()` 후 import.
- 마운트 프레임은 그리퍼의 **리지드바디 링크**(`/Robotiq_2F_85/base_link`)여야 한다. 루트 Xform을 주면 조립은 되지만
  관절이 아티큘레이션에 합쳐지지 않고 물리가 발산한다.
- `finger_joint` 목표를 즉시 점프시키면 발산 → 1.5 rad/s 램프, 0.7 rad 클램프(`--finger-rate`, `--finger-close`).
  드라이브 게인·최대힘은 `--finger-kp/--finger-kd/--finger-max-effort`로 바꿀 수 있다(기본은 USD 값: kp 9.7, maxF 16.5).
- `Duplicate link name 'base_link'` 경고(UR5e와 그리퍼 모두 base_link)는 남아 있지만 동작에는 지장이 없었다.
  MovePrim으로 이름을 바꾸려 했으나 참조된 프림이라 적용되지 않는다.

## 창 깜빡임

창 모드에서 물리 스텝마다 렌더 여부를 번갈아 바꾸면(이전 구현) 화면이 깜빡인다. 지금은 **렌더 주기마다 한 번만
`world.step(render=True)`** 하고 물리(1/120)는 그 안에서 서브스텝된다. 렌더 주기는 첫 `reset()` 뒤 60프레임을 실측해
`n × physics_dt`(n ≤ 12)로 **한 번만** 정한다 — 실행 중 `set_simulation_dt`를 부르면 Kit가 멈춘다(겪음).
공유 GPU에서 프레임 26 ms → 주기 33 ms(4 서브스텝), 30 Hz 루프로 시뮬 시간이 실시간을 따라간다. 루프가 설정 주기의 70% 아래로
떨어지면 경고를 찍는다(그때는 시뮬 시간이 실시간보다 느려진다).

## 검증 (2026-10-21, 이 PC)

| 항목 | 결과 |
|---|---|
| 셀프테스트(헤드리스, 10 cm 사각형) | 120 Hz, 정착 후 오차 1~2 mm |
| `smoke_test.py`(헤드리스) | 메뉴·Z 고정·녹화(125f)·+10 cm 제스처·재생 복귀 "재생 완료" |
| `smoke_test.py`(창 모드) | 동일 통과, 시뮬 80~105 Hz(GPU를 다른 학습과 공유 중) |
| 브리지 루프 | 120 Hz 유지(요청/응답 2회/주기) |

## 좌표계·동작

- 보고/명령 포즈는 **UR 컨트롤러의 Base 프레임**(URDF `base_link`를 Z축으로 180° 돌린 것), 회전은 회전벡터 —
  `ur_rtde`의 `getActualTCPPose`/`servoL`과 같은 규약이라 `config.yaml`의 `mapping`(yaw, center, r_map)을 실기와 공유할 수 있다.
- 엔드이펙터는 `tool0`(플랜지). **Robotiq 2F-85**(Nucleus `Robotiq_2F_85_edit.usd`)를 Robot Assembler로 `tool0`에 붙이고
  브리지의 트리거 값(`gripper` 0..1)으로 `finger_joint`를 닫는다(`--no-gripper`로 생략). 손끝은 tool0에서 약 15 cm 앞이다.
- 그립 테스트용 **5 cm 주황 큐브**가 홈 TCP 바로 아래 바닥(z 0.025)에 놓인다(`--no-cube`). tool0를 큐브 위 ~0.16 m까지 내려
  트리거를 당기면 집힌다. 그리퍼·팔 사이 충돌은 마스킹돼 있다.
- 추종은 RMPflow(관절 속도 상한 2.15 rad/s, 관절 한계 회피)이며, 목표가 작업공간 밖이면 가능한 데까지만 간다.
  브리지의 속도 제한·작업공간 클램프가 먼저 적용되므로 보통 목표와 실제 TCP 오차는 수 mm 안에서 수렴한다.
- 서보가 `--stale`초 이상 끊기거나 `hold`를 받으면 현재 포즈에 정지한다(브리지의 `servoStop`에 해당).

## 프로토콜 (`isaac/ur5e_sim.py` 머리말과 동일)

| 보냄 | 뜻 |
|---|---|
| `{"servo":[x,y,z,rx,ry,rz], "gripper":0..1}` | 이 포즈를 추종 (+ 그리퍼 닫힘 정도, 어느 메시지에나 실을 수 있음) |
| `{"hold":1}` | 현재 포즈에 정지 |
| `{"get":1}` | 상태만 |
| `{"shot":"path.png"}` | 창 모드에서 스크린샷 |

응답: `{"tcp":[6],"q":[6],"grip":0..1,"t":sim_time,"hz":loop_hz,"tracking":bool,"ok":true}`. 여러 클라이언트 동시 접속 가능.
`{"quit":1}`로 종료.
