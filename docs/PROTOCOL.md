# 클라이언트 ↔ 브리지 프로토콜

전송: WebSocket 텍스트 프레임, JSON 1개/프레임. WebXR·Unity 클라이언트·PC 모니터 페이지가 같은 형식을 쓴다.
URL: `wss://<PC>:8765/ws` (WebXR, TLS) 또는 `ws://<PC>:8765/ws` (`--source.tls false`, Unity).

## 클라이언트 → 브리지: 컨트롤러 프레임 (72~90 Hz)

```json
{"t": 12.345,
 "right": {"p": [x, y, z], "q": [qx, qy, qz, qw], "grip": 0.0, "trigger": 0.0,
           "a": false, "b": false, "stick": false, "axes": [0.0, 0.0], "tracked": true},
 "left":  {... 같은 형식 ...} | null,
 "head":  {"p": [x, y, z], "q": [qx, qy, qz, qw]} | null}
```

- 좌표계("XR 프레임"): 오른손, 미터, +Y 위, −Z 전방(세션 시작 시 바라본 방향), +X 오른쪽.
  WebXR `local-floor` 레퍼런스 공간과 OpenXR 로컬 공간이 이 규약이다.
  Unity(왼손, +Z 전방)는 `p=(x,y,−z)`, `q=(−qx,−qy,qz,qw)`로 바꿔 보낸다.
- `p`,`q`는 **그립 포즈**(컨트롤러 손잡이). 쿼터니언 순서는 (x, y, z, w).
- `grip`,`trigger`: 0..1 아날로그. `a`/`b`: 오른손 A/B, 왼손은 X/Y. `stick`: 썸스틱 클릭.
- `axes`: 썸스틱 (x, y), **WebXR 게임패드 규약: y 음수 = 앞(위)**. Unity·oculus_reader는 y를 뒤집어 맞춘다.
  메뉴에서는 y < −0.6 = 위, y > 0.6 = 아래로 커서가 움직이고 |y| < 0.3에서 다시 준비된다.
- `tracked`가 false이면 브리지가 클러치를 푼다.
- `head`는 헤드셋 착용 판정에 쓴다(3초간 2 cm·3° 이내로 정지 → headless). 없으면 항상 headless.
- 250 Hz보다 빠르게 오는 프레임은 파싱하지 않고 버린다(제어 루프 보호, 상태 줄 `dropped=`).

## 클라이언트 → 브리지: 기타 메시지

| 메시지 | 의미 |
|---|---|
| `{"type":"ping","t":...}` | `{"type":"pong","t":...}`가 돌아온다(RTT) |
| `{"type":"visibility","state":"hidden"}` | WebXR 세션 가시성 변화(헤드셋 벗음 등), 로그용 |
| `{"type":"cmd","cmd":...}` | 아래 UI 명령. 모니터 페이지·헤드셋 어느 쪽에서 보내도 된다 |

`cmd` 목록:

| cmd | 인자 | 동작 |
|---|---|---|
| `menu` / `up` / `down` / `select` / `back` | | 메뉴 열고 닫기·이동·선택 (썸스틱/A/B와 동일) |
| `lock` | `axis: x|y|z|rot`, `on: bool`(없으면 토글) | 축 고정 |
| `record` | `on: bool`(없으면 토글) | 녹화 시작/정지 → `runs/rec_*.jsonl` |
| `replay` | `path`(없으면 최근 녹화), `speed` | TCP 궤적 재생 (B 또는 `stop`으로 중단) |
| `stop` | | 재생 중단, 진행 중인 보정/튜토리얼 취소, 메뉴 닫기, 클러치 해제 |
| `scale` | `value` | 이동 배율 |
| `map_mode` | `value: relative|absolute` | 매핑 방식 (absolute는 보정 2단계가 선행돼야 함) |
| `ui_mode` | `value: auto|headless|hmd` | 모드 강제 |
| `rot` | `on: bool`(없으면 토글) | 회전 추종 |
| `yaw` | `value`(deg) | `operator_yaw_deg` 직접 지정 |
| `calib` / `calib3` / `tutorial` | | 흐름 시작 |
| `confirm` / `cancel` / `trigger` | | 흐름 안에서 A / B / 트리거 대신 (제스처 단계는 실제 컨트롤러 필요) |
| `ghost` | `hide: bool` | 다음 그립 때 홀로그램을 손에 맞춤 / `hide`면 숨김 |
| `path_clear` | | AR 경로 지우기 (`ui.path_epoch` 증가) |
| `ghost_move` | `stop: bool` | 홀로그램 옮기기 모드 시작/완료 (`ui.ghost_move`) |
| `home` | | 시작 TCP로 복귀(속도 제한 접근, 그리퍼 열림) |
| `scene_reset` | | Isaac 씬 전체 초기화(물리 리셋·홈 관절·그리퍼 열림·큐브 복귀). 물리가 발산하면 시뮬이 스스로도 수행 |
| `cam` | `name: ceiling|side|ee`, `on: bool`(없으면 토글) | Isaac 오버레이 카메라 (`ui.cams`) |
| `view` | `value: default|top|side|ee`(없으면 순환) | Isaac 뷰포트 시점 (`ui.view`) |

## 브리지 → 클라이언트: 상태 (15 Hz)

```json
{"type": "status", "state": "ENGAGED", "rot": true, "robot": "ur_rtde", "hand": "right",
 "tcp": [x, y, z, rx, ry, rz], "q": [6 관절각 rad] | null, "gripper": 0..1, "engaged": true,
 "ui": {"mode": "hmd", "mode_cfg": "auto",
        "locks": {"x": false, "y": false, "z": true, "rot": false},
        "scale": 1.0, "map_mode": "relative", "yaw": -90.0, "msg": "Z 고정 켜짐",
        "menu": {"items": [{"key": "lock_z", "label": "Z축 고정", "value": "켜짐", "sel": true}, ...]} | null,
        "flow": {"name": "calib", "title": "보정: 로봇 방향·조작 위치", "step": 0, "n": 4, "lines": ["..."]} | null,
        "cams": {"ceiling": false, "side": false, "ee": true} | null, "view": "default" | null,
        "tcp_offset": 0.121 | null, "home": true,
        "ghost": {"p": [x, y, z], "q": [qx, qy, qz, qw]} | null,   // XR 프레임 기준 로봇 Base 프레임 포즈
        "path_epoch": 0,                                            // 바뀌면 클라이언트가 경로를 지운다
        "recording": {"path": "rec_20261020_210439.jsonl", "frames": 85, "seconds": 2.0} | null,
        "playing": {"path": "...", "phase": "approach|play", "progress": 0.42} | null}}
```

`state`: `ENGAGED[ clamp][ rate]` | `holding (squeeze grip to engage)` | `stale` | `no controller` |
`soft stop (B held)` | `MENU (robot holding)` | `<흐름 제목> i/n | <텔레옵 상태>` | `PLAYING approach` | `PLAYING 42%` |
`UNSAFE: <이유>`. `tcp`는 로봇 베이스 기준 TCP(회전벡터). 클라이언트는 `ui`를 그대로 그리면 된다(메뉴·흐름 로직은 브리지에 있다).
`ui.ghost`는 첫 그립 때 브리지가 정한다: 로봇 Base 프레임의 TCP가 그 순간의 컨트롤러 위치에 오도록, 회전은 `effective_r_map^T`.
WebXR 페이지(`web/ghost.js`)는 이 포즈 아래에 ur5e.urdf 체인(관절 원점·메시 오프셋)을 그려 `q`로 관절을 돌린다.

## oculus_reader 경로

APK가 logcat에 `wE9ryARX: r:<4x4 16개 숫자>|l:<...>&<버튼들>` 형식으로 찍고, Python
`OculusReader.get_transformations_and_buttons()`가 `{'r': 4x4, 'l': 4x4}`, 버튼 dict
(`A B RThU RJ RG RTr rightJS rightTrig rightGrip ...`)를 돌려준다. `oculus_reader_source.py`가
이를 위와 같은 `Sample`로 바꾼다(머리 포즈 없음 → headless). 4x4의 좌표계는 OpenXR 로컬 공간(동일 규약).
`keep_awake: true`면 시작 시 `am broadcast -a com.oculus.vrpowermanager.prox_close`를 보내 헤드셋을 벗어도 깨어 있게 한다.

## 오버레이 카메라 이미지

헤드셋 페이지는 `GET /cam/<name>.jpg`(브리지 https)로 최신 JPEG을 받는다. 브리지는 이를 Isaac 시뮬의 `http://127.0.0.1:5561/cam/<name>.jpg`
에서 받아 그대로 돌려준다(`source.cam_proxy`). 켜진 카메라만 렌더·인코딩된다(8 Hz).

## Isaac Sim 경로

`isaac/ur5e_sim.py`와의 TCP/JSON-lines 프로토콜은 `docs/ISAAC_SIM.md`. 브리지 메시지에 `gripper: 0..1`이 실리면 시뮬의
Robotiq 2F-85가 그만큼 닫히고, 응답의 `q`가 상태의 `q`로 전달된다.

## 기록/재생

- 녹화 파일(`--record` 또는 메뉴 녹화): 첫 줄 `{"meta": {...}}`, 이후 매 샘플
  `{...컨트롤러 프레임..., "tcp": [6], "engaged": bool, "gripper": 0..1}`.
- `--source.type replay --source.path runs/x.jsonl`: **컨트롤러 샘플**을 다시 매퍼에 넣어 재생(매핑 테스트).
- `cmd: replay`: **TCP 궤적**을 직접 servo로 재생(동작 재현). 현재 위치에서 첫 포즈까지 속도 제한으로 접근 후 시작.
