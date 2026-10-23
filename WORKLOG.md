# 작업 로그

## 2026-10-20 — 초기 구현

- 조사: `G:\Claude\social`의 메타 계정은 Instagram Graph API 토큰만 보유 → Quest 개발은 같은 Meta 계정으로
  developers.meta.com 조직 생성·개발자 모드 활성화가 필요(문서화). Unity 에디터는 이 PC에 없음,
  기존 `G:\Works\Unity\teleop`(OpenXR+XRI)·`Unity_ROS`(UR5e URDF) 자산 확인. adb는 Android SDK에 있음.
- 외부 프로젝트: `jborbik/oculus_reader`(Quest 3 지원 포크, APK 10.7MB 포함)를 `third_party/`에 복제.
- 구현: `bridge/`(소스 3종·매핑·ur_rtde·더미·그리퍼), `web/`(WebXR AR 클라이언트+HUD), `unity/`(C# 스트리머),
  `tools/`(데모 기록, URSim 기동·원격모드 자동화), `docs/` 3종.
- 검증: pytest 14건 통과. URSim 도커(PolyScope 5.26.1, UR5)에서 VNC로 Remote Control 활성화 후
  `--robot.type ur_rtde` 리플레이 실행 → TCP z +0.079 m 추종, 재생 종료 시 stale 정지.
- 버그 수정: websockets 17 정적 파일 응답(bytes), 스레드 내 SIGINT 핸들러.
- 미완: 실제 Quest 3 연결 테스트, 실제 UR5e 연결 테스트, Unity APK 빌드(에디터 없음).

## 2026-10-20 (2) — 조작 모드·메뉴·보정·녹화

- 요구: 그립(중지) 누른 채 움직이면 EE 기준 xyz+회전 텔레옵(헤드셋 없이도), 착용 시 보정·튜토리얼·축 고정·녹화 등.
- `bridge/session.py`: 머리 포즈 정지 3 s → headless(메뉴 잠금) / 움직임 → hmd 자동 판정, 썸스틱 메뉴(스틱 상하·A·B),
  튜토리얼 6단계(실제 입력으로 자동 진행), 보정 흐름(로봇 쪽 밀기 → yaw, 트리거 → 조작 중심, 10 cm 확인, A 저장),
  3축 정밀 보정, 녹화/재생, `cmd` 처리. `bridge/trajectory.py`: Recorder(샘플+TCP+그리퍼), TrajectoryPlayer(접근→재생).
- `bridge/mapping.py`: lock_x/y/z/rot, mode relative/absolute(center_T·tcp_center).
- `web/`: HUD에 메뉴·흐름·칩 렌더링, visibility 전송, 2D 테스트 키(m·화살표·t·h); `monitor.html` PC 제어 페이지.
- `tools/quest_keep_awake.py`(prox_close/automation_disable), oculus_reader 소스 keep_awake + 스틱 y 부호 통일, Unity도 y 반전.
- 문제: 검증 클라이언트가 Windows asyncio.sleep(1/72)≈0 으로 수천 Hz 플러딩 → 루프 2 Hz. 서버에 250 Hz 입력 상한 추가,
  테스트 클라이언트는 데드라인 기반 페이싱. HeadMotion은 scipy 루프가 느려 벡터화+50 ms 데시메이션.
- 검증: pytest 29건, URSim에서 smoke_test 전 과정(메뉴·Z 고정·녹화 87f·재생 완료) 확인.
- 미완: 실제 Quest 3/UR5e 미연결(헤드셋 벗은 상태의 트래킹 유지는 adb prox_close 또는 테이프로 사용자 확인 필요).

## 2026-10-21 — Isaac Sim 4.2 백엔드

- `isaac/ur5e_sim.py`: Isaac Sim 4.2 안의 UR5e(Nucleus `ur5e.usd`)를 Lula RMPflow(`tool0`)로 추종시키고 TCP/JSON-lines
  서버(5560)로 노출. 포즈는 UR Base 프레임(base_link를 Z로 180°) 회전벡터 — ur_rtde와 같은 규약. `servo/hold/get/shot/quit`.
  실시간 페이싱(winmm 1 ms), stale 0.3 s → hold, 창 모드는 물리 2스텝당 1렌더.
- `bridge/robots/isaac_robot.py`(`--robot.type isaac`), `tools/isaac_sim.py`(런처), `tools/isaac_ctl.py`(status/shot/quit/servo),
  `tests/test_isaac_robot.py` 2건, `docs/ISAAC_SIM.md`.
- 삽질: standalone `python.bat`에는 numpy가 없어 extension 로드가 줄줄이 실패 → conda `isaaclab` 환경의 python으로 실행
  (활성화 스크립트의 CARB_APP_PATH/EXP_PATH/ISAAC_PATH/PYTHONPATH를 `activate.bat && set`으로 받아 전달). ccd 대기열은 .bat를
  거부하므로 venv python 래퍼. 셀프테스트 첫 판정은 목표 전환 순간에 샘플링돼 FAIL로 나왔던 것 → 홀수 초 샘플로 판정.
  뷰포트 스크린샷(`capture_viewport_to_file`)은 프로세스당 첫 장 이후 대부분 검은 이미지 — 원인 미해결, 첫 장만 사용.
- 검증: pytest 31건, 헤드리스·창 모드 모두 smoke_test 전 과정 통과(상세는 README 검증 기록).

## 2026-10-21 (2) — Isaac 그리퍼·깜빡임, AR 홀로그램·경로, Quest QR

- **Isaac**: Robotiq 2F-85를 Robot Assembler로 `tool0`에 조립(확장 `omni.isaac.robot_assembler`는 `enable_extension`으로
  켜야 함; 마운트는 그리퍼의 리지드바디 링크 `base_link`). 14 DOF 단일 아티큘레이션, `finger_joint`를 트리거 값으로 구동
  (목표를 1.5 rad/s로 램프·0.7 rad 클램프 — 즉시 점프시키면 물리가 발산했음). 그립 테스트용 5 cm 큐브.
  브리지→시뮬 메시지에 `gripper`가 실리고(`IsaacGripper`, robot.type isaac이면 자동) 응답 `q`가 상태로 전달.
- **깜빡임**: 물리 스텝마다 렌더를 번갈아 끄던 것이 원인 → 렌더 주기마다 한 번만 step(render=True), 물리는 서브스텝.
  렌더 주기는 기동 시(첫 reset 후) 실측해 `set_simulation_dt`로 1회 설정(실행 중 바꾸면 Kit가 멈춤 — 실제로 겪음).
  공유 GPU에서 프레임 26 ms → 33 ms 주기(4 서브스텝), 30 Hz 루프, 시뮬 시간 = 실시간.
- **AR 홀로그램**: `web/ghost.js`(three.js 0.160 로컬 번들 + ur_description 충돌 STL 7개, URDF 관절 원점/메시 오프셋 하드코딩).
  브리지가 첫 그립 때 `ui.ghost`(XR←Base 변환: TCP가 손 위치에, 회전 `effective_r_map^T`)를 정하고 상태에 `q`·`gripper`·`engaged`
  를 15 Hz로 보냄. FK 검증: 홀로그램 tool0 vs 브리지 TCP 오차 0.0~0.1 mm. 경로: TCP(하늘색)·손(주황) 선, 메뉴 "경로 지우기".
  메뉴 "홀로그램 손에 맞추기", cmd `ghost`/`path_clear`, monitor.html 버튼.
- **Quest**: `bridge/netinfo.py`로 LAN IP 자동 선택(Wi-Fi 우선)·ASCII/PNG QR(`tools/show_qr.py`, 브리지 기동 시 출력).
  방화벽 8765 인바운드 규칙은 관리자 권한 필요(UAC 창). 헤드셋 페이지 로그(`XR start failed …`, 지원 여부)를
  `{"type":"log"}`로 브리지에 전달해 터미널에 `[headset]`로 출력 — VR 버튼 실패 원인 추적용.
- 검증: pytest 34건(ghost/gripper 3건 추가), 그리퍼 장착 시뮬에서 smoke_test 통과(+10 cm·재생), `tools/web_check.py`
  (Playwright 헤드리스 Chromium, 2D test 드래그)로 홀로그램·경로 렌더 확인, 그리퍼 닫힘 스크린샷 `isaac/logs/isaac_gripper_closed.png`.
- 남은 것: 실제 Quest에서 VR/AR 세션 진입 확인(사용자 테스트, 로그 전달됨). 창 모드 뷰포트 캡처는 여전히 프로세스당 첫 장만.

## 2026-10-21 (3) — 옵션 메뉴·초기 자세·오버레이 카메라·TCP·그리퍼 정렬

- A 버튼 = 옵션 메뉴(썸스틱 클릭과 동일), 회전 추종은 메뉴 항목으로. 메뉴에 초기 자세로(시작 TCP로 속도 제한 복귀), 천장/사이드/EE
  카메라 오버레이, Isaac 시점 전환 추가. cmd `home`/`cam`/`view`, 상태 `ui.cams/view/tcp_offset/home`, monitor.html 버튼.
- Isaac: Robotiq 2F-85를 손가락 메시 바운딩박스로 재서 `tool0` +Z 방향으로 정렬(이전엔 옆으로 누워 있었음), TCP = 손끝 중심(+0.121 m).
  오버레이 카메라 3대(Replicator 렌더 프로덕트, 켜질 때만 생성) + 시뮬 내 HTTP(5561) → 브리지가 `/cam/<name>.jpg`로 프록시.
  창 렌더가 느릴 때 물리 전용 스텝으로 시뮬 시간을 벽시계에 맞춤(공유 GPU에서 10 Hz 렌더여도 로봇은 실시간).
- 삽질: 링크 원점(USD·물리 모두)이 그리퍼 베이스에 겹쳐 있어 기하 측정이 0 → 메시 바운딩박스 사용. `omni.isaac.sensor`는 이 PC에서
  cp949 인코딩 오류로 로드 실패 → Replicator 애노테이터 직접 사용. `hydra_texture.set_updates_enabled` 토글은 데이터가 비어
  렌더 프로덕트를 생성/파괴로 대체. 다른 Claude 세션이 같은 시뮬/브리지를 껐다 켜서 충돌 → 이 세션이 단독 관리.
- 검증: pytest 37건(menu_options 3건 추가), smoke_test·web_check 통과, 그리퍼 정렬 스크린샷 `isaac/logs/isaac_gripper_aligned.png`,
  EE 카메라 프레임 `runs/web_check/cam_ee.jpg`(큐브가 손가락 사이에 보임).
- 추가(18:50): 버튼 유실 수정 — WebXR 프레임이 버스트로 와서 250 Hz 상한에 걸려 버려질 때 A/B/스틱/트리거 눌림을 다음 프레임에
  합쳐 전달(짧은 클릭이 사라져 메뉴가 안 열리던 문제). PC Wi-Fi(연구실 공유기)가 DHCP를 못 받아 169.254.x가 되던 문제는
  `tools/wifi_static.ps1`/netsh로 192.168.0.52 고정. 헤드셋 HUD는 서브에이전트가 그룹형 옵션 패널·카메라 패널·monitor.html 카드로 정리
  (`runs/web_check/hud_menu.png`).
- 추가(19:10): Isaac 씬 초기화(메뉴/모니터/cmd scene_reset, 물리 발산 자동 감지). AR 월드 메뉴(web/xrmenu.js): A를 누르면 오른손 컨트롤러 위에 패널이 확대·페이드로 뜨고 사용자를 향해 빌보드, 스틱/A/B 조작은 그대로. 헤드셋에서는 DOM 메뉴 숨김. 커밋 09ccddc.
- 추가(19:40): AR 카메라 창(web/xrcams.js) — 켜진 Isaac 카메라를 메뉴 오른쪽(메뉴 닫히면 메뉴 자리)에 12 cm 창으로 오른손에 붙여 유지, 8 Hz JPEG. 홀로그램 옮기기/돌리기 모드(그립 끌기·손목 yaw, 완료 시 operator_yaw 갱신). 커밋.
- 추가(20:05): Wi-Fi 복구 — 핫스팟을 끄면 USB Wi-Fi 라디오가 꺼진 채 남아 0x80342002로 재접속 실패 → Windows.Devices.Radios API로 라디오 켜고 공유기 재접속(tools/hotspot.ps1 off에 포함). 공유기 게이트웨이는 핑에 응답하지 않아도 PC↔Quest(192.168.0.55) L2 통신은 됨.
- 추가(20:50): Quest 3 실기 접속 확인 — 폰 핫스팟 LAN에서 https://<PC-IP>:8765, immersive-ar 세션 config #1(local-floor+dom-overlay) 통과, 메뉴 열림/조작 확인. 앞선 NotSupportedError는 그 전 세션이 끝나지 않은 상태의 재요청으로 추정. Isaac 메인 루프가 30분간 정지(소켓 스레드는 응답)하는 현상 1회 → 프로세스 재기동으로 해결, 루프 감시 필요.
- 2026-10-21 22:30 Isaac 재기동이 대기열에서 바로 종료돼(server_gui19.log 1줄) 시뮬 미기동 상태로 사용자 요청에 따라 전체 종료. 브리지의 Isaac 링크가 끊기면(시뮬 멈춤·재시작) 2초마다 스스로 재접속하도록 isaac_robot.py 수정(+테스트) — 다음 세션부터는 시뮬만 다시 띄우면 브리지 재시작 불필요.
- 2026-10-24 GitHub 공개 전환(https://github.com/jeakun2/quest-ur5e-teleop): README를 Isaac Sim(A)/실기(B) 두 트랙으로 재구성, 헤드셋 녹화 영상 대신 모니터 글자를 흐린 정지 컷 4장(docs/media) 사용(영상 자체는 개인정보 때문에 미업로드), 로컬 경로·대기열·공유기/핫스팟 이름 제거, MIT 라이선스. 공개 전 이력은 단일 커밋으로 압축(전체 이력은 로컬 브랜치 private-history).
