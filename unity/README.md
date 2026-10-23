# Unity 네이티브 클라이언트 (선택)

WebXR 클라이언트(`web/`)로도 충분하지만, 패스스루 위에 UR5e 모델·가상 UI를 겹쳐
보이게 하려면 유니티 앱이 낫다. 기존 프로젝트 `G:\Works\Unity\teleop`
(Unity 2022.3.31f1, OpenXR 1.10, XR Interaction Toolkit 3.0.3)에 그대로 얹을 수 있다.

## 넣는 법

1. `QuestTeleopStreamer.cs`를 `Assets/Scripts/`에 복사하고 씬의 아무 GameObject(예: XR Origin)에 붙인다.
2. Inspector의 `serverUrl`을 `ws://<브리지 PC IP>:8765/ws`로 바꾼다.
3. 브리지는 TLS를 끄고 띄운다 (유니티의 `ClientWebSocket`은 자체서명 인증서를 거부한다):
   ```
   .venv\Scripts\python bridge\teleop_bridge.py --source.tls false
   ```
4. (선택) 월드스페이스 `TextMesh`를 `hudText`에 연결하면 브리지 상태(ENGAGED/holding 등)가 보인다.

## Quest 3 빌드 설정 (Meta 공식 권장값)

- File → Build Settings → **Android**, Texture Compression ASTC
- Player: Minimum API 32, Scripting Backend IL2CPP, Target Architectures **ARM64**만
- XR Plug-in Management → Android 탭 → **OpenXR** 체크
  → OpenXR Feature Groups에서 **Meta Quest Support** 체크, Interaction Profile에
  **Oculus Touch Controller Profile** 추가
- 패스스루가 필요하면 Meta XR Core SDK(`com.meta.xr.sdk.core`, UPM)의 Passthrough 기능 또는
  OpenXR `XR_FB_passthrough` 지원 패키지를 추가한다.
- 빌드된 APK는 `adb install -r teleop.apk`로 설치 (개발자 모드 필요, `docs/META_DEVELOPER_SETUP.md`).

## UR5e 모델을 VR에 표시하기

`G:\Works\Unity\Unity_ROS\Assets\Robots\ur5e\ur5e.urdf`(URDF Importer로 가져온 모델)를
복사해 오면 된다. 브리지 상태 메시지의 `tcp`([x,y,z,rx,ry,rz], 로봇 베이스 기준)를 받아
가상 TCP 마커를 그리거나, `rtde_receive.getActualQ()`를 브리지에서 함께 보내도록 확장하면
조인트 상태까지 겹쳐 볼 수 있다 (프로토콜 확장: `docs/PROTOCOL.md`).

좌표 변환 주의: 유니티는 왼손 좌표계(+Z 전방)이고 로봇/XR 프레임은 오른손 좌표계다.
스크립트는 보낼 때 `p=(x,y,-z)`, `q=(-x,-y,z,w)`로 바꿔 보낸다. 로봇 베이스 프레임을
유니티에 그릴 때는 (x_robot, z_robot, y_robot)처럼 Y/Z를 바꾸고 회전을 맞춰야 한다.
