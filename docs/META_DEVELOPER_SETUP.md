# 메타 개발자 계정 · Quest 3 개발자 모드

**WebXR 경로(기본)는 이 문서가 필요 없다.** APK를 올리는 경로(oculus_reader, Unity)만 해당된다.

## 어떤 계정을 쓰나

SNS 자동화(`G:\Claude\social`)의 Instagram 앱은 developers.facebook.com 에 만든 Meta 앱이다.
그 앱을 만든 **Meta(페이스북) 계정**이 곧 메타 개발자 계정이며, Quest 개발도 같은 계정으로 한다.
`.env`의 토큰·앱 ID는 Instagram Graph API용이라 Quest 개발에는 쓰이지 않는다. 필요한 것은
계정 로그인뿐이다.

- 계정 확인: https://developers.facebook.com/apps 에 로그인되는 계정 = 사용할 계정.
- Quest 3의 Meta 계정이 다른 계정이면, 아래 조직에 그 계정을 멤버로 추가하거나
  헤드셋 계정으로 조직을 따로 만든다(무료). 개발자 모드는 **헤드셋에 로그인된 계정**이 어떤
  개발자 조직에든 속해 있으면 켜진다.

## 1. 개발자 조직 만들기 (1회)

1. https://developers.meta.com/horizon/manage/organizations/create/ 접속, 위 Meta 계정으로 로그인.
2. 조직 이름 입력(예: `KITECH DT Lab`), 약관 동의.
3. 계정 인증 요구 시: 휴대폰 번호 인증 또는 결제수단 등록 중 하나. **2단계 인증(2FA)** 활성화도 요구된다.
   - 2026년 기준 Meta는 "verified developer" 상태를 요구한다. 상태는
     https://developers.meta.com/horizon/manage/verify/ 에서 확인.

## 2. 헤드셋 개발자 모드 켜기

1. 휴대폰 **Meta Horizon** 앱 → 메뉴 → 기기 → Quest 3 선택 → **헤드셋 설정** → **개발자 모드** → ON.
   (조직에 속하지 않은 계정이면 토글이 보이지 않는다 → 1단계로.)
2. 헤드셋 재시작.
3. USB-C로 PC에 연결하고 헤드셋을 쓰면 "USB 디버깅 허용" 팝업 → **항상 허용** 체크 후 허용.
4. PC에서 확인:
   ```
   C:\Users\USER\AppData\Local\Android\Sdk\platform-tools\adb.exe devices
   ```
   `XXXXXXXX  device` 가 보이면 완료. `unauthorized`면 헤드셋에서 팝업을 다시 허용한다.
   Windows에서 장치가 안 보이면 Oculus ADB 드라이버를 설치한다:
   https://developers.meta.com/horizon/downloads/package/oculus-adb-drivers/

## 3. oculus_reader APK 설치 (자동)

`--source.type oculus_reader`로 브리지를 처음 실행하면 `third_party/oculus_reader/oculus_reader/APK/teleop-debug.apk`
를 adb로 설치하고 앱을 띄운다. 헤드셋 안에서는 "알 수 없는 출처" 라이브러리에 `teleop` 앱이 생긴다.

Wi-Fi로 쓰려면 한 번 USB를 꽂고:
```
adb tcpip 5555
adb shell ip route        # src 뒤의 IP가 헤드셋 IP
```
`config.yaml`의 `source.ip_address`에 그 IP를 넣는다.

## 4. Unity 앱 (선택)

`unity/README.md` 참조. 빌드한 APK는 `adb install -r <apk>`로 올린다. Meta Quest Developer Hub
(MQDH)를 설치하면 케이블 없이 APK 설치·로그 보기가 편하다.

## 주의

- 개발자 모드는 보안상 측면 로드를 허용하는 상태다. 공용 헤드셋이면 작업 후 끈다.
- 메타 계정 비밀번호·2FA 코드는 채팅에 적지 않는다. 포털 작업은 사용자가 직접 한다.
