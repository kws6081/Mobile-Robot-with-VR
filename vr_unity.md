# ROS#을 이용한 Oculus Quest 2 ROS message 수신
  <br/><br/>

### 1. Test environment

- Unity : `2021.3.1f1 LTS`
- ROS# : `version 1.6`

1. Unity 개발 환경 구축
  - Unity Hub 실행
  - `install`버튼을 선택 시 `Android Build Support`플러그 인 추가
  - `Android SDK`, `Android JDK` 모두 선택
2. 보유한 Oculus Quest 2의 개발자 모드를 유효화
  - [링크](https://developer.oculus.com/manage/)에서 Organization 작성
  - 휴대폰 Oculus 앱에서 `설정`-`기타 설정`-`개발자 모드`-`on` 선택
3. 프로젝트 작성
  - 3D 템플릿 선택

  <br/>

### 2. Build Settings

1. Build Setting 변경
  - `File`-`Build Setting` 선택
  - Platform에서 `Android` 선택 후 `Switch Platform` 클릭
2. Oculus Integration
  - `Window`-`Asset Store` 선택 후 `Oculus Integration` 설치 후 import
3. XR Plug-in Management 설치
  - `Window`-`Package Manager` 선택
  - `XR Plug-in Management` 설치
4. `Player Setting` 변경
  - `Edit`-`Project Settings`-`Player`-`Other Settings` 선택
  - `Package Name` 설정
  - `Minimum API Level` : `Android 7.1 'Nougat'(API level 25)`
  - `API Compatibility Level` : `NET 4.x`
  - `XR Plug-in Management` : `Oculus`
5. ROS# import
  - [링크](https://github.com/siemens/ros-sharp/releases)에서 `RosSharp.unitypackage` 다운로드
  - `Asset`-`Import Package`-`Custom Package`에서 다운로드한 패키지 선택
6. 추가 세팅
  - `Edit`-`Project Settings`-`Player`-`Other Settings` 에서 추가 세팅
  - `Rendring`-`Color Space` : `Linear` 선택
  - `Configuration`-`Active Input Handling` : `Both` 선택


  <br/>

### 3. Main Camera Settings

1. `Hierarchy`-`Main Camera` 삭제
2. `Project`에서 `Asset/Oculus/VR/Prefabs` 열고 `OVRCameraRig`를 `Hierarchy`에 드래그 & 드롭
3. `OVRCameraRig`의 `OVR Manager` 스크립트의 Target Devices를 `Quest`로 변경

  <br/>

### 4. Controller Settings

![hierarchy](https://user-images.githubusercontent.com/68265609/170938632-c8cbd171-e25f-4695-b3e3-42f958ca7af7.png)

1. `Project`-`Assets/Oculus/VR/Prefabs`-`OVRControllerPrefab`을 `Hierarchy`-`RightControllerAnchor`에 드랍
2. `LeftControllerAnchor`에도 똑같이 하고 `Inspector`-`Controller`-`L Tracked Remote`로 변경
3. `Oculus`-`Tools`-`Remove AndroidManifest.xml`
4. `Create store-compatible AndroidManifest.xml`
5. 아래를 따라 코드 변경

```xml
<!--category android:name="android.intent.category.INFO"/-->
<category android:name="android.intent.category.LAUNCHER"/>
```

  <br/>

### 5. Build Test

- Unity와 Oculus Quest 2를 유선 연결 해준 뒤, `Build and Run` 실행
- Oculus Quest 2 실행 후, 알 수 없는 앱을 선택한 후 설정해 준 Package Name의 앱을 실행

  <br/>

### 6. ROS sharp

1. `Hierarchy`에서 오른쪽 클릭 후 `Create Empty`-`Game Object`, 이름을 `RosConnector`로 변경
2. `Project`-`Assets/RosSharp/Scripts/RosBridgeClient/RosCommunication`
3. `RosConnector` script를 `RosConnector` Object에 드래그 & 드롭

  <br/>

### 7. PC setup

- 아래의 명령을 통해 `RosBridge Server` install

```bash
$ sudo apt-get install ros-<rosdistro>-rosbridge-server
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

- ROS PC와 Oculus Quest 2가 같은 Wifi에 연결되어야 함
- 연결 후 ROS PC terminal에 `Client Connected`가 뜨면 성공

  <br/>

### 8. Message setup

1. `Hierarchy`에서 오른쪽 클릭 후 `3D Object`-`Plane` 작성 후 `ImageReceiver`로 변경
2. `Inspector`에서 다음과 같이 변경
  - `Position` : (0,0,10)
  - `Rotation` : (90,0,0)
  - `Scale` : (-1,-1,0.75)
3. `Project`에서 `Assets/RosSharp/Scripts/RosBridgeClient/RosCommunication` 
  - `ImageSubscriber` 스크립트를 `RosConnector` Object에 터치
  - 터치한 `ImageSubscriber`의 Topic을 `/camera/color/image_raw/compressed`로 설정
  - `MessageReceiver`에 `ImageReceiver` Object를 드래그 & 드랍
4. PC 설정 (D435i 예시)

```bash
$ roslaunch rosbridge_server rosbridge_websocket.launch
$ roslaunch realsense2_camera rs_camera.launch
```

5. `Project`-`Assets/RosSharp/Scripts/RosBridgeClient/RosCommunication`에서

![topic](https://user-images.githubusercontent.com/68265609/170943660-a3dd495a-6ec9-44ea-b04b-edf69612a8b7.png)

  - `PoseStampedPublisher` 스크립트를 `RosConnector` Object에 터치
  - 이후 토픽 명을 설정
  - 세 개의 객체를 `Hierarchy`에서 드래그 & 드랍
  - 헤드셋 : `CenterEyeAnchor`
  - L컨트롤러 : `LeftControllerAnchor`
  - R컨트롤러 : `RightControllerAnchor`

6. `Project`-`Assets/RosSharp/RosBridgeClient/RosCommunication`에서
  - `JoyPublisher`스크립트를 `RosConnector` Object에 터치
  - 토픽 명 설정
  - `Assets/RosSharp/Scripts/RosBridgeClient/MessageHandling`에서
  - `JoyAxisReader` 스크립트와 `JoyButtonReader`를 필요한만큼 `RosConnector`에 터치
  - `JoyAxisReader`에서 읽고싶은 Name을 `JoyButtonReader`의 Name에 설정

  <br/>
  
### 9. Final Step

- ![링크](https://github.com/TakaHoribe/joy_to_twist) 에서 패키지 설치
- 해당 패키지를 통해 `Twist`메세지로 로봇 컨트롤 가능!