# M0609 + RG2 실행 및 팀 인계 계획

이 문서는 초기 마일스톤의 계획과 당시 상태를 보존한 기록이다. 현재 실행 절차와 결과는 [README](../README.md)와 [검증 기록](validation.md)을 참고한다.

## 현재 상태

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, MoveIt 2 2.12.4, gz_ros2_control 1.2.20은 기존 및 사전 확인 환경이다.
- `src/m0609_gz`에서 사용자가 MoveIt으로 팔을 이동하고 Gazebo의 RG2를 개폐했다. 재현 절차와 이전 검증 기록은 패키지 README에 있다.
- 로컬 `ws_dsr/src/doosan-robot2`는 상위 `cobot_rg2` 저장소에 포함된 복사본이다. 로컬에는 공식 `dsr_mujoco`와 MJCF가 없다. Gazebo 데모는 기존 `dsr_description2`, `m0609_rg2_bringup`, `onrobot_rg_description` 자산을 재사용한다.
- 임시 Python 3.12 환경에서 MuJoCo 3.17.0으로 공식 팔을 로드했다. 메시 경로를 맞춘 후 관절 6개, 액추에이터 6개를 확인했다. 첫 모터 입력에 2초 후 1축이 약 0.145 rad 움직였고 `mj_resetData` 후 1축과 시간이 0으로 돌아왔다. 이 결과는 팔 단독 로드·반응 확인이다. RG2 통합, 파지, GUI 및 새 체크아웃은 미검증이다.

## MuJoCo 구현 순서

1. **모델과 설치 환경 준비.** 두산 공식 `DoosanRobotics/doosan-robot2` 커밋 `243af5d1a071f030818bbcd6e9e5ee23d9edacae`의 `dsr_description2/mujoco_models/m0609/`에서 `m0609.xml`, `scene.xml`, `assets/`를 확보한다. 세 항목을 같은 모델 폴더에 두는 경우 XML의 메시 경로를 `m0609/assets/...`에서 `assets/...`로 정규화한다. 출처와 라이선스를 확인해 필요한 자산 및 고지를 포함하거나 고정 커밋에서 가져오는 스크립트를 둔다. 팀원 PC에 복사할 수 없는 절대 경로나 심볼릭 링크를 사용하지 않는다. Python 가상환경과 검증된 의존성 버전을 기록한다.
2. **팔 및 RG2 제어.** 공식 팔의 `<motor>` 입력을 관절 위치로 간주하지 않는다. 위치 서보 또는 PD 제어로 목표를 추종시킨다. `src/m0609_gz/urdf/m0609.urdf.xacro`와 설치된 RG2 매크로에서 `link_6 → tool0 → rg2_base_link`의 고정 변환, 관절축·제한, `rg2_finger_joint`와 다섯 mimic 관계를 가져온다. MJCF에서 mimic 부호·비율·오프셋을 확인하고 `rg2_tcp` site를 정의한다. 같은 관절 자세에서 Gazebo 모델과 TCP 위치·방향을 비교한다. 기존 열림 -0.50 rad와 닫힘 +0.70 rad는 개폐 비교에 쓴다.
3. **장면과 기준 시퀀스.** 고정 받침대에 자유 관절을 가진 40 mm 정육면체를 놓고, 열린 그리퍼로 접근한 뒤 닫고 상승한다. 동일 방향을 유지하는 사전 계산 경유점이나 IK를 활용한다. 패드 충돌 형상과 실제 접촉을 맞춘다. 파지 중 상자를 고정하거나 위치를 직접 바꾸지 않는다. 상자가 안정된 시작 높이를 기록하고, 상승 후 상자 중심이 그보다 50 mm 이상 높은 상태를 유지하는지 검증한다. 유지 시간은 예를 들어 시뮬레이션 시간 1초로 정하고 결과와 함께 기록한다.
4. **초기화와 검증.** `reset()`에서 팔·그리퍼·상자의 위치와 속도, 제어 입력, 에피소드 상태를 복원한다. 같은 시퀀스를 쓰는 GUI와 헤드리스 CLI를 제공한다. 예시는 `python mujoco_demo/demo_pick.py --viewer`와 `python mujoco_demo/demo_pick.py --episodes 10`이다. 같은 초기 상태에서 10회 반복해 성공 여부, 초기 높이, 유지 구간 최소 상승량, 유지 시간을 남긴다. 실행하지 않은 GUI나 팀원 PC 검증을 성공으로 기록하지 않는다.

## 팀 인계

- 루트 README에 Gazebo·MoveIt, Python MuJoCo 실행 절차를 각각 기록한다. 기존 ROS 패키지와 새 `mujoco_demo/`를 한 저장소로 관리한다. `build/`, `install/`, `log/`, `.venv/`와 생성된 캐시는 제외한다.
- Gazebo는 새 빌드 셸에서 `/opt/ros/jazzy/setup.bash`와 팀원 자신의 `ws_dsr/install/local_setup.bash`를 불러오고 루트에서 `colcon build --symlink-install`을 실행한다. 새 실행 셸마다 개인 `install/setup.bash`와 기존 패키지 README의 시뮬레이션 환경을 적용한다. RG2 데모 기준은 도메인 110, 로컬 탐색, `GZ_PARTITION=m0609_rg2_110`이다.
- `ros2 launch m0609_gz sim.launch.py rg2:=true gz_flags:=-v4`와 `ros2 launch m0609_gz moveit.launch.py rg2:=true`를 각각 기동한다. RViz에서 팔 궤적과 그리퍼 open/closed를 실행하고 Gazebo 동작을 확인한다.
- MuJoCo는 새 체크아웃과 새 가상환경에서 README 명령만으로 모델 준비와 파지 반복이 되는지 검사한다. 팀원 PC 테스트가 남았다면 그 상태를 명시한다.
