# M0609 + RG2 시뮬레이션

저장소에는 MoveIt 2로 Gazebo의 M0609와 RG2를 제어하는 데모와 Python MuJoCo에서 40 mm 정육면체를 파지하는 데모가 있다.

## MoveIt 2 → Gazebo Harmonic

Ubuntu 24.04와 ROS 2 Jazzy 환경에서 Gazebo Harmonic, MoveIt 2, gz_ros2_control을 사용한다. 기존 ROS underlay에는 `dsr_description2`, `dsr_moveit_config_m0609`, `m0609_rg2_bringup`, `onrobot_rg_description`이 설치되어 있어야 한다. 빌드 셸에서 `WS_DSR`을 각자 설치한 underlay 경로로 지정한다.

```bash
source /opt/ros/jazzy/setup.bash
WS_DSR="$HOME/ws_cobot_pjt/ws_dsr"
source "$WS_DSR/install/local_setup.bash"
colcon build --symlink-install
```

저장소 루트에서 새 터미널을 열 때마다 아래 환경을 적용한다. ROS 탐색은 로컬 호스트로 제한하고 Gazebo는 별도 파티션을 사용한다.

```bash
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=110 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=m0609_rg2_110
```

첫 터미널에서 Gazebo를 시작한다.

```bash
ros2 launch m0609_gz sim.launch.py rg2:=true gz_flags:=-v4
```

두 번째 터미널에서 컨트롤러와 시계를 확인하고 MoveIt과 RViz를 시작한다.

```bash
ros2 control list_controllers
ros2 topic echo /clock --once
ros2 launch m0609_gz moveit.launch.py rg2:=true
```

1. `joint_state_broadcaster`, `joint_trajectory_controller`, `rg2_controller`가 모두 `active`인지 확인한다.
2. RViz의 **MotionPlanning**에서 시작 상태를 **Current State**, 계획 그룹을 `manipulator`로 선택한다. 초기 자세가 0 rad 부근이면 `base-turn`을 골라 **Plan → Execute**를 실행한다. Gazebo에서 팔이 회전하고 `joint_1`이 0.50 rad 부근에 도달하는지 확인한다.
3. 계획 그룹을 `gripper`로 바꾸고 `open`, `closed`를 차례로 **Plan → Execute**한다. Gazebo에서 손가락이 벌어졌다가 좁아지는지 확인한다. 별도 터미널에서 `ros2 topic echo /joint_states --once`를 실행하면 각 목표의 구동 관절 각도인 −0.50 rad, +0.70 rad를 확인할 수 있다.

팔 끝 마커, TF 확인과 컨트롤러 직접 검증 절차는 [Gazebo 패키지 README](src/m0609_gz/README.md)에 있다.

## Python → MuJoCo

저장소 루트에서 ROS 설정을 불러오지 않은 터미널을 사용한다. 아래 명령은 Ubuntu 24.04 x86-64와 Python 3.12에서 검증했다. GUI에는 OpenGL을 지원하는 데스크톱 세션이 필요하다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r mujoco_demo/requirements.txt
python -m pip check
python mujoco_demo/demo_pick.py --check-arm
python mujoco_demo/demo_pick.py --check-gripper
python mujoco_demo/demo_pick.py --episodes 10
python mujoco_demo/demo_pick.py --viewer
```

모델과 메시는 저장소에 포함되어 있어 실행 시 다운로드나 ROS underlay가 필요 없다.

- 팔은 두산 공식 커밋 `243af5d1a071f030818bbcd6e9e5ee23d9edacae`의 `dsr_description2/mujoco_models/m0609/`에서 가져왔다. `m0609.xml`과 `scene.xml`의 메시 경로를 모델 디렉터리 기준으로 정규화했다. STL 파일은 원본 그대로이며 Apache 2.0 고지를 함께 둔다. [공식 모델](https://github.com/DoosanRobotics/doosan-robot2/tree/243af5d1a071f030818bbcd6e9e5ee23d9edacae/dsr_description2/mujoco_models/m0609) · [라이선스](mujoco_demo/models/m0609/LICENSE)
- RG2 시각 메시는 기존 underlay의 `onrobot_rg_description/meshes/rg2/visual` 자산을 복사했으며 MIT 고지를 함께 둔다. 장착 및 관절 프레임은 `m0609.urdf.xacro rg2:=true`를 전개한 URDF에서 옮겼다. `rg2_tcp`는 RG2 베이스의 +Z 217 mm에 있고, MJCF 관절 제약 5개가 Xacro의 mimic 배율과 오프셋을 따른다. 양쪽 안쪽 손가락의 접촉면은 마찰이 있는 박스 패드로 근사했다. [RG2 모델](mujoco_demo/models/rg2.xml) · [라이선스](mujoco_demo/models/rg2/LICENSE)

파지 데모는 공식 모터에 위치 오차와 속도, 바이어스 토크를 사용한 제어 입력을 보낸다. 사전 계산한 관절 자세 사이를 보간해 접근과 상승을 수행한다. 상자는 고정 받침대 위에서 자유 관절로 시작하며 매 에피소드의 MuJoCo 초기화 이후 접촉과 중력으로 움직인다. 성공하려면 초기화 상태가 일치하고, 정착한 상자의 중심이 상승 후 1초 내내 시작 높이보다 50 mm 이상 높아야 한다. 이 구간의 각 시뮬레이션 스텝에서 양쪽 패드에 양의 수직 접촉력이 있고 다른 물체의 지지가 없는지도 확인한다. 실패하면 CLI가 0이 아닌 종료 코드를 반환한다.

## 검증 기록

### MuJoCo

Python 3.12.3, MuJoCo 3.17.0, NumPy 2.5.5를 사용했다. 나머지 Python 의존성 버전은 [requirements.txt](mujoco_demo/requirements.txt)에 고정했다.

- `--check-arm`에서 관절 6개와 모터 6개를 로드했다. 첫 모터에 30 N·m를 2초 동안 적용했을 때 1축이 0.144928 rad 움직였고, 초기화 후 시각·관절 위치·속도·제어 입력이 0으로 복원됐다.
- `--check-gripper`에서 열림 −0.497833 rad, 닫힘 +0.700232 rad를 측정했다. 패드 간격은 각각 102.39 mm와 7.71 mm였고 mimic 각도 오차는 최대 0.000002 rad였다.
- Gazebo용 전개 URDF의 순기구학과 MJCF의 TCP 위치·방향을 6축 영점 및 접근·파지 자세에서 비교했다. 최대 차이는 위치 0.000374 mm, 회전 0.000028°였다.
- `--episodes 10`에서 10회 모두 초기 상태가 일치했고, 상자 시작 높이 319.892 mm, 유지 구간 최소 상승량 83.091 mm, 양쪽 패드로 유지한 시간 1.000초를 기록했다.
- `--viewer`로 같은 시퀀스를 재생했다. 1회 성공 후 창이 닫혔고 종료 코드는 0이었다.
- 작업 디렉터리의 Git 메타데이터를 사용할 수 없어 소스와 자산만 임시 로컬 저장소에 담고 별도 경로에 복제했다. 그 체크아웃의 빈 가상환경에서 위 설치 명령과 `pip check`, 팔·그리퍼 검사, 헤드리스 10회를 확인했다. 팀원 PC에서의 실행은 아직 확인하지 않았다.

### Gazebo와 MoveIt

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, MoveIt 2.12.4, gz_ros2_control 1.2.20에서 확인했다.

- 위 underlay 설정으로 별도 빌드·설치 경로에 `colcon build --symlink-install`을 실행해 1개 패키지를 빌드했다. 두 launch의 `--show-args`, 단독 및 RG2 Xacro 전개, RG2 모델의 `gz sdf -p`와 `gz sdf -k`가 성공했다.
- 위 GUI launch 두 개를 로컬 도메인 110에서 실행해 Gazebo 스폰과 RViz의 MoveIt 초기화를 확인했다. 이어서 동일 도메인의 단일 서버 세션에서 `sim.launch.py rg2:=true`와 `moveit.launch.py rg2:=true rviz:=false`로 동작을 재검증했다. `ros2 control list_controllers`에서 세 컨트롤러가 모두 `active`였고, `ros2 topic echo /clock --once`에서 0.706초를 받았다.
- 동작 검증에는 RViz 버튼 대신 MoveIt의 `/move_action` 액션에 계획 및 실행 목표를 보냈다. 1축 0.50 rad 및 RG2 −0.50/+0.70 rad 목표가 모두 `SUCCEEDED`와 MoveIt 오류 코드 `SUCCESS (1)`로 끝났다. `/joint_states`에서 순서대로 0.499489, −0.498089, +0.698110 rad를 측정했다.
- 제한된 작업 셸에서는 DDS가 `failed to enumerate interfaces for "udp": -1`을, GLFW가 `X11: Failed to open display :0`을 반환했다. UDP와 디스플레이 접근을 허용한 로컬 세션에서 위 검증을 완료했다.

## MoveIt → Gazebo → MuJoCo 궤적 재생

같은 MoveIt 계획을 Gazebo에서 실행하고 MuJoCo에서 재생한다. 기존 두 데모의 실행 방법은 위와 같다. 새 파지 장면은 `pick:=true`로 선택한다.

### Gazebo에서 생성 및 검증

위 ROS 빌드 절차를 실행한다. 아래 환경을 실행 터미널 세 곳에 각각 적용한다.

```bash
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=112 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=m0609_pick_112
```

터미널별로 아래 명령을 한 줄씩 실행한다. 새 시도마다 Gazebo를 다시 시작해 상자와 로봇의 초기 상태를 복원하고 빈 출력 디렉터리를 지정한다.

```bash
ros2 launch m0609_gz sim.launch.py rg2:=true pick:=true
ros2 launch m0609_gz moveit.launch.py rg2:=true pick:=true rviz:=false
ros2 run m0609_gz record_pick log/pick-run --ros-args -p use_sim_time:=true
```

`record_pick`은 MoveIt의 `/plan_kinematic_path`에서 계획을 받아 Gazebo의 두 `FollowJointTrajectory` 액션에 보낸다. `trajectory.json`에는 재생 원점의 관절 자세와 각 컨트롤러가 수락한 계획의 단계, 시작 시각, 관절 이름, 목표 위치·속도·가속도, 상대 시각을 기록한다. 재생 원점의 관절 자세는 준비 이동 후 Gazebo에서 한 번 측정해 고정한다. `gazebo.json`에는 별도로 관절·TF 기반 TCP·상자 자세, 상자 접촉, 종료 단계, 궤적 실행 완료 여부와 파지 판정을 기록한다. 닫기 액션에서만 자유공간 최종 각도 허용 오차를 해제해, 상자가 손가락을 막은 상태에서도 상승을 시험할 수 있다. 실행이 중단되면 수락된 단계까지만 저장한다. CLI 종료 코드 0은 세 궤적의 실행 완료를 뜻하며 파지 판정은 `grasp_success`에서 확인한다.

### 장면 대응

| 항목 | Gazebo와 MuJoCo |
| --- | --- |
| 좌표 | 월드와 고정 로봇 베이스 원점 일치. TCP는 RG2 베이스의 +Z 217 mm |
| 받침대 | 중심 `[0.47, 0, 0.15]` m, 크기 `[0.28, 0.24, 0.30]` m, 고정 |
| 상자 | 중심 `[0.47, 0, 0.32]` m에서 시작, 한 변 40 mm, 64 g, 대각 관성 1.707×10⁻⁵ kg·m², 자유 물체 |
| 패드 | 양쪽 안쪽 손가락에 `[0, 0.016, 0.042]` m 오프셋과 `[0.018, 0.006, 0.020]` m 크기의 박스 |
| 마찰 | 패드 미끄럼 계수 1.2, 받침대 1.0. Gazebo는 상자 `mu/mu2=1.2`이고 MuJoCo는 상자 기본 1.0과 패드 1.2 중 큰 값을 접촉에 사용하며 패드 비틀림 마찰 0.005를 사용 |
| 물리·제어 | Gazebo DART 1 ms, ros2_control 100 Hz 위치 명령 및 소프트웨어 mimic. MuJoCo implicitfast 2 ms, 토크 PD·그리퍼 힘 제한 위치 서보 및 관절 등식 제약 |

MoveIt Planning Scene에는 받침대와 상자를 월드 충돌체로 등록한다. 접근 전에는 상자와 로봇 간의 충돌을 검사하고, 닫기부터는 상자와 두 패드 링크 사이만 허용한다. 받침대와의 충돌 검사는 계속 적용된다. 상자는 양쪽 물리 엔진에서 자유 물체로 남는다.

### 기록 재생과 파지 비교

위 MuJoCo 가상환경을 활성화한 뒤 저장소 루트에서 실행한다.

```bash
python mujoco_demo/demo_pick.py --trajectory log/pick-run/trajectory.json \
  --episodes 10 --report log/pick-run/mujoco.json
python mujoco_demo/compare_pick.py log/pick-run/trajectory.json \
  log/pick-run/gazebo.json log/pick-run/mujoco.json > log/pick-run/compare.json
python mujoco_demo/pick_env.py log/pick-run/trajectory.json
```

MuJoCo는 관절 이름으로 매핑하고 명령에 포함된 미분값에 따라 선형·3차·5차 보간을 적용한다. 매 에피소드에서 자유 상자를 초기화하고 0.5초간 정착시킨다. 상승 후 1초 동안 상자 높이가 기준보다 50 mm 이상 높고, 양쪽 패드만 상자를 지지해야 통과한다. 비교 도구는 각 측정 보고서가 같은 명령 파일의 SHA-256을 가리키는지 확인한다.

`PickEnv.reset()`은 접근 끝까지 실행하고 관절 위치·속도, 그리퍼 위치, TCP와 상자 위치를 관측값으로 반환한다. `step([action])`에서 `action`은 −1~1의 실수 하나이며, 기록된 단계 간격이 허용하는 범위 안에서 닫기 시작을 최대 ±0.15초 바꾼다. 상승 시작 시각은 고정이다. 한 번의 결정으로 에피소드가 종료되며, 반환 형식은 `(observation, reward, terminated, truncated, info)`다. 성공 보상은 1, 실패는 0이다.

저장소 루트에서 학습 루프에 다음과 같이 연결할 수 있다.

```python
from mujoco_demo.pick_env import PickEnv

env = PickEnv("mujoco_demo/trajectories/moveit_pick.json")
observation, info = env.reset()
observation, reward, terminated, truncated, info = env.step([0.0])
```

### 파이프라인 검증 결과

Ubuntu 24.04에서 ROS 2 Jazzy, MoveIt 2.12.4, Gazebo Harmonic 8.15.0, gz_ros2_control 1.2.20과 Python 3.12.3, MuJoCo 3.17.0, NumPy 2.5.5를 사용했다. 최종 검증은 로컬 도메인 113 및 `GZ_PARTITION=m0609_pick_113`에서 실행했다. 실행한 명령 데이터는 [moveit_pick.json](mujoco_demo/trajectories/moveit_pick.json)에 포함했다. Gazebo를 다시 시작하지 않고도 위 재생 및 환경 명령에 이 파일을 지정해 검증된 기준 동작을 실행할 수 있다. 이 작업 공간의 `log/pick-validated/`에는 각 시뮬레이터의 원시 측정값, 비교 보고서, 학습 환경 스모크 테스트 결과가 있다.

| 측정 | Gazebo | MuJoCo |
| --- | ---: | ---: |
| 파지 성공 | 1/1 | 10/10 |
| 시작 상자 높이 (mm) | 319.999 | 319.892 |
| 유지 구간 최소 상승량 (mm) | 84.068 | 83.621 |
| 최대 상승량 (mm) | 84.940 | 84.342 |
| 양쪽 패드 접촉이 확인된 유지 구간 (s) | 1.070 | 1.000 |
| 팔 목표 추종 RMSE / 최대 절대 오차 (rad) | 0.01129 / 0.06449 | 0.00431 / 0.02615 |
| 구동 손가락 목표 추종 RMSE / 최대 절대 오차 (rad) | 0.19198 / 0.30270 | 0.14113 / 0.22251 |
| 유지 끝 구동 손가락 각도 (rad) | 0.39725 | 0.47843 |

- 재생 시작 시 양쪽 팔 관절의 최대 차이는 0.00000003 rad였다. MuJoCo 정착 후 손가락은 Gazebo 시작값보다 0.00213 rad 더 닫혔다. 시작·접근 끝·닫기 끝·상승 끝·유지 끝의 TCP 위치 차이는 각각 0.007, 1.599, 0.123, 1.380, 0.114 mm였다. 단계별로 명령 종료 시각에 가장 가까운 측정값을 사용했다. Gazebo는 872개, MuJoCo는 첫 에피소드의 934개 관절 샘플에서 추종 오차를 계산했다.
- Gazebo는 상자 자세 샘플 간 최대 간격 20 ms, 양쪽 패드 접촉 센서 및 받침대 이탈로 유지 여부를 확인했다. MuJoCo는 유지 구간의 모든 2 ms 스텝에서 양쪽 패드에 양의 수직력이 있는지 확인했다. 손가락 추종 오차는 상자가 닫힘을 막은 동안에도 자유공간 목표 +0.70 rad를 기준으로 계산했다.
- 초기 Gazebo 시도에서는 원래 RG2 너클 메시가 상자와 충돌해 MoveIt의 닫기 계획이 실패했다. 패드 형상을 맞춘 다음 시도에서는 상자가 손가락을 약 +0.397 rad에서 막아 기존 0.01 rad 도달 허용 오차로 닫기 액션이 중단됐다. 닫기 액션의 위치 도달 허용 오차만 해제한 뒤 상승에 성공했다. 별도 중간 실행에서는 상자가 83.992 mm 이상 올라갔지만 접촉 토픽이 연결되지 않아 양쪽 접촉을 판정하지 못했다. Harmonic에 맞게 `<contact><topic>`으로 설정한 최종 실행에서 두 패드 접촉을 확인했다. [Gazebo Harmonic Contact 구현](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc)
- `PickEnv`의 행동 0은 직접 재생과 최종 `qpos`, `qvel`, 판정 및 상승량이 정확히 일치했다. 다시 초기화했을 때 같은 관측값을 반환했고, 행동 +1로 닫기를 0.150초 늦춘 시도도 종료 및 성공했다. 기존 독립 MuJoCo 데모도 헤드리스 10회 재검증했다.
- `pick` 옵션을 생략한 기존 Gazebo·MoveIt 경로도 도메인 114에서 재검증했다. 세 컨트롤러가 활성화됐고, 팔 1축 및 RG2 열림·닫힘 MoveIt 액션이 모두 성공했다. 측정값은 각각 0.498073, −0.498233, +0.698061 rad였다. 새 월드의 SDF 검증과 두 launch의 인자 조회도 통과했다.
- 기본 샌드박스에서 `failed to enumerate interfaces for "udp": -1`이 발생해 DDS 검증은 허용된 로컬 셸에서 진행했다. 새 파이프라인은 헤드리스에서 검증했으며 이번에는 GUI를 통해 재실행하지 않았다.
