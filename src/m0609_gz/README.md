# M0609 Gazebo Harmonic

설치된 `dsr_description2`의 흰색 M0609 형상과 메시를 재사용한다. 개인 패키지에 추가한 파일은 다음과 같다.

- `urdf/m0609.urdf.xacro`는 베이스를 world에 고정하고 GazeboSimSystem 및 Harmonic 시스템 플러그인을 연결한다.
- `config/controllers.yaml`은 6축 위치 명령, 위치·속도 상태, 관절 상태 브로드캐스터와 궤적 컨트롤러를 설정한다.
- `launch/sim.launch.py`는 Gazebo와 /clock 브리지를 시작하고 스폰 성공 후 컨트롤러를 활성화한다.
- `CMakeLists.txt`와 `package.xml`은 자산 설치와 ROS 의존성을 정의한다.

## 재현

빌드 터미널에서 실행한다.

```bash
cd ~/m0609_moveit_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/local_setup.bash
colcon build --symlink-install
```

새 터미널에서 별도 ROS 도메인과 Gazebo 파티션으로 실행한다. 기본은 서버 모드다. 창이 필요하면 launch 명령에 `gz_flags:=-v4`를 추가한다.

```bash
cd ~/m0609_moveit_ws
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=109 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=m0609_sim_109
ros2 launch m0609_gz sim.launch.py
```

별도 검증 터미널에도 같은 `source`, `unset`, `export`를 적용한다.

```bash
ros2 control list_controllers
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
```

두 컨트롤러가 `active`인지 확인한다. 아래 예시는 초기 위치가 6축 모두 0 rad일 때의 명령이다. 초기 위치가 다르면 관절 이름 순서에 맞춰 측정값을 넣고 첫 축에만 0.05 rad를 더한다.

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}'
ros2 topic echo /joint_states --once
```

액션이 `SUCCEEDED` 및 `error_code: 0`을 반환하고, `joint_1`의 위치가 약 0.05 rad 변했는지 확인한다.

## 빌드 및 작업 셸 검증

- `colcon build --symlink-install` 성공. 패키지 1개 빌드 완료.
- Xacro 전개 및 `gz sdf -p` 성공. URDF에서 GazeboSimSystem, 시스템 플러그인 1개, 회전 관절 6개, 각 관절의 위치 명령 및 위치·속도 상태를 확인했다. 변환된 SDF에 world 고정 관절과 6축 관절, 시스템 플러그인이 유지됐다.
- 첫 launch에서 상속된 DDS 설정 파일을 열지 못했다. 실제 로그는 `log/m0609-launch.log`에 있다.
- DDS 설정을 해제한 재시도에서도 `failed to enumerate interfaces for "udp": -1` 및 `rmw_create_node: failed to create domain`으로 노드가 시작되지 않았다. 별도 소켓 생성 검사에서 UDP와 TCP 모두 `PermissionError(1, 'Operation not permitted')`를 반환했다. 같은 launch에서 Gazebo 로거의 segmentation fault도 기록됐다. 실제 로그는 `log/m0609-retry.log`에 있다.

## 로컬 실행 검증

사용자가 로컬 터미널에서 실행한 출력으로 M0609 단독 시뮬레이션의 완료 조건을 모두 확인했다.

- `joint_state_broadcaster`와 `joint_trajectory_controller` 모두 `active`.
- `/clock`에서 24.398초, `/joint_states`에서 6축 이름과 위치·속도 수신.
- `FollowJointTrajectory`의 2초 목표가 `SUCCEEDED` 및 `error_code: 0`을 반환했다.
- `joint_1`은 초기 약 0 rad에서 0.0499959567 rad로 이동했다. 명령값 0.05 rad와의 오차는 약 0.00000404 rad였으며, 나머지 5축은 0 rad 부근을 유지했다.
- 현재 제어 Xacro가 제공하는 상태 인터페이스는 position과 velocity다. effort는 `/joint_states`에서 `nan`으로 표시됐다.

## MoveIt 2 연동

워크스페이스를 다시 빌드한 뒤 위 Gazebo launch를 실행한다. 이미 같은 도메인에서 M0609가 실행 중이면 해당 시뮬레이션을 사용한다. 두 컨트롤러가 `active`인 상태에서 새 터미널을 열고 실행한다.

```bash
cd ~/m0609_moveit_ws
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=109 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch m0609_gz moveit.launch.py
```

1. RViz의 **MotionPlanning** 패널에서 그룹을 `manipulator`, 시작 상태를 **Current State**로 선택한다.
2. 목표 상태로 `all-zeros` 또는 `base-turn`을 고른다. 현재 1축이 0.50 rad 부근이면 `all-zeros`를 고르고, 0 rad 부근이면 `base-turn`을 고른다.
3. **Plan**으로 경로가 생성되는지 확인하고 **Execute**를 누른다. Gazebo에서 로봇이 움직이는지 보고, 같은 ROS 설정을 적용한 검증 터미널에서 `ros2 topic echo /joint_states --once`로 1축이 목표값에 도달하는지 확인한다.

`moveit.launch.py`는 Gazebo의 URDF를 그대로 사용하고 기존 Doosan 패키지의 KDL, OMPL, 관절 한계, RViz 설정을 읽는다. 기본 속도 및 가속도 스케일은 0.1이다. `m0609.srdf.xacro`는 로봇 이름을 URDF에 맞추고, 기본 모드에서 `base_link`부터 `link_6`까지의 6축 체인과 Doosan의 본체 충돌 제외 쌍, 검증용 자세 두 개를 정의한다. 베이스 고정은 URDF의 world 고정 관절로 구성한다. `moveit_controllers.yaml`은 기존 Gazebo 컨트롤러의 `/joint_trajectory_controller/follow_joint_trajectory`를 사용한다.

빌드와 두 launch의 인자 조회는 성공했다. 설치된 설정에서 6축 모델과 컨트롤러 매핑, OMPL 및 KDL 설정과 RViz MotionPlanning 표시를 확인했다. MoveIt 2.12.4의 PlanningScene으로 두 자세의 관절 한계 및 자체 충돌을 확인했고, 두 자세 사이 1축 회전 구간에서 추출한 21개 상태의 자체 충돌 검사도 통과했다. 검사 로그는 `log/m0609-moveit-model-check.log`에 있다.

작업 셸에서 `move_group` 실행 시 Cyclone DDS가 `failed to enumerate interfaces for "udp": -1` 및 `failed to create domain`을 반환했다. 실제 로그는 `log/m0609-moveit-launch.log`에 있다. 로컬 실행에서 사용자가 RViz의 `all-zeros` 목표를 실행했고 Gazebo의 M0609가 같은 자세로 움직이는 것을 확인했다.

### RViz에서 팔 끝 목표 지정

1. RViz에서 **Interact**를 선택하고 **Displays → MotionPlanning → Planning Request → Query Goal State**를 켠다.
2. `all-zeros`를 선택해 마커를 팔 끝에 맞추고 직선 화살표를 조금 드래그한다. 주황색 팔이 따라오면 목표 자세가 갱신된 것이다.
3. **Plan → Execute**를 누르고 Gazebo에서 움직임을 확인한다. 화살표만 움직이면 목표를 다시 맞추고 다른 방향으로 더 짧게 드래그한다.

사용자가 로컬에서 RViz 마커를 드래그한 뒤 팔이 움직이는 것을 확인했다. 저장 자세 `all-zeros`의 Gazebo 실행은 앞서 별도로 확인했다.

## RG2 장착 및 Gazebo 개폐

`sim.launch.py rg2:=true`는 기존 M0609에 RG2를 장착하고, 팔 컨트롤러와 함께 `rg2_controller`를 활성화한다. 기본 launch와 기존 MoveIt launch는 M0609 단독 모드를 사용한다.

- `urdf/m0609.urdf.xacro`는 설치된 `m0609_rg2_bringup`의 장착 자세와 RG2 형상·관절 매크로, `onrobot_rg_description`의 메시를 재사용한다. 고정 연결은 `link_6 → tool0 → rg2_base_link`다.
- 기존 통합 매크로가 추가하는 Gazebo Classic 제어 및 mimic 플러그인과 effort transmission은 전개에서 제외했다. Harmonic의 단일 `gz_ros2_control` 시스템에서 `rg2_finger_joint`만 위치 명령을 받고, URDF의 mimic 관절 5개는 위치·속도 상태를 제공한다. [gz_ros2_control Jazzy 문서](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html#using-mimic-joints-in-simulation)
- `config/controllers.yaml`은 RG2용 1축 궤적 컨트롤러를 추가한다. `launch/sim.launch.py`는 RG2 모드에서 스폰 성공 후 컨트롤러를 함께 기동하고 Gazebo의 메시 검색 경로를 추가한다. `package.xml`에 두 RG2 자산 패키지 의존성을 선언했다.

### 재현 및 확인

위의 빌드 명령을 실행한다. 새 터미널마다 다음 공통 환경을 적용한다. 기존 단독 시뮬레이션의 109 및 실기 프로젝트 기록의 50과 분리된 도메인이다.

```bash
cd ~/m0609_moveit_ws
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=110 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=m0609_rg2_110
```

첫 터미널에서 Gazebo 창을 포함해 실행한다. 서버만 필요하면 `gz_flags:=-v4`를 생략한다.

```bash
ros2 launch m0609_gz sim.launch.py rg2:=true gz_flags:=-v4
```

별도 터미널에서 컨트롤러 세 개가 `active`인지 확인하고, 아래 순서로 초기 위치, 열림, 닫힘을 기록한다.

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once

rg2_goal() {
  ros2 action send_goal /rg2_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: [rg2_finger_joint], points: [{positions: [$1], time_from_start: {sec: 2}}]}}"
}

rg2_goal -0.50
ros2 topic echo /joint_states --once
rg2_goal 0.70
ros2 topic echo /joint_states --once
```

두 액션에서 `SUCCEEDED`와 `error_code: 0`을 확인한다. 메시의 `name`과 `position`을 같은 인덱스로 대응시켜, 각 목표에서 아래 값에 수렴하는지 확인한다. 두 손가락이 손목의 양쪽에서 대칭으로 벌어졌다가 좁아지는지도 Gazebo 창에서 확인한다.

| 관절 | 열림 목표 (rad) | 닫힘 목표 (rad) |
| --- | ---: | ---: |
| `rg2_finger_joint` | -0.50 | +0.70 |
| `rg2_right_outer_knuckle_joint`, 좌우 `rg2_*_inner_knuckle_joint` | +0.50 | -0.70 |
| 좌우 `rg2_*_inner_finger_joint` | -0.50 | +0.70 |

같은 환경에서 `rviz2 --ros-args -p use_sim_time:=true`를 실행한다. **Fixed Frame**을 `world`로 설정하고 **RobotModel**을 추가한 뒤 **Description Source**를 **Topic**, **Description Topic**을 `/robot_description`으로 지정한다. Gazebo와 같은 모델이 로드되면 손목 끝 장착 위치와 손가락 개폐 방향을 비교한다.

### 작업 셸 검증 결과

- 빌드 및 두 launch의 인자 조회 성공. 팔 단독 Xacro는 기존 6축 구성을 유지했다.
- RG2 Xacro 전개, `gz sdf -p`, `gz sdf -k` 성공. 장착 연결, 메시 경로, Harmonic 시스템 플러그인 1개와 URDF/SDF의 mimic 관계 5개를 확인했다. URDF와 메시를 기준으로 계산한 손가락 간격은 열림 약 103 mm, 닫힘 약 8 mm다. 계산 결과는 `log/m0609-rg2-model-check.log`에 있다.
- 작업 셸 실행 시 Cyclone DDS가 `failed to enumerate interfaces for "udp": -1`, Gazebo가 `error in getifaddrs: Unknown error -1`을 반환했다. UDP 소켓 생성도 `Operation not permitted`로 실패했다. 실제 로그는 `log/m0609-rg2-launch.log`에 있다.

### 로컬 개폐 검증 결과

사용자가 Gazebo 창에서 RG2가 열리고 닫히는 것을 확인했다. `FollowJointTrajectory`는 두 목표 모두 `SUCCEEDED` 및 `error_code: 0`을 반환했다. 아래 수치는 각 액션 종료 직후의 `/joint_states`에서 읽었다.

| 관절 | 열림 측정 (rad) | 닫힘 측정 (rad) |
| --- | ---: | ---: |
| `rg2_finger_joint` | -0.49998938 | +0.69999044 |
| 좌우 `rg2_*_inner_finger_joint` | -0.49998813 | +0.69998931 |
| 좌우 `rg2_*_inner_knuckle_joint`, `rg2_right_outer_knuckle_joint` | +0.49998813 | -0.69998931 |

mimic 관절 5개는 두 자세 모두 지정 배율을 따랐다. 구동 관절 측정값에 배율을 적용한 값과의 최대 차이는 약 0.00000126 rad였다. 팔 6축은 0 rad 부근을 유지했다.

## MoveIt에서 RG2와 팔 끝 제어

RG2 모드의 팔 계획 그룹 `manipulator`는 `rg2_tcp`를 목표 링크로 사용한다. `rg2_tcp`는 RG2 베이스 +Z 방향으로 217 mm에 고정했다. 이 값은 설치된 손가락 메시에서 잡은 시뮬레이션용 기준이며 실제 장비의 TCP는 별도 보정 대상이다.

`m0609.srdf.xacro`는 RG2를 말단 장치로 등록하고 `gripper` 그룹과 `open`·`closed` 자세를 정의한다. 팔 플랜은 기존 6축 액션으로, 개폐 플랜은 기존 Gazebo의 `/rg2_controller/follow_joint_trajectory`로 전달된다. 개폐 궤적에는 구동 관절 `rg2_finger_joint`가 들어간다. 충돌 제외 쌍에 손목 장착부와 각 손가락 링크의 기구 접촉부를 추가했다.

### RViz 검증

위의 빌드 및 RG2 공통 환경을 적용한다. 이전 Gazebo와 MoveIt을 종료한 뒤 새 URDF가 로드되도록 Gazebo를 다시 스폰한다.

```bash
ros2 launch m0609_gz sim.launch.py rg2:=true gz_flags:=-v4
```

컨트롤러 세 개가 활성화된 후 같은 환경의 별도 터미널에서 실행한다.

```bash
ros2 launch m0609_gz moveit.launch.py rg2:=true
```

1. RViz의 **MotionPlanning**에서 계획 그룹을 `manipulator`, 시작 상태를 **Current State**로 선택한다. **Interact**와 **Query Goal State**를 켜고 손가락 끝 부근의 화살표를 조금 움직인다. **Plan → Execute** 후 같은 RG2 모델이 Gazebo에서 움직이는지 확인한다. TCP 프레임은 `ros2 run tf2_ros tf2_echo rg2_base_link rg2_tcp`로 확인할 수 있다.
2. 계획 그룹을 `gripper`로 바꾸고 시작 상태를 **Current State**로 선택한다. 목표 상태를 `open`과 `closed`로 바꿔 각각 **Plan → Execute**를 실행한다. Gazebo의 양쪽 손가락과 `/joint_states`를 확인한다. 목표는 각각 -0.50 rad와 +0.70 rad다.

### 검증 결과

- 빌드와 launch 인자 조회 성공. MoveIt의 PlanningScene에서 RG2 모드의 6축 TCP 체인, 그리퍼의 구동 관절 1개와 mimic 관절 5개, 저장 자세와 단일 관절 궤적 메시지를 확인했다. 1축 0~0.50 rad와 그리퍼 -0.50~+0.70 rad를 조합한 1,281개 상태의 자체 충돌 검사와 단독 모드의 회귀 검사를 통과했다. 로그는 `log/m0609-rg2-moveit-model-check.log`에 있다.
- 작업 셸에서 `move_group`은 Cyclone DDS의 UDP 초기화 오류로 시작되지 않았다. 로그는 `log/m0609-rg2-moveit-launch.log`에 있다.
- 로컬에서 사용자가 RViz의 **Plan → Execute**가 성공하고 Gazebo 모델이 움직이는 것을 확인했다. `tf2_echo`는 시작 직후 프레임을 기다린 뒤 `rg2_base_link → rg2_tcp` 변환을 받았다. 평행 이동은 `[0, 0, 0.217]` m이고 회전은 0이다.
- 닫힘 후 `/joint_states`에서 구동 관절은 +0.69991981 rad였다. 좌우 안쪽 손가락 관절도 +0.69991981 rad였고, 좌우 안쪽 너클과 오른쪽 바깥쪽 너클은 -0.69991981 rad로 mimic 설정과 일치했다.
