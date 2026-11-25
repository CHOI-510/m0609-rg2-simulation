# 구현 상세

처음 실행할 때는 [루트 README](../README.md)를 참고하세요. 실제 수치와 제약은 [검증 기록](validation.md)에 있습니다.

## 모델 출처와 좌표계

- M0609 MJCF와 STL은 두산 공식 저장소 커밋 `243af5d1a071f030818bbcd6e9e5ee23d9edacae`의 `dsr_description2/mujoco_models/m0609/`에서 가져왔습니다. `m0609.xml`의 메시 경로는 `assets/` 기준으로 정규화했고 STL은 원본 그대로 보관합니다. [공식 모델](https://github.com/DoosanRobotics/doosan-robot2/tree/243af5d1a071f030818bbcd6e9e5ee23d9edacae/dsr_description2/mujoco_models/m0609) · [Apache 2.0 고지](../mujoco_demo/models/m0609/LICENSE)
- Gazebo용 URDF는 `dsr_description2`와 `m0609_rg2_bringup`의 Xacro 및 `onrobot_rg_description`의 메시를 사용합니다. MuJoCo용 RG2 시각 메시도 underlay의 `onrobot_rg_description/meshes/rg2/visual`에서 가져왔습니다. [로봇 Xacro](../src/m0609_gz/urdf/m0609.urdf.xacro) · [RG2 MJCF](../mujoco_demo/models/rg2.xml) · [MIT 고지](../mujoco_demo/models/rg2/LICENSE)

두 모델 모두 월드 원점에 로봇 베이스를 고정합니다. URDF의 `link_6 → tool0 → rg2_base_link` 장착 변환을 MJCF에 옮겼고, TCP는 RG2 베이스의 +Z 방향 217 mm입니다. Gazebo에서는 `rg2_tcp` TF 프레임, MuJoCo에서는 같은 이름의 site입니다. Gazebo의 `gz_ros2_control`은 구동 관절 `rg2_finger_joint`에 위치 명령을 보내고 나머지 다섯 관절을 mimic으로 연동합니다. MJCF는 같은 배율과 오프셋을 관절 등식 제약에 사용합니다.

## 장면과 접촉

| 항목 | 공통 값과 엔진별 차이 |
| --- | --- |
| 받침대 | 중심 `[0.47, 0, 0.15]` m, 전체 크기 `[0.28, 0.24, 0.30]` m, 고정 |
| 상자 | 중심 `[0.47, 0, 0.32]` m에서 시작, 한 변 40 mm, 질량 64 g, 대각 관성 약 1.707×10⁻⁵ kg·m², 자유 물체 |
| 패드 | 좌우 안쪽 손가락 로컬 좌표 기준 중심 `[0, 0.016, 0.042]` m, 전체 크기 `[0.018, 0.006, 0.020]` m |
| 마찰 | 패드 미끄럼 계수 1.2, 받침대 1.0. Gazebo 상자는 `mu/mu2=1.2`. MuJoCo 상자는 기본값 1.0이며 패드와 닿을 때 더 큰 값인 1.2를 사용. MuJoCo 패드의 비틀림 마찰은 0.005 |
| 물리·제어 | Gazebo DART 1 ms, `ros2_control` 100 Hz 위치 명령. MuJoCo implicitfast 2 ms, 팔 토크 PD와 힘이 제한된 그리퍼 위치 서보 |

`pick:=true`에서 Gazebo와 MoveIt은 RG2의 시각 메시를 유지하면서 파지 충돌 형상을 양쪽 박스 패드로 바꿉니다. MuJoCo 장면도 같은 패드 크기를 사용합니다. SDF의 박스 크기는 전체 길이이고 MJCF의 `size`는 반길이입니다. 장면 변경 시 MoveIt의 충돌체 위치와 크기까지 맞추세요. [Gazebo 월드](../src/m0609_gz/worlds/pick.sdf) · [파지 URDF 설정](../src/m0609_gz/config/pick_robot.py) · [MuJoCo 장면](../mujoco_demo/model.py)

`record_pick`은 받침대와 상자를 Planning Scene의 월드 충돌체로 등록합니다. 접근에는 상자 충돌 검사를 적용하고, 닫기부터 상자와 두 안쪽 손가락 링크 사이의 계획 충돌만 허용합니다. 받침대 검사는 유지합니다. 물리 시뮬레이터에서 상자는 끝까지 자유 상태이며, 이동과 회전은 접촉력과 중력으로 계산합니다. [계획 및 기록 코드](../src/m0609_gz/scripts/record_pick)

## 궤적 포맷과 재생 제어

`record_pick`은 준비 자세로 이동한 뒤 Gazebo에서 관절 시작 자세를 한 번 측정합니다. MoveIt의 `/plan_kinematic_path`에서 계획하고, 팔과 그리퍼의 `FollowJointTrajectory` 액션으로 전송합니다. 상자가 손가락 닫힘을 제한할 수 있으므로 닫기 액션에서만 자유공간 목표 각도의 도달 허용 오차를 해제합니다.

| `trajectory.json` 필드 | 의미 |
| --- | --- |
| `schema`, `frame` | `m0609.pick.v1`, `world` |
| `initial_positions_rad` | 재생 원점에서 측정한 초기 관절 각도. 관절 이름이 키 |
| `phases` | 순서대로 `approach`, `close`, `lift` |
| `joint_names`, `gazebo_start_s`, `dwell_s` | 단계별 관절 순서, 재생 원점 기준 시작 시각, 단계 뒤 대기 설정 |
| `points` | 단계 시작 기준 `time_s`와 관절 순서대로 나열한 `positions_rad`, `velocities_rad_s`, `accelerations_rad_s2` |

기록이 중단되면 컨트롤러가 수락한 단계까지만 JSON에 남습니다. 재생에는 세 단계가 모두 필요합니다. Gazebo의 CLI 종료 코드 0은 궤적 실행 완료를 뜻합니다. 파지 여부는 측정 보고서의 `grasp_success`에서 확인합니다.

MuJoCo는 관절 이름을 매핑하고, 위치만 있으면 선형, 속도까지 있으면 3차, 가속도까지 있으면 5차 보간을 사용합니다. 보간 목표와 측정 관절값으로 PD 토크를 계산하고 바이어스 토크를 더해 공식 팔 모터에 입력합니다. 그리퍼는 힘 제한이 있는 위치 서보로 목표 각도를 추종합니다. 반복 실행마다 관절과 제어 입력을 초기화하고 자유 상자를 0.5초간 정착시킵니다. 독립 데모는 코드에 정의된 관절 자세를 사용합니다. 비교 도구는 두 보고서의 명령 파일 SHA-256과 관절 순서가 일치하는지 확인합니다. [재생 제어](../mujoco_demo/replay.py) · [독립 데모](../mujoco_demo/demo_pick.py) · [비교 도구](../mujoco_demo/compare_pick.py)

## 파지 성공 판정

- 공통 기준은 상승 후 상자 중심이 정착 높이보다 최소 50 mm 높고, 양쪽 패드의 접촉으로 1초 이상 유지되는 것입니다.
- Gazebo는 상자 자세 샘플이 1초 이상을 덮고 샘플 최대 간격이 50 ms 이내인지 검사합니다. 유지 샘플에서 양쪽 패드 접촉이 확인되고 상자에 받침대나 바닥 접촉이 없어야 합니다.
- MuJoCo는 유지 구간의 모든 2 ms 스텝에서 양쪽 패드에 양의 수직 접촉력이 있고 다른 지지 접촉이 없는지 검사합니다. 반복 재생 CLI는 파지 또는 리셋 일치 검사에 실패하면 0이 아닌 종료 코드를 반환합니다.

## 처음부터 셋업하기

1. **Python**. ROS 설정을 불러오지 않은 터미널의 저장소 루트에서 `python3 -m venv .venv`, `source .venv/bin/activate`, `python -m pip install -r mujoco_demo/requirements.txt`, `python -m pip check`를 실행합니다. Gazebo용 underlay는 필요 없습니다.
2. **ROS**. Ubuntu 24.04의 ROS 2 Jazzy 환경에 Gazebo Harmonic, MoveIt 2, `gz_ros2_control`을 준비합니다. 팀의 빌드된 `ws_dsr`에는 `dsr_description2`, `dsr_moveit_config_m0609`, `m0609_rg2_bringup`, `onrobot_rg_description`이 필요합니다. 나머지 실행 의존성은 패키지 매니페스트에 있습니다. 팀원의 설치 경로로 `WS_DSR`을 설정하고 ROS와 underlay를 순서대로 source한 뒤 이 저장소에서만 `colcon build --symlink-install`을 실행합니다. [패키지 매니페스트](../src/m0609_gz/package.xml)
3. **Gazebo 및 MoveIt**. [README의 세 터미널 절차](../README.md#2-gazebo와-moveit을-실행하고-기록)를 따릅니다. `ros2 control list_controllers`에서 `joint_state_broadcaster`, `joint_trajectory_controller`, `rg2_controller`가 모두 활성 상태인지 확인합니다. `ros2 topic echo /clock --once`로 시뮬레이션 시각도 확인할 수 있습니다.
4. **기록, 재생, 비교**. 새 출력 디렉터리에 `record_pick` 결과를 저장한 뒤 [README의 재생 명령](../README.md#3-mujoco에서-재생하고-비교)을 실행합니다. 측정 보고서와 명령 파일을 같은 실행에서 생성한 것으로 사용합니다.

빌드 셸에서 실행할 명령입니다.

```bash
source /opt/ros/jazzy/setup.bash
WS_DSR="$HOME/ws_cobot_pjt/ws_dsr"
source "$WS_DSR/install/local_setup.bash"
ros2 pkg prefix dsr_description2
ros2 pkg prefix dsr_moveit_config_m0609
ros2 pkg prefix m0609_rg2_bringup
ros2 pkg prefix onrobot_rg_description
colcon build --symlink-install
```

Gazebo·MoveIt·기록 터미널에 공통으로 적용할 환경입니다. `install/setup.bash`는 이 저장소를 빌드한 뒤 생성됩니다.

```bash
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=112 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=m0609_pick_112
```

독립 MuJoCo 모드에서는 아래 검사와 GUI를 사용할 수 있습니다. GUI에는 OpenGL을 지원하는 데스크톱 세션이 필요합니다. CLI에서 `--viewer`와 `--trajectory`는 별도 모드입니다.

```bash
python mujoco_demo/demo_pick.py --check-arm
python mujoco_demo/demo_pick.py --check-gripper
python mujoco_demo/demo_pick.py --episodes 10
python mujoco_demo/demo_pick.py --viewer
```

Gazebo의 RViz 계획 그룹, 목표 자세, TF 확인과 컨트롤러 직접 검증은 [Gazebo 패키지 가이드](../src/m0609_gz/README.md)에 있습니다.
