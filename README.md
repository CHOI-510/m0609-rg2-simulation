# M0609 + RG2 시뮬레이션

두산 M0609 로봇과 RG2 그리퍼로 40 mm 상자를 들어 올리는 시뮬레이션 프로젝트입니다.

- MoveIt/RViz에서 로봇, 파지 장면, 계획 궤적을 확인할 수 있습니다.
- MoveIt에서 계획한 **접근 → 닫기 → 상승**을 Gazebo에서 실행하고, 상자의 상승량과 손가락 접촉으로 파지를 검증할 수 있습니다.
- Gazebo에서 실행한 관절 궤적을 JSON으로 저장해 MuJoCo에서 재생할 수 있습니다. 저장소에 포함된 기준 궤적으로 반복 파지와 `PickEnv`도 바로 실행할 수 있습니다.

현재 검증 범위는 **상자를 잡아 들어 올린 뒤 유지하는 단계까지**입니다. 내려놓기, 실기 연동, 정책 학습은 확장 항목입니다.

## 가장 빠른 실행 · 기준 궤적 재생

클론한 저장소 루트에서 실행하세요. Ubuntu 24.04 x86-64와 Python 3.12에서 검증했습니다. Python의 `venv` 모듈과 [Python 의존성](mujoco_demo/requirements.txt)이 필요합니다. 모델과 기준 궤적은 저장소에 포함되어 있으므로 이 경로는 **ROS, Gazebo, MoveIt, GUI 없이** 실행할 수 있습니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r mujoco_demo/requirements.txt
python mujoco_demo/demo_pick.py --trajectory mujoco_demo/trajectories/moveit_pick.json --episodes 10
python mujoco_demo/pick_env.py mujoco_demo/trajectories/moveit_pick.json
```

성공하면 재생 명령은 회차별 JSON에 `"reset_matches": true`, `"grasp_success": true`를 출력합니다. `PickEnv` 검사는 행동값 0에서 직접 재생과 일치하면 `"baseline_matches": true`, 에피소드가 끝나면 `"terminated": true`를 출력합니다.

기준 궤적의 이전 검증에서 Gazebo는 **1/1회, 최소 84.068 mm**, MuJoCo는 **10/10회, 최소 83.621 mm** 상승을 기록했습니다. 두 엔진에서 각각 양쪽 패드로 상자를 1초 이상 유지했습니다. [상세 검증 기록](docs/validation.md)

## 내 동작을 새로 만들기

### 1. ROS 환경을 준비하고 빌드

ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, `gz_ros2_control`과 빌드된 `ws_dsr` underlay가 필요합니다. `WS_DSR`을 팀원이 설치한 경로로 바꾸세요. 빌드와 실행에 필요한 패키지 목록은 [구현 상세](docs/implementation.md#처음부터-셋업하기)에 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
WS_DSR="$HOME/ws_cobot_pjt/ws_dsr"
source "$WS_DSR/install/local_setup.bash"
colcon build --symlink-install
```

### 2. Gazebo와 MoveIt을 실행하고 기록

저장소 루트에서 새 터미널을 세 개 열고, **각 터미널**에 같은 환경을 적용하세요. ROS 도메인과 Gazebo 파티션은 로컬 시뮬레이션에 사용합니다.

```bash
source install/setup.bash
unset CYCLONEDDS_URI ROS_STATIC_PEERS
export ROS_DOMAIN_ID=112 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=m0609_pick_112
```

**터미널 1 · Gazebo**

```bash
ros2 launch m0609_gz sim.launch.py rg2:=true pick:=true
```

**터미널 2 · MoveIt**

```bash
ros2 launch m0609_gz moveit.launch.py rg2:=true pick:=true rviz:=false
```

**터미널 3 · 기록**. `ros2 control list_controllers`에서 세 컨트롤러가 `active`인 것을 확인한 뒤 실행하세요.

```bash
ros2 run m0609_gz record_pick log/pick-run --ros-args -p use_sim_time:=true
```

화면에서 확인하려면 데스크톱 세션에서 터미널 1 명령에 `gz_flags:=-v4`를 추가하고, 터미널 2의 `rviz:=false`를 `rviz:=true`로 바꾸세요. RViz에서 팔과 그리퍼를 직접 계획·실행하는 기존 절차는 [Gazebo 패키지 가이드](src/m0609_gz/README.md)에 있습니다. 새 기록을 생성할 때는 Gazebo를 다시 시작해 상자와 로봇을 초기 상태로 돌리고, 새 출력 디렉터리를 지정하세요.

| 생성 파일 | 용도 |
| --- | --- |
| `log/pick-run/trajectory.json` | Gazebo 컨트롤러가 수락한 관절 명령. 재생 시작 자세, 단계 구분, 관절 이름, 시간, 목표 관절값을 담습니다. |
| `log/pick-run/gazebo.json` | Gazebo 실측값과 실행 상태. 관절, TCP, 상자 자세, 접촉, `trajectory_complete` 및 `grasp_success`가 기록됩니다. |
| `log/pick-run/mujoco.json` | 다음 단계에서 생성되는 MuJoCo 회차별 결과와 첫 회차 측정값입니다. |
| `log/pick-run/compare.json` | 같은 명령 파일로 실행했는지 확인하고 양쪽 측정값을 비교한 결과입니다. |

### 3. MuJoCo에서 재생하고 비교

ROS 설정을 불러오지 않은 새 터미널에서 앞서 만든 Python 가상환경을 활성화하세요. `gazebo.json`에서 `trajectory_complete`가 참이면 세 단계 명령을 재생할 수 있습니다. Gazebo의 파지 결과는 별도 필드인 `grasp_success`에서 확인하세요.

```bash
source .venv/bin/activate
python mujoco_demo/demo_pick.py --trajectory log/pick-run/trajectory.json \
  --episodes 10 --report log/pick-run/mujoco.json
python mujoco_demo/compare_pick.py log/pick-run/trajectory.json \
  log/pick-run/gazebo.json log/pick-run/mujoco.json > log/pick-run/compare.json
```

MuJoCo는 저장된 **관절 명령**을 입력으로 사용합니다. Gazebo와 MuJoCo는 각각 접촉과 물리를 계산하므로, 명령이 같아도 상자와 그리퍼의 움직임이나 파지 결과는 달라질 수 있습니다.

## 개발 가이드

장면을 바꿀 때는 Gazebo의 [월드](src/m0609_gz/worlds/pick.sdf), MoveIt의 [Planning Scene 등록과 목표 자세](src/m0609_gz/scripts/record_pick), MuJoCo의 [장면 구성](mujoco_demo/model.py)을 함께 확인하세요. 상자 위치나 받침대를 바꿨다면 접촉 위치와 여유 공간에 맞춰 `ABOVE`, `AT_CUBE`를 갱신하고, MoveIt에서 새 궤적을 계획한 뒤 Gazebo 기록과 MuJoCo 반복 재생을 확인하세요. 독립 MuJoCo 데모의 자세는 [demo_pick.py](mujoco_demo/demo_pick.py)에 따로 있습니다.

새 행동·관측·보상은 [pick_env.py](mujoco_demo/pick_env.py)의 `reset()`, `observation()`, `step(action)`에서 시작하세요. 현재 환경은 접근 후 닫기 시각을 한 번 조정합니다. 변경 후에는 행동값 0이 기준 동작을 재현하고 에피소드가 정상 종료되는지 먼저 확인하세요.

| 확장 항목 | 추가할 것 | 먼저 검증할 것 |
| --- | --- | --- |
| 내려놓기 | 하강·열기·후퇴 단계와 성공 판정 | 받침대 간섭, 놓은 뒤 상자의 안정성 |
| 다른 공구·접촉 작업 | 공구 모델, TCP, 충돌 형상과 작업별 목표 | 좌표계 정합, 접근 여유, 의도한 접촉 |
| 실기 연동 | 실기 인터페이스와 별도 실행 절차 | 실제 TCP·관절 한계와 정지 절차 |
| 정책 학습 | 학습 루프, 초기 조건 변화, 평가 지표 | 행동값 0 재현, 에피소드 종료, 보상 일관성 |

| 파일·폴더 | 역할 |
| --- | --- |
| [src/m0609_gz/](src/m0609_gz/) | Gazebo 월드, 로봇 URDF, MoveIt 설정, 실행·기록 도구 |
| [mujoco_demo/models/](mujoco_demo/models/) | M0609 MJCF, RG2 모델과 메시, 라이선스 |
| [mujoco_demo/trajectories/](mujoco_demo/trajectories/) | 검증에 사용한 기준 관절 궤적 |
| [demo_pick.py](mujoco_demo/demo_pick.py), [replay.py](mujoco_demo/replay.py) | 독립 파지 데모와 JSON 궤적 재생 |
| [pick_env.py](mujoco_demo/pick_env.py), [compare_pick.py](mujoco_demo/compare_pick.py) | 학습 환경과 시뮬레이터 비교 도구 |
| [docs/](docs/) | [구현 상세](docs/implementation.md), [검증 기록](docs/validation.md), [초기 인계 계획](docs/initial-plan.md) |
