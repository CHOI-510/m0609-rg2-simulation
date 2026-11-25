# 검증 기록

현재 실행 순서는 [루트 README](../README.md), 장면 구성과 성공 판정 기준은 [구현 상세](implementation.md)에 있다.

## 이전 검증 기록


### MuJoCo

Python 3.12.3, MuJoCo 3.17.0, NumPy 2.5.5를 사용했다. 나머지 Python 의존성 버전은 [requirements.txt](../mujoco_demo/requirements.txt)에 고정했다.

- `--check-arm`에서 관절 6개와 모터 6개를 로드했다. 첫 모터에 30 N·m를 2초 동안 적용했을 때 1축이 0.144928 rad 움직였고, 초기화 후 시각·관절 위치·속도·제어 입력이 0으로 복원됐다.
- `--check-gripper`에서 열림 −0.497833 rad, 닫힘 +0.700232 rad를 측정했다. 패드 간격은 각각 102.39 mm와 7.71 mm였고 mimic 각도 오차는 최대 0.000002 rad였다.
- Gazebo용 전개 URDF의 순기구학과 MJCF의 TCP 위치·방향을 6축 영점 및 접근·파지 자세에서 비교했다. 최대 차이는 위치 0.000374 mm, 회전 0.000028°였다.
- `--episodes 10`에서 10회 모두 초기 상태가 일치했고, 상자 시작 높이 319.892 mm, 유지 구간 최소 상승량 83.091 mm, 양쪽 패드로 유지한 시간 1.000초를 기록했다.
- `--viewer`로 같은 시퀀스를 재생했다. 1회 성공 후 창이 닫혔고 종료 코드는 0이었다.
- 작업 디렉터리의 Git 메타데이터를 사용할 수 없어 소스와 자산만 임시 로컬 저장소에 담고 별도 경로에 복제했다. 그 체크아웃의 빈 가상환경에서 설치 명령과 `pip check`, 팔·그리퍼 검사, 헤드리스 10회를 확인했다. 팀원 PC에서의 실행은 아직 확인하지 않았다.

### Gazebo와 MoveIt

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, MoveIt 2.12.4, gz_ros2_control 1.2.20에서 확인했다.

- 기존 underlay 설정으로 별도 빌드·설치 경로에 `colcon build --symlink-install`을 실행해 1개 패키지를 빌드했다. 두 launch의 `--show-args`, 단독 및 RG2 Xacro 전개, RG2 모델의 `gz sdf -p`와 `gz sdf -k`가 성공했다.
- GUI launch 두 개를 로컬 도메인 110에서 실행해 Gazebo 스폰과 RViz의 MoveIt 초기화를 확인했다. 이어서 동일 도메인의 단일 서버 세션에서 `sim.launch.py rg2:=true`와 `moveit.launch.py rg2:=true rviz:=false`로 동작을 재검증했다. `ros2 control list_controllers`에서 세 컨트롤러가 모두 `active`였고, `ros2 topic echo /clock --once`에서 0.706초를 받았다.
- 동작 검증에는 RViz 버튼 대신 MoveIt의 `/move_action` 액션에 계획 및 실행 목표를 보냈다. 1축 0.50 rad 및 RG2 −0.50/+0.70 rad 목표가 모두 `SUCCEEDED`와 MoveIt 오류 코드 `SUCCESS (1)`로 끝났다. `/joint_states`에서 순서대로 0.499489, −0.498089, +0.698110 rad를 측정했다.
- 제한된 작업 셸에서는 DDS가 `failed to enumerate interfaces for "udp": -1`을, GLFW가 `X11: Failed to open display :0`을 반환했다. UDP와 디스플레이 접근을 허용한 로컬 세션에서 해당 검증을 완료했다.

### MoveIt → Gazebo → MuJoCo 파이프라인

Ubuntu 24.04에서 ROS 2 Jazzy, MoveIt 2.12.4, Gazebo Harmonic 8.15.0, gz_ros2_control 1.2.20과 Python 3.12.3, MuJoCo 3.17.0, NumPy 2.5.5를 사용했다. 최종 검증은 로컬 도메인 113 및 `GZ_PARTITION=m0609_pick_113`에서 실행했다. 실행한 명령 데이터는 [moveit_pick.json](../mujoco_demo/trajectories/moveit_pick.json)에 포함했다. Gazebo를 다시 시작하지 않고도 재생 및 환경 명령에 이 파일을 지정해 검증된 기준 동작을 실행할 수 있다. 이 작업 공간의 `log/pick-validated/`에는 각 시뮬레이터의 원시 측정값, 비교 보고서, 학습 환경 스모크 테스트 결과가 있다.

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
- 기본 샌드박스에서 `failed to enumerate interfaces for "udp": -1`이 발생해 DDS 검증은 허용된 로컬 셸에서 진행했다. 새 파이프라인은 헤드리스에서 검증했으며 당시 GUI 재실행은 확인 범위에 포함되지 않았다.

### 미검증 범위

- 새 파이프라인의 Gazebo·RViz GUI 재실행 및 다른 팀원 PC에서의 실행은 아직 검증 기록이 없다. 독립 MuJoCo 뷰어는 위에 기록한 환경에서 확인했다.
- 내려놓기, 실기 연동, 다른 공구 및 접촉 작업, 정책 학습은 현재 검증 범위 밖이다.

## 문서 개편 중 재실행

기존 프로젝트 가상환경에서 ROS 설정을 제외한 Python 실행으로 [README의 빠른 실행](../README.md#가장-빠른-실행--기준-궤적-재생)을 대조했다.

- `python -m pip check`에서 의존성 오류가 없었다.
- 포함된 기준 궤적의 헤드리스 재생은 **10/10회** 성공했다. 모든 회차의 초기화가 일치했고, 유지 구간 최소 상승량은 **83.621 mm**, 양쪽 패드 유지 시간은 **1.000초**였다.
- `PickEnv` 검사에서 행동값 0의 결과가 직접 재생과 일치했다. 행동값 0과 +0.150초 지연 에피소드가 모두 정상 종료됐고 보상은 각각 1이었다.
- 이번에는 기존 가상환경을 사용했다. Gazebo, RViz, DDS 실행 및 새 가상환경 설치는 재실행 범위에 포함하지 않았다. 해당 환경의 앞선 검증은 위 기록을 참고한다.
