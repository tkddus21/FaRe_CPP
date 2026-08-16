# FaRe-CPP: 장거리 센서 정보를 이용한 자율 순찰 로봇의 고속 재방문 커버리지 경로 계획

> 영문판: [README.md](README.md) · 두 문서는 같은 내용이므로 한쪽을 고치면 다른 쪽도 함께 고쳐 주세요.

## 개요

**FaRe-CPP**는 장거리 센서 정보를 활용해 자율 순찰 로봇의 재방문 커버리지 경로를 효율적으로 계획하는 알고리즘입니다. 이 저장소는 최적화된 순찰 경로를 생성하고 **AWS RoboMaker**, **Dynamic Logistics Warehouse** 같은 시뮬레이션 환경에서 실행하는 도구를 제공합니다.

본 연구를 사용하신다면 다음과 같이 인용해 주세요:
> S. Kachavarapu, T. Doernbach and R. Gerndt, "Fast-Revisit Coverage Path Planning for Autonomous Mobile Patrol Robots Using Long-Range Sensor Information," 2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Hangzhou, China, 2025, pp. 7677-7683, doi: [10.1109/IROS60139.2025.11246182](https://doi.org/10.1109/IROS60139.2025.11246182).

## 구현

이 저장소에는 FaRe-CPP 알고리즘의 두 가지 구현이 있습니다.

| 구현 | 디렉터리 | 설명 |
| --- | --- | --- |
| **Python** | [`FaRe/`](FaRe/) | 참조 구현. 오프라인 경로 계획기와 ROS/Gazebo 온라인 순찰 시뮬레이션(`PatrolSim.py`)을 포함합니다. 읽고 확장하기 가장 쉽습니다. |
| **C++** | [`FaRe_CPP/`](FaRe_CPP/) | 오프라인 경로 계획기를 독립적으로 이식한 고속 버전(약 10~100배 빠르며 Python 런타임 불필요). 빌드·사용법은 [`FaRe_CPP/README.md`](FaRe_CPP/README.md) 참고. |

> 참고: **FaRe-CPP**의 "CPP"는 C++가 아니라 **Coverage Path Planning**(알고리즘 이름)의 약자입니다. Python 패키지는 `FaRe/`에, C++ 이식본은 `FaRe_CPP/`에 있습니다.

## 시작하기

### 1. 시뮬레이션 환경 점유격자 맵 내려받기

FaRe-CPP를 실행하기 전에 아래 중 하나를 설치·설정하세요.
- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse)

### 2. 저장소 클론

```bash
git clone https://github.com/Srinikstudent/FaRe_CPP.git
```
또는 IPA coverage path planning과 동일한 설정을 원한다면:
```bash
git clone --branch FaRe_cpp_ipa_fov https://github.com/hcr-lab/FaRe-CPP.git
```

클론 후 `FaRe/config.py`의 경로를 환경 맵 파일(.pgm, .yaml) 위치로 수정하고, 로봇 센서 사양에 맞게 파라미터를 조정하세요. (C++ 구현은 대신 `FaRe_CPP/include/Config.hpp`를 수정 — [`FaRe_CPP/README.md`](FaRe_CPP/README.md) 참고.)

### 3. 의존성 설치

```bash
pip install -r requirements.txt
```

### 4. Surveillance 실행

```bash
python3 FaRe/Surveillance.py
```

점유격자 맵을 처리해 waypoint를 출력 디렉터리에 저장합니다. 이 waypoint들이 시뮬레이션 주행에 쓰입니다.

## 환경 선택

두 가지 환경이 준비돼 있습니다. `FARE_MAP`으로 고르며 기본값은 `aws`이므로, 기존 환경만 쓴다면 이 README의 모든 명령이 그대로 동작합니다.

| `FARE_MAP` | 월드 | 맵 | 결과 저장 위치 |
| --- | --- | --- | --- |
| `aws` (기본값) | AWS RoboMaker Small House | `aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi/` | `FaRe/results/` |
| `house` | 기본 `turtlebot3_house` | [`maps/turtlebot3_house/`](maps/turtlebot3_house/) | `FaRe/results/turtlebot3_house/` |

```bash
export FARE_MAP=house      # 모든 터미널에서 TURTLEBOT3_MODEL과 함께
```

환경마다 `output_dir`을 따로 두는 데는 이유가 있습니다. `results/wp_ori_data.txt`는 오프라인 계획기와 순찰 사이의 인계 지점이라, 디렉터리를 공유하면 house 순찰이 AWS waypoint를 주행하고도 아무 경고 없이 무의미한 실행 결과를 남깁니다.

프리셋은 [`FaRe/config.py`](FaRe/config.py) 상단의 `MAPS`에 있으며, 항목을 추가하면 환경이 늘어납니다. 프리셋의 시작점은 `starting_position`(격자 셀) 또는 `start_world`(미터, 로드 시 변환) 중 하나로 지정합니다. 직접 만든 맵에는 `start_world`를 권장합니다. `map_saver`는 원점을 임의로 잡기 때문에, 격자 셀을 하드코딩하면 맵을 다시 만들 때마다 조용히 엉뚱한 지점을 가리키게 됩니다.

### 계획 전에 맵 검사하기

```bash
FARE_MAP=house python3 FaRe/check_map.py
```

FaRe는 자유공간을 `unexplored_value`(254)와의 **정확한 일치**로 판별합니다. 따라서 rviz에서 멀쩡히 보이는 맵도 계획기에게는 절반이 보이지 않을 수 있습니다. 이미지 편집기로 내보낸 맵에는 255인 셀과 안티에일리어싱된 회색값이 섞이는데, `map_server`는 이를 자유공간으로 취급하지만 FaRe는 그러지 않습니다. 증상은 커버리지 수치가 집의 절반만 기준으로 조용히 계산되는 것뿐입니다. `check_map.py`는 `{0, 205, 254}`를 벗어난 값이 있으면 실패시키고, 자유 면적과 여유공간을 보고하며, `starting_position`을 스폰할 월드 좌표로 변환해 줍니다.

## turtlebot3_house 맵 만들기

이 집에는 기본 제공 맵이 없으므로 gmapping으로 직접 만듭니다. 두 가지는 피하세요. 이미지 편집기로 맵을 손보는 것(위 참고), 그리고 다른 월드용으로 만든 맵을 가져다 쓰는 것입니다. 몇 cm 어긋난 맵은 그 오차가 `inflation_radius`와 같은 수준이라, navigation 실패처럼 보이는 방식으로 goal을 깎아먹습니다.

```bash
sudo apt install ros-noetic-slam-gmapping     # 기본 설치돼 있지 않음
```

터미널 3개, 각각에서 `export TURTLEBOT3_MODEL=waffle_pi`:

```bash
# T1  월드 + 로봇
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_sim.launch

# T2  gmapping + move_base
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_mapping.launch open_rviz:=true

# T3  방을 찾고 → 빈틈을 메우고 → 저장
python3 ~/catkin_ws/src/FaRe_CPP/FaRe/explore_for_mapping.py
python3 ~/catkin_ws/src/FaRe_CPP/FaRe/tour_house_for_mapping.py
rosrun map_server map_saver -f ~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map

# T3  정원을 미탐색으로 되돌림
python3 ~/catkin_ws/src/FaRe_CPP/FaRe/crop_map_to_house.py \
    ~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map.pgm
```

이후 `check_map.py`로 검사하고, `Surveillance.py`로 이 맵의 waypoint를 생성합니다.

**`house_mapping.launch`가 매핑 스택 전부입니다** — robot_state_publisher, gmapping, move_base가 모두 들어 있습니다. `turtlebot3_slam.launch`를 같이 띄우지 마세요. 둘 다 같은 이름으로 노드를 올리고, roslaunch는 그 충돌을 먼저 뜬 쪽을 죽여서 해결합니다. 살아남은 쪽에는 scan 변환이 없고 `/map`은 다시는 발행되지 않습니다. 겉으로는 move_base가 안 올라오는 것처럼 보이지만, 그때까지 만든 맵은 이미 사라진 뒤입니다.

**주행 스크립트 두 개를 이 순서로.** `explore_for_mapping.py`는 gmapping이 실시간으로 내보내는 `/map`에서 프론티어(미탐색에 닿아 있는 자유 셀)를 골라 갑니다. 방을 *찾는* 데는 강하지만 *끝내는* 데는 약합니다 — 프론티어가 없어지는 즉시 멈추는데, 4 m 밖에서 한 번 스친 벽도 얼룩인 채로 그 조건을 만족하기 때문입니다. `tour_house_for_mapping.py`는 반대로 접근합니다. 이 집은 이미 알려진 정적 월드이므로, `turtlebot3_gazebo`의 `model.sdf`에서 방 경계를 읽어 그 위에 1.2 m 격자를 깔고 전부 방문합니다. gmapping의 `maxUrange` 3.0 m보다 촘촘하게 잡은 값이라, 모든 방의 모든 지점을 스캔이 닿는 거리에서 봅니다. 넓은 방 한가운데를 손으로 지나가라던 기존 안내를 대신하는 부분입니다 — 채워지지 않은 실내는 장벽으로 취급되고, `diagnose_waypoints.py`는 그런 곳에 waypoint를 놓지 않습니다. 탐색 후에 투어를 이어 돌려도 안전합니다. 같은 gmapping 노드가 계속 누적합니다.

**크롭이 필요한 이유.** 집의 동쪽 벽은 y = -0.40 ~ 0.50 구간이 트여 있고 남쪽 벽에도 x = 5.0 ~ 5.8 부근에 틈이 있어, 레이저가 그대로 잔디밭으로 빠져나갑니다. gmapping은 정원을 자유공간으로 기록하고 이를 막을 경계가 없습니다. 그대로 두면 FaRe가 잔디에 waypoint를 놓고 잔디까지 포함한 커버리지를 보고합니다. `crop_map_to_house.py`는 `model.sdf`의 방 영역으로 맵을 한정합니다 — 동봉된 맵 기준 100.7 → 93.4 ㎡로 측정됐습니다. 위에서 경고한 이미지 편집기 손질과는 다릅니다. 리샘플링이 없고, 쓰는 값은 맵이 이미 미탐색에 쓰고 있는 205 하나뿐이며, origin을 건드리지 않으므로 맵은 여전히 Gazebo 월드 좌표와 정렬됩니다. 크롭 전 원본을 `.orig`로 남기고 매 실행마다 그쪽에서 다시 읽으므로, 여백을 넓혀 다시 돌리는 것도 그대로 됩니다.

## 온라인 주행(순찰)

시뮬레이션에서 순찰을 실행하려면 ROS와 Gazebo가 설치돼 있어야 합니다.

사전 준비:

1. ros-noetic과 Gazebo 시뮬레이션이 정상 동작할 것
2. [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) 또는 [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse), 혹은 원하는 환경을 설정할 것
3. `python3 FaRe/Surveillance.py`를 먼저 실행해 `results/wp_ori_data.txt`에 최적화된 waypoint가 저장돼 있을 것

### 전체 흐름 한눈에 보기

```
오프라인 (맵당 1회)                온라인 (실행당 1회)
─────────────────────            ────────────────────────────────────
Surveillance.py                  T1  view_small_house.launch      월드
  frontier + FOV 선정            T2  spawn_turtlebot3.launch      로봇
  GRASP waypoint 순서 최적화     T3  fare_navigation.launch       map_server + AMCL + move_base
  ↓                              T4  set_initial_pose.py          AMCL 초기 자세
results/wp_ori_data.txt  ──────────► run_patrol_test.sh           기록 + 순찰
  (좌표와 방향만)                     ↓
                                 results/<날짜>_<시각>_<라벨>/
```

FaRe가 제공하는 것은 **어디에 서서 어느 쪽을 볼지**뿐입니다. waypoint 사이의 경로는 move_base(NavfnROS + DWA)가 실시간 costmap을 보고 온라인으로 계획합니다. `path.png`에 그려지는 A\* 경로는 지표 계산과 시각화 전용이며 **실제 주행에 쓰이지 않습니다.**

이 분리가 두 가지를 설명합니다. 로봇 모델을 바꿔도 waypoint를 다시 생성할 필요가 없다는 점, 그리고 goal 실패가 경로 계획기가 아니라 navigation stack의 문제라는 점입니다.

## 온라인 주행 실행 절차

터미널 4개를 사용합니다. 각 터미널에서 `source ~/catkin_ws/devel/setup.bash`와 `export TURTLEBOT3_MODEL=waffle_pi`를 먼저 실행하세요. (`burger`도 가능하지만, `waffle_pi`가 동봉된 맵을 만들 때의 스캔 높이와 일치하며 실측에서도 더 좋았습니다 — 24/26 대 20/26.) turtlebot3_house를 쓴다면 `export FARE_MAP=house`도 함께 실행합니다.

**터미널 1 — 월드 실행**
```bash
roslaunch aws_robomaker_small_house_world view_small_house.launch
```
또는
```bash
roslaunch dynamic_logistics_warehouse logistics_warehouse.launch
```

**터미널 2 — 그 월드에 TurtleBot 3 스폰**
```bash
roslaunch aws_robomaker_small_house_world spawn_turtlebot3.launch x_pos:=4.65 y_pos:=-2.0
```
`x_pos`/`y_pos`는 첫 waypoint(`FaRe/config.py`의 `starting_position`)의 월드 좌표입니다. 계획된 지점에서 순찰이 시작되도록 맞춘 값입니다. 격자 셀 `(row, col)`은 다음으로 변환합니다:
`x = col * resolution + origin[0]`, `y = (map_height - 1 - row) * resolution + origin[1]`
`check_map.py`가 이 스폰 명령을 그대로 출력해 주므로 그대로 복사해 써도 됩니다.

> 여기서 `roslaunch turtlebot3_gazebo turtlebot3_world.launch`를 쓰면 **안 됩니다.** 그 launch 파일은 자체 gzserver를 turtlebot3 전용 월드로 띄우므로, 집 안에 스폰되는 게 아니라 **다른 월드가 하나 더 열리고** 터미널 1과 `gazebo` 노드 이름이 충돌합니다.

**turtlebot3_house의 터미널 1·2** — launch 파일 하나가 둘 다 처리합니다:
```bash
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_sim.launch
```
기본 스폰 위치는 이 집의 기본 지점인 `(-3.0, 1.0)`이며, `house` 프리셋이 waypoint 0으로 변환하는 좌표와 같습니다. `start_world`를 바꿨다면 `x_pos:=`/`y_pos:=`로 맞추세요.

> 여기에 `roslaunch turtlebot3_gazebo turtlebot3_house.launch`를 쓰면 **안 됩니다.** 같은 월드를 띄우긴 하지만 Gazebo 모델 이름을 그냥 `turtlebot3`로 스폰합니다. 반면 `set_initial_pose.py`와 `run_patrol_test.sh`의 AMCL 사전 검사는 둘 다 `turtlebot3_$TURTLEBOT3_MODEL`을 조회합니다. 사전 검사에는 이 이름을 바꿀 수단이 없어서, 찾지 못한 모델의 0 자세와 AMCL을 비교하고 모든 실행을 중단시킵니다.

**터미널 3 — navigation stack (map_server + AMCL + move_base)**
```bash
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/fare_navigation.launch
# turtlebot3_house:
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/fare_navigation.launch \
  map_file:=$HOME/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map.yaml
```
`PatrolSim.py`는 `move_base` 액션 서버로 목표를 보내는데, 이게 떠 있어야만 동작합니다.

이 launch는 기본 `turtlebot3_navigation.launch`에 `launch/costmap_override.yaml`을 얹은 것입니다. 기본 파라미터는 이 맵에서 로봇을 문틈에 끼게 만듭니다(아래 "알려진 한계" 참고). `FaRe_CPP`는 catkin 패키지가 아니므로 경로로 직접 실행합니다. 기본값은 AWS 하우스 맵이며, 다른 맵은 `map_file:=<경로>`로 지정하되 `FaRe/config.py`가 가리키는 맵과 같아야 합니다.

> `open_rviz:=false`는 의도된 설정입니다. 내장 GPU에서는 rviz와 Gazebo 클라이언트를 같이 띄우면 렌더러가 고갈되어 gzserver가(그리고 ROS 마스터까지) 실행 도중 죽을 수 있습니다. GPU 여유가 있고 costmap을 보고 싶을 때만 `open_rviz:=true`를 쓰세요.

**터미널 4 — AMCL 초기 자세 설정 후 순찰**
```bash
python3 FaRe/set_initial_pose.py
./FaRe/run_patrol_test.sh              # 권장: 실행마다 별도 폴더에 기록
```
`set_initial_pose.py`는 Gazebo에서 로봇의 실제 자세(`/gazebo/get_model_state`)를 읽어 `/initialpose`로 발행합니다. rviz의 "2D Pose Estimate" 버튼과 같은 일입니다. AMCL은 초기 자세 없이는 위치를 추정하지 못하고 그동안 모든 goal이 실패하므로, **선택이 아니라 필수 단계**입니다. `open_rviz:=true`로 띄웠다면 "2D Pose Estimate"를 직접 클릭해도 됩니다.

`run_patrol_test.sh`는 `PatrolSim.py`를 감싼 것이며(원본을 그대로 실행해도 됩니다), 단순 동작 확인이 아니라 **측정이 목적일 때** 쓰면 좋습니다. 어느 쪽이든 `results/wp_ori_data.txt`의 waypoint를 주행하고 goal마다 한 줄씩(`SUCCEEDED` / `ABORTED` / `TIMEOUT`, 소요시간 포함) `patrol_log.csv`에 기록합니다. goal마다 120초 타임아웃이 있어 도달 불가능한 waypoint 하나가 전체 실행을 멈추지 않으며, 실패한 goal 뒤에는 로봇이 costmap을 비우고 후진해 끼임에서 빠져나옵니다.

## 실행 기록하기

`./FaRe/run_patrol_test.sh [라벨]`은 순찰 한 번을 `<output_dir>/<날짜>_<시각>_<라벨>/`에 담습니다. 덮어쓰지 않고 실행이 쌓입니다. 라벨을 생략하면 `${FARE_MAP}_${TURTLEBOT3_MODEL}`이 쓰입니다. 실행 간에 보통 이 둘이 달라지며, 환경이 둘 이상이면 `waffle_pi`라는 이름만으로는 어떤 실행인지 알 수 없기 때문입니다.

```bash
./FaRe/run_patrol_test.sh                  # -> results/20260729_2350_aws_waffle_pi
FARE_MAP=house ./FaRe/run_patrol_test.sh   # -> results/turtlebot3_house/20260729_2350_house_waffle_pi
./FaRe/run_patrol_test.sh infl03           # -> results/20260729_2350_infl03
RECORD_SCAN=1 ./FaRe/run_patrol_test.sh    # /scan도 기록 (bag이 훨씬 커짐)
```

이 스크립트는 `results/`를 가정하지 않고 `FaRe/config.py`의 `output_dir`을 읽으므로, `FARE_MAP`이 고른 환경을 그대로 따라갑니다.

폴더명을 `test1`, `test2` 대신 이렇게 정한 이유는, 시간순 정렬과 충돌 방지는 타임스탬프가, "무엇을 바꿨는지"는 라벨이 담당하기 때문입니다. 번호만으로는 나중에 어떤 조건이었는지 알 수 없습니다.

각 폴더에는 bag, `patrol_log.csv`, **실제로 주행한** `wp_ori_data.txt`, 커버리지·지표 산출물, 그리고 모델·사용한 맵·적용된 costmap 파라미터·git 리비전을 담은 `run_info.txt`가 남습니다. 몇 달 뒤에도 수치를 해석할 수 있도록 하기 위함입니다.

기록을 시작하기 전에 스크립트가 확인하는 것: `move_base`가 떠 있는지, waypoint가 있는지, 그리고 **AMCL 추정이 Gazebo 실측과 0.5 m 이내로 일치하는지**입니다. 마지막 검사가 중요합니다. 파티클 필터가 발산하면 로그상으로는 주행 실패처럼 보이지만 실제로는 **측정 실패**이고, 로그만 봐서는 구분되지 않습니다. 또한 `rosbag`을 SIGKILL이 아닌 **SIGINT**로 종료합니다. 강제 종료된 bag은 색인이 없어 재생이 불가능할 수 있습니다.

### 기록 재생하기

```bash
rosparam set /use_sim_time true                     # rviz를 띄우기 "전"에
rviz -d launch/patrol_playback.rviz
rosbag play --clock -d 5 -r 3 results/<실행폴더>/patrol.bag
```
`rviz`는 시작할 때 `use_sim_time`을 읽습니다. 나중에 설정하면 rviz가 벽시계 시간으로 동작해 화면에 아무것도 나오지 않습니다.

`-d 5` 지연도 필요합니다. `/move_base/global_costmap/costmap`은 **단 한 번 발행되고 latch도 걸리지 않아서**, 지연이 없으면 rviz의 구독이 성립하기 전에 지나가 버려 global costmap이 끝내 표시되지 않습니다. (`/map`은 latch가 걸려 있어 늦게 구독해도 표시됩니다.)

## 커버리지·탐지 성능 확인

```bash
python3 FaRe/check_map.py             # 맵 자체: 3값 여부, 자유 면적, 시작 셀
python3 FaRe/diagnose_waypoints.py    # waypoint별 여유공간 vs 로봇 풋프린트
python3 FaRe/trash_eval.py            # 계획 경로가 배치된 물체를 몇 개나 보는지
python3 FaRe/trash_eval.py --range 5  # 낙관적인 최대 사거리 기준과 비교
```
`trash_eval.py`는 AWS 전용입니다. `FaRe/trash_positions.txt`에 들어 있는 격자 좌표는 그 맵 기준이라 다른 맵에서는 엉뚱한 지점을 가리킵니다.
`Surveillance.py`는 `results/coverage_map.png`(주황 = 센서가 본 영역, 빨강 = 놓친 자유공간)도 만들고 커버리지 비율을 `results/metrics.csv`에 덧붙입니다. `trash_eval.py`가 쓰는 물체 위치는 `FaRe/trash_positions.txt`에 `row, col` 격자 좌표로 들어 있습니다.

## 알려진 한계

- **`PatrolSim.py` 좌표 변환 (수정됨):** `grid_to_world_coords()`가 row/col을 뒤바꿔 쓰고 `map_server`의 상하 반전(pgm의 row 0 = 이미지 맨 위 = 월드 y의 *최댓값*, 맵 원점은 좌하단 픽셀)을 반영하지 않아 goal이 엉뚱한 위치로 전송됐습니다. 테스트로 확인: 수정 전에는 개활지 맵에서도 모든 waypoint가 실패하거나 엉뚱한 곳에 도달했고, 수정 후 13~14/14가 성공했습니다. 같은 작업에서 함께 고친 것: 잘못된 goal 쿼터니언(실제 yaw 쿼터니언 대신 `orientation.z = theta`), `wait_for_result()`만 믿던 성공 판정(`ABORTED`인 goal에도 `True`를 반환함), 그리고 `wp`(waypoint, 시작점 복귀 지점 포함)가 `ori`(방향)보다 하나 길어서 생기던 `IndexError`.

- **goal *방향*에도 위치와 똑같은 좌표계 뒤집힘이 있었음 (수정됨):** `grid_to_world_coords()`는 pgm 상하 반전을 반영하도록 고쳤지만, 그 옆에 같이 넘어가던 방향값은 고쳐지지 않았습니다. `cast_fov()`는 row 축이 *아래로* 증가하는 격자 좌표에서 부채꼴을 훑는 반면 월드 y는 위로 증가하므로, 월드 yaw는 `theta`가 아니라 `-theta`입니다. `PatrolSim.py`는 이 값을 변환 없이 그대로 보냈습니다. `Scout.fov()`가 방향을 `[0, 90, 180, 270]`도로 양자화하기 때문에 수직 방향 두 개가 정확히 반대로 나갔고, **AWS 26개 waypoint 중 15개(58%)가 정반대를 바라봤습니다.** 위치는 멀쩡하니 goal은 `SUCCEEDED`로 기록되고 로그 어디에도 이상 징후가 없어서, 대가는 로봇이 조용히 관측하지 못한 커버리지뿐이었습니다. 위치 수정 때 함께 발견되지 않은 이유가 이것입니다. 지금은 같은 경계에서 `grid_yaws_to_world()`가 변환합니다. `wp_ori_data.txt`는 격자 공간 그대로 두는데, `Surveillance.py`와 `trash_eval.py`가 격자 좌표에서 FOV를 계산하므로 변환하면 안 되기 때문입니다. `patrol_log.csv`에는 `grid_theta_rad` 열이 추가되어, `yaw_rad`를 "실제로 명령한 yaw"로 읽을 수 있습니다.

  이 수정 이전에 기록된 실행의 goal 성공률은 여전히 유효합니다(도착 여부를 재는 값이므로). 다만 그 실행들이 *실제로 확보한* 커버리지는 계획치보다 낮았습니다.

- **센서 모델은 평면 2D 부채꼴 (의도된 설계이나 알아둘 것):** FOV는 점유격자 위에서 광선을 쏘는 90° 부채꼴이며, 0.05~5 m 범위에 가려짐은 반영되지만(광선이 장애물에서 멈춤) 수직 방향 개념·센서 높이·장착 형상은 없습니다. 격자 평면 자체가 스캔 평면이고, 그 높이는 맵을 만든 높이입니다. 방향은 `[0, 90, 180, 270]`도 네 개 후보 중 보이는 면적이 가장 큰 것으로 정해집니다. `surveillance_range`는 5 m인데 waffle_pi의 LDS는 3.5 m까지라, 계획 커버리지는 두 맵 모두에서 낙관적입니다. 미탐색 셀은 광선을 막지도, 본 것으로 세지도 않습니다. (`FaRe_CPP/src/FOV.cpp`가 카메라 모델로 수직 FOV를 계산하지만 호출하는 곳이 없습니다 — C++ 계획기도 같은 평면 부채꼴을 씁니다.)

- **waypoint 여유공간이 로봇 풋프린트를 반영하지 않던 문제 (수정됨):** 예전 `find_frontier_cells()`는 후보 셀의 ±4셀 정사각 이웃에 벽이 없으면 통과시켰습니다. 문제가 둘이었습니다. 정사각형은 *체비쇼프* 거리를 재므로 로봇 크기와 무관하게 5셀 = 0.25 m만 보장했고, 벽(`0`)만 검사해서 미탐색 공간(`205`)은 — `crop_map_to_house.py`가 그리는 경계까지 포함해 — 아예 보이지 않았습니다. house 맵에서 26개 중 14개가 벽에서 정확히 0.25 m에, 하나는 크롭 경계에서 0.05 m에 놓였습니다.

  이제 생성은 **inflated 맵** 위에서 이뤄집니다. move_base의 costmap inflation에 대응하는 계획기 쪽 장치로, `Exploration.placeable_mask()`가 모든 장벽(벽과 미탐색 셀 모두)을 거리변환 한 번으로 `config['waypoint_clearance']`만큼 부풀리고, 살아남은 영역에만 waypoint를 놓습니다. `diagnose_waypoints.py`도 같은 상수로 판정하므로, 갓 생성한 집합은 정의상 `TIGHT`도 `UNREACHABLE`도 0입니다. 즉 이 스크립트는 걸러내는 필터가 아니라 "이 waypoint 파일이 이 맵에서 나온 게 맞는지" 확인하는 검사가 됩니다.

  마스크는 **위치만** 제약합니다. 프론티어 판정과 `cast_fov()`는 여전히 실제 격자에서 돌기 때문에 inflation이 커버리지를 깎지 않습니다. 로봇이 설 수 없는 셀이라도 볼 수는 있으니까요.

  `config['waypoint_clearance']`는 실측으로 0.35 m입니다. 아래 경계: 벽에서 0.30 m인 goal은 `ABORTED`, 0.32 m는 `SUCCEEDED`였으므로 `robot_radius * 2` = 0.31이 딱 그 칼날 위입니다. 위 경계: 0.45 m에서는 house 맵의 배치 가능 영역이 두 조각으로 갈라집니다. 0.35는 house 자유공간의 64%를 하나의 연결 요소로 남기고(AWS 69%), 모든 자유 셀이 여전히 `surveillance_range` 안에서 배치 가능 지점을 갖습니다.

  turtlebot3_house + waffle_pi 실주행 측정: **수정 전 12/26, 수정 후 25/26**(`results/turtlebot3_house/20260811_1736_house_waffle_pi`), 소요 10.7분, goal 중앙값 17초. 계획 커버리지는 98.80% → 98.13%로 거의 그대로이고 순회 거리는 오히려 짧아졌습니다(94.85 → 76.45 m).

  AWS 하우스도 같은 날 돌려 **25/26**이었습니다(`results/20260811_1757_aws_waffle_pi`, 소요 10.5분, goal 중앙값 18.6초). 수정 전 마지막 비교 가능한 실행은 21/26이었습니다. 다만 이 숫자는 조심해서 읽어야 합니다 — 기준선이 같은 세션에서 짝지어 측정한 값이 아니라 2026-07-30 기록이고, 이 도구로 돌린 과거 AWS 실행들이 20/26에서 24/26 사이에 흩어져 있어서 4개 차이는 이 맵이 원래 보이는 편차 범위 안입니다. AWS에서 확실한 것은 정적 결과 쪽입니다 — `TIGHT` 5개가 0개가 되고 커버리지는 96.2%로 유지됐으며, 대가는 19% 늘어난 순회 거리였습니다. 여유공간이 애초에 제약이 아니던 맵에서 inflation은 공짜가 아니지만, 적어도 goal을 깎아먹지는 않았습니다.

  유일하게 남은 실패가 오히려 중요합니다. 배치 문제도, 이동 중 끼임도 아니고 지역 계획기(DWA) 문제였습니다. 기록된 bag으로 추적한 내용은 아래 "DWA가 개활지에서 오작동한다" 항목에 정리했습니다. 생성 단계가 여유공간을 강제하게 된 지금은 `diagnose_waypoints.py`로 설명되는 실패가 남지 않으므로, 진짜 원인이 오히려 또렷해졌습니다.

  `FaRe_CPP/src/Scout.cpp`는 아직 예전 정사각 검사를 쓰므로 두 구현이 이 지점에서 갈라져 있습니다.

- **DWA가 개활지에서 오작동함 (미해결; 수정 1회 시도 후 되돌림):** 배치 수정 뒤에도 남는 실패는 전부 지역 계획기가 **여유가 충분한 곳에서** 길을 잃는 것입니다. 기록된 bag으로 추적한 두 가지 형태입니다.

  *목표에서 멀어지는 형태.* `results/turtlebot3_house/20260811_1736_house_waffle_pi`에서 goal #8 (6.95, 0.10)이 120초 `TIMEOUT`했습니다. waypoint는 멀쩡했고(여유공간 0.350 m, 접근 경로 0.79 m 안에 벽 없음) 계획도 멀쩡했습니다 — NavFn의 첫 plan은 1.4 m 직선이었고 120초 내내 끝점이 목표에 붙어 있었습니다. 로봇은 후진으로 **0.193 m**까지 접근해 x를 6.51 → 6.78로 좁혔지만 y가 0.190에 멈춘 채 목표 0.10까지 못 갔고, 이후 전진으로 뒤집혀 동쪽을 가리키는 계획을 두고 **서쪽으로 12 m**를 달렸습니다. tf 샘플 3719개 중 목표 0.15 m 안에 들어온 것은 하나도 없습니다.

  마지막 0.19 m를 못 좁힌 이유는 스톡 `dwa_local_planner_params_waffle_pi.yaml`이 설명합니다. 시뮬레이션 가능한 최소 이동거리가 `min_vel_trans * sim_time` = 0.13 × 2.0 = **0.26 m**인데 `xy_goal_tolerance`는 **0.05 m**입니다. tolerance 안에서 끝나는 후보 궤적이 하나도 없으니 전부 지나치고, 지나친 뒤에는 반대 방향이 최고점을 받습니다. 이 형태는 짧은 goal에서만 나타납니다 — 주행/직선 거리 배율이 #8 26배, #16 38배, #6 9배인 반면 2.7 m 이상 goal은 전부 1.0~1.8배였습니다. 25개 goal 중 10개가 이전 waypoint에서 0.30 m 이내이고 3개는 **같은 셀에 방향만 다르기** 때문에 자주 걸립니다. `set_goals()`가 반복마다 거의 같은 프론티어 셀을 다시 고르기 때문입니다.

  *제자리에서 진동하는 형태.* `results/turtlebot3_house/20260817_0152_arrival_radius`에서 로봇이 우측 wing의 (5.73, −2.74)에 멈춰 60초간 진동하다가, move_base가 `Robot is oscillating. Even after executing recovery behaviors.`를 내고 abort했습니다. 이후 goal 3개도 함께 무너졌습니다. 그 지점의 정적 여유공간은 0.492 m, 자체 local costmap 기준 북쪽 자유 통로 폭은 0.90 m, 유효한 96포즈·2.5 m 전역 경로가 내내 존재했습니다. 막는 것이 없었습니다.

  **시도했다가 되돌린 수정.** `PatrolSim.send_goal()`이 순찰에 필요 없는 정밀도를 요구하지 않도록 바꿔 봤습니다. waypoint는 센서를 놓는 자리이고 `surveillance_range`가 5 m이므로, 로봇이 0.25 m 안에 들어오면 goal을 취소하고 제자리에서 FOV가 원하는 방향으로 돌리는 방식입니다. 메커니즘은 설계대로 작동했습니다 — 26개 중 22개가 그렇게 끝났고 오차는 0.038~0.240 m, 짧은 goal은 크게 단축됐습니다(#16 40.8초 → 2.2초, #12 13.5초 → 2.5초). **그런데도 결과는 25/26 대비 22/26으로 나빴습니다.**

  이유가 배울 점입니다. 우측 wing에는 `x ≈ 6.30`의 장애물을 사이에 두고 나란한 통로가 둘 있습니다. 회전으로 goal을 끝내면 로봇이 waypoint에서 최대 0.25 m 떨어진 곳에 남는데, wing 탈출을 (6.25, −4.92) 대신 (6.06, −4.75)에서 시작한 것만으로 NavFn이 동쪽 통로(x ≈ 6.75, 성공했던 쪽)에서 서쪽 통로(x ≈ 5.75, 진동이 일어난 쪽)로 선택을 뒤집었습니다. 즉 이 변경이 실패를 *만든* 것이 아니라 통로 선택을 바꿨고, 그중 한쪽에 지역 계획기 결함이 숨어 있었습니다. 조건당 실행이 1회뿐이라 이를 일반적인 편차와 구분할 방법이 없고 22/26은 25/26보다 나쁘므로, 추측으로 유지하지 않고 되돌렸습니다.

  다시 시도하기 전에 알아둘 것: 위 두 형태는 같은 용의자이고, 스톡 DWA 파라미터도 `set_goals()`의 근접 중복 waypoint도 아직 손대지 않았습니다. 재시도할 때는 조건마다 여러 번 돌려야 합니다 — 이 맵에서 단일 실행으로는 수정과 운을 구분할 수 없습니다.

   AWS Small House 맵 실측: **26개 waypoint 전부가 burger 풋프린트를 통과**했고, 최소 여유공간은 0.25 m로 풋프린트 반폭 0.105 m 대비 여유가 있었습니다. 즉 이 맵에서는 배치가 기하학적으로 주행 불가능한 게 아니며, 앞서의 불안정성은 통로 폭이 아니라 위 항목의 좌표/쿼터니언/짝맞춤 버그 때문이었습니다. 그 수정 후 waypoint 5개 순찰은 5/5 `SUCCEEDED`였습니다. 다만 단일 지점 검사라 다른 맵에서는 뚫릴 수 있으므로 이 검사는 유지할 가치가 있습니다.

- **끼임 한 번이 순찰 전체를 무너뜨리던 문제 (수정됨):** 가구가 많은 맵에서는 waypoint *사이를 이동하는 중에* 로봇이 자기 폭과 거의 같은 틈으로 들어가 끼일 수 있습니다. 이때 move_base 자체의 rotate recovery는 동작을 거부하고("can't rotate in place because there is a potential collision. Cost: -1.00"), 로봇은 갇힌 채로 남아 이후 모든 goal이 움직이지 못하는 로봇을 상대로 실패합니다. 이제 `PatrolSim.py`는 `SUCCEEDED`가 아닌 goal 뒤에 `/move_base/clear_costmaps`를 호출하고 `/cmd_vel`로 직접 약 16 cm 후진합니다. move_base가 스스로 할 수 없는 동작입니다. 이 변경 하나로 AWS 하우스 순찰이 7/26에서 20/26으로 올랐습니다.

- **기본 costmap 파라미터가 실내 맵에 맞지 않음 (`launch/costmap_override.yaml`에서 수정):** `turtlebot3_navigation`은 `inflation_radius: 1.0`으로 배포되는데, AWS 하우스 맵의 자유공간 여유공간 *중앙값*은 0.552 m뿐입니다. 주행 가능 영역의 절반 이상이 inflation 경사 안에 들어가 비용이 낮은 공간이 사실상 없습니다. 오버라이드를 적용하려면 `turtlebot3_navigation.launch` 대신 `launch/fare_navigation.launch`를 쓰세요.

   AWS 하우스 맵 실측, 실행당 waypoint 26개:

   | 설정 | 도달 goal | 비고 |
   | --- | --- | --- |
   | 기본값 (`inflation_radius: 1.0`) | 7/26 | goal 8에서 끼인 뒤 이후 전부 실패 |
   | `inflation_radius: 0.30` | 7/26 | 같은 지점에서 끼임, 오히려 3 cm 더 깊이 진입 |
   | + `footprint_padding: 0.045` | 4/26 | **악화** — 패딩은 문제의 병목뿐 아니라 모든 통로를 좁힘 |
   | + `footprint_padding: 0.020` 및 PatrolSim 복구 | **20/26** | 연쇄 실패 대신 복구 |

   셋째 행을 주목하세요. inflation을 낮추거나 풋프린트를 부풀리는 것은 공짜가 아닙니다. 유효 반경 0.150 m까지 패딩하면 문제의 0.112 m 병목은 막히지만, 그 전까지 잘 지나다니던 0.180 m 지점에서 새로 끼였습니다.

- **waypoint 여유공간이 goal 실패를 예측하는지는 맵에 따라 다름 (두 결과 모두 유효):** 처음 검증은 **AWS 하우스 + burger**에서 이뤄졌고 기각됐습니다. 20/26 실행에서 실패한 goal의 여유공간 중앙값은 0.450 m, 성공한 goal은 0.480 m로 구분되지 않았고, *실패한* goal 중 하나는 1.412 m의 개활지에 있었습니다. 모든 waypoint가 0.105 m 풋프린트를 여유 있게 통과했으니 그 맵에서는 여유공간이 제약 조건 자체가 아니었던 것이고, 실패는 **이동 중에** global planner가 waypoint들이 피해 간 병목으로 경로를 잡아서 생겼습니다.

  **turtlebot3_house + waffle_pi**에서는 같은 측정이 정반대로, 그것도 아주 깨끗하게 나옵니다.

  | 여유공간 판정 | `SUCCEEDED` | 실패 |
  |---|---|---|
  | `OK` (≥ 0.31 m) | **11** | 0 |
  | `TIGHT` (0.155~0.31 m) | 1 | **13** |
  | `UNREACHABLE` (< 0.155 m) | 0 | **1** |

  두 결과 모두 사실입니다. house가 더 좁고(자유공간 여유공간 중앙값 0.450 m 대 0.552 m) waffle_pi가 burger보다 1.5배 넓기 때문에(0.155 m 대 0.105 m), 한쪽 맵에서 넉넉했던 waypoint가 다른 쪽에서는 costmap inflation 경사 안에 들어가 global planner가 아예 경로를 만들지 못합니다. 교훈은 둘 중 하나가 틀렸다는 게 아니라 **이 측정은 맵마다·로봇마다 다시 해야 한다**는 것이고, `diagnose_waypoints.py`가 바로 그 용도입니다. 위의 배치 수정은 house 쪽 문제를 해결하며, 이동 중 실패는 여전히 복구 동작과 costmap 오버라이드가 담당합니다.
