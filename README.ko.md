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

터미널 4개를 사용합니다. 각 터미널에서 `source ~/catkin_ws/devel/setup.bash`와 `export TURTLEBOT3_MODEL=waffle_pi`를 먼저 실행하세요. (`burger`도 가능하지만, `waffle_pi`가 동봉된 맵을 만들 때의 스캔 높이와 일치하며 실측에서도 더 좋았습니다 — 24/26 대 20/26.)

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

> 여기서 `roslaunch turtlebot3_gazebo turtlebot3_world.launch`를 쓰면 **안 됩니다.** 그 launch 파일은 자체 gzserver를 turtlebot3 전용 월드로 띄우므로, 집 안에 스폰되는 게 아니라 **다른 월드가 하나 더 열리고** 터미널 1과 `gazebo` 노드 이름이 충돌합니다.

**터미널 3 — navigation stack (map_server + AMCL + move_base)**
```bash
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/fare_navigation.launch
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

`./FaRe/run_patrol_test.sh [라벨]`은 순찰 한 번을 `results/<날짜>_<시각>_<라벨>/`에 담습니다. 덮어쓰지 않고 실행이 쌓입니다. 라벨을 생략하면 `$TURTLEBOT3_MODEL`이 쓰입니다. 실행 간에 보통 이 값이 달라지기 때문입니다.

```bash
./FaRe/run_patrol_test.sh              # -> results/20260729_2350_waffle_pi
./FaRe/run_patrol_test.sh infl03       # -> results/20260729_2350_infl03
RECORD_SCAN=1 ./FaRe/run_patrol_test.sh    # /scan도 기록 (bag이 훨씬 커짐)
```

폴더명을 `test1`, `test2` 대신 이렇게 정한 이유는, 시간순 정렬과 충돌 방지는 타임스탬프가, "무엇을 바꿨는지"는 라벨이 담당하기 때문입니다. 번호만으로는 나중에 어떤 조건이었는지 알 수 없습니다.

각 폴더에는 bag, `patrol_log.csv`, **실제로 주행한** `wp_ori_data.txt`, 커버리지·지표 산출물, 그리고 모델·적용된 costmap 파라미터·git 리비전을 담은 `run_info.txt`가 남습니다. 몇 달 뒤에도 수치를 해석할 수 있도록 하기 위함입니다.

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
python3 FaRe/diagnose_waypoints.py    # waypoint별·구간별 여유공간, 직전 실행 결과와 함께 표시
python3 FaRe/trash_eval.py            # 계획 경로가 배치된 물체를 몇 개나 보는지
python3 FaRe/trash_eval.py --range 5  # 낙관적인 최대 사거리 기준과 비교
```
`Surveillance.py`는 `results/coverage_map.png`(주황 = 센서가 본 영역, 빨강 = 놓친 자유공간)도 만들고 커버리지 비율을 `results/metrics.csv`에 덧붙입니다. `trash_eval.py`가 쓰는 물체 위치는 `FaRe/trash_positions.txt`에 `row, col` 격자 좌표로 들어 있습니다.

## 알려진 한계

- **`PatrolSim.py` 좌표 변환 (수정됨):** `grid_to_world_coords()`가 row/col을 뒤바꿔 쓰고 `map_server`의 상하 반전(pgm의 row 0 = 이미지 맨 위 = 월드 y의 *최댓값*, 맵 원점은 좌하단 픽셀)을 반영하지 않아 goal이 엉뚱한 위치로 전송됐습니다. 테스트로 확인: 수정 전에는 개활지 맵에서도 모든 waypoint가 실패하거나 엉뚱한 곳에 도달했고, 수정 후 13~14/14가 성공했습니다. 같은 작업에서 함께 고친 것: 잘못된 goal 쿼터니언(실제 yaw 쿼터니언 대신 `orientation.z = theta`), `wait_for_result()`만 믿던 성공 판정(`ABORTED`인 goal에도 `True`를 반환함), 그리고 `wp`(waypoint, 시작점 복귀 지점 포함)가 `ori`(방향)보다 하나 길어서 생기던 `IndexError`.

- **waypoint 배치가 로봇 몸체를 무시하는 것은 버그가 아니라 논문의 설계입니다:** `FaRe/Scout_Multi_Processing.py`의 `find_frontier_cells()`는 후보 셀이 *가장 가까운* 장애물 셀에서 `buffer_distance`(기본 4셀, 해상도 0.05에서 0.2 m)만큼 떨어져 있는지만 봅니다. 풋프린트는 전혀 보지 않습니다. 논문도 이를 명시합니다 — FaRe는 "로봇의 물리적 치수를 고려하는 대신" 센서 FOV를 풋프린트로 삼고(III장), waypoint 최적화 단계에서는 "계산 복잡도를 피하려고" 맵을 아예 쓰지 않습니다(III-D).

   24/26 waffle_pi 실행에서 실측한 결과, 이 추상화가 *goal 지점에서* 치르는 대가는 예상보다 작았습니다. **26개 waypoint 전부가 풋프린트를 통과**했고(최소 0.25 m, 패딩 포함 waffle_pi가 필요한 0.175 m 대비), goal 지점의 여유공간은 실패와 성공을 구분하지 못했습니다 — *실패한* goal 하나는 1.412 m 개활지에 있었습니다. 실패는 goal이 아니라 **이동 중에** 발생하며, 그게 다음 항목의 주제입니다. `FaRe/diagnose_waypoints.py`가 이제 waypoint별과 구간별을 모두 보고합니다.

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

- **오프라인 경로가 점 로봇 기준으로 계산되던 문제 (`FaRe/traversability.py`에서 수정):** 기존 경로 탐색은 모든 자유 셀을 주행 가능으로 보고 4방향으로만 움직였기 때문에, 어떤 로봇도 들어갈 수 없는 틈을 태연히 통과했습니다. 24/26 실행의 waypoint 26개로 실측한 결과 **그 경로는 0.050 m 틈을 비집고 지나갑니다** — waffle_pi는 0.175 m가 필요합니다. `path.png`가 벤치를 관통하던 이유이자, 보고된 `path_length`가 move_base는 결코 따라가지 않을 경로를 설명하던 이유입니다.

   이제 라우팅은 `robot_radius + footprint_padding`만큼 팽창시킨 격자 위에서 이뤄집니다. `launch/costmap_override.yaml`이 move_base에 주는 것과 같은 반경입니다. 여기에 costmap_2d의 inflation 그라디언트로 셀마다 가중치를 줘서, 경계에 딱 붙는 대신 NavfnROS처럼 통로 가운데로 지나가게 했습니다. 같은 waypoint, 두 모델 비교(`results/routing_comparison.png`):

   | | 점 로봇 | 풋프린트 반영 |
   | --- | --- | --- |
   | 경로 길이 | 69.60 m | 61.52 m |
   | 사용한 가장 좁은 틈 | **0.050 m** | 0.200 m |
   | 틈 중앙값 | 0.250 m | 0.300 m |

   오히려 *짧아진* 이유는 8방향 이동이 4방향의 계단형 낭비를 없애기 때문이며, 그 이득이 가구를 우회하는 비용보다 큽니다. GRASP도 이제 벽을 무시하는 직선거리가 아니라 이 실제 주행 거리로 순서를 정합니다.

- **기하학만으로는 어떤 goal이 실패할지 여전히 예측하지 못함 (미해결):** 풋프린트를 반영해도 모든 waypoint 쌍에 주행 가능한 경로가 있고, 구간별 최협부도 결과를 가르지 못합니다. 실패한 goal 두 개는 0.200 m와 0.292 m였지만, 같은 최협부를 가진 구간들이 성공적으로 주행됐고 성공 구간의 중앙값도 0.320 m에 불과합니다. 좁다는 건 위험 요인이지 예측 변수가 아닙니다. 남은 원인은 런타임 쪽 — DWA의 궤적 탐색, AMCL 잡음, 복구 동작 — 이며, 그래서 `PatrolSim.py`는 계획이 안전하다고 믿는 대신 costmap 초기화 + 후진 복구를 유지합니다. 순찰 후 `FaRe/diagnose_waypoints.py`를 실행하면 `patrol_log.csv`의 결과를 구간별 기하와 나란히 붙여 보여줍니다.
