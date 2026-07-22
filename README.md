# 2025 성균관대학교 자율주행 해커톤

ROS Noetic 기반 시뮬레이터에서 경로 추종, 미션 상태 전환, 주차 제어를 구현한 팀 프로젝트입니다. 이 저장소는 [`jooeonjohn/2025_HEVEN_Hackathon`](https://github.com/jooeonjohn/2025_HEVEN_Hackathon)의 fork이며, 팀 전체 코드와 제 개인 기여가 함께 있습니다.

> **개인 기여 요약**
> Git 이력으로 확인되는 제 구현은 `brain_final.py`의 전면·평행 주차 상태 머신과 adaptive pure pursuit 통합, `fsds_final.py`의 racing-line 최적화·곡률 기반 속도 제어·RViz 디버그 시각화입니다. 두 파일은 기존 팀/대회 실행 환경을 바탕으로 만든 통합본이므로 팀 전체 시스템을 개인 성과로 주장하지 않습니다.

> **대회 결과**
> 4인 팀으로 최종 **2위**를 수상했습니다. 대회 주행에서는 3개 주차 미션 중 전면·평행 주차 2개를 완료했고, 일반 주행 구간의 목표 속도를 높이는 전략을 함께 적용했습니다. 수상 결과는 2025-11-22 발급 상장으로 확인했습니다.

| 문제 | 구현 | 확인 가능한 결과 |
|---|---|---|
| 주차 동작을 순차 제어해야 함 | 전면·평행 주차를 단계별 FSM으로 구성하고 위치·자세 오차와 타임아웃으로 전환 | 주차 미션 3개 중 2개 완료 |
| 제한된 경기 시간 안에 주행 속도와 안정성을 함께 확보해야 함 | 일반 구간은 빠르게 주행하고 조향량이 큰 구간은 감속하도록 목표 속도를 구간별 조정 | 최종 2위 수상에 기여한 팀 전략 |
| 제어 상태를 빠르게 확인하기 어려움 | 목표점, 경계, racing line, 조향·속도 정보를 RViz marker로 게시 | 디버그 publisher와 marker 코드 확인 가능 |

정량 랩타임과 완주율, 속도 전략 단독의 성능 개선 폭은 재현 가능한 로그가 없어 기재하지 않았습니다.

## 프로젝트 개요

- 대회: 2025 성균관대학교 자율주행 해커톤
- 일정: 2025-11-21 17:00 ~ 2025-11-22 10:00
- 형태: 4인 팀 프로젝트, 시뮬레이션 기반 자율주행 알고리즘 구현
- 결과: 최종 2위, 주차 미션 3개 중 2개 완료
- 환경: Ubuntu 20.04, ROS Noetic, Python, RViz
- 주요 인터페이스: ROS topic, Ackermann steering command, FSDS message

## 시스템 구조

```text
pose / lidar / cone / velocity
            │
            ▼
 path·mission processing
   ├─ global/parking path
   ├─ mission FSM
   └─ racing-line candidate
            │
            ▼
 path tracking & speed control
   ├─ pure pursuit
   ├─ curvature-based target speed
   └─ steering/throttle/brake command
            │
            ├─► vehicle/simulator
            └─► RViz debug markers
```

## My Contribution

Git author가 이 계정 소유자와 일치하는 것으로 확인된 커밋만 근거로 정리했습니다.

### 1. 주차 제어와 상태 머신

- [`a5c79ac`](https://github.com/choiYhunn/skku-autonomous-driving-hackathon-2025/commit/a5c79acdb57625928c2b94e1f68c199fe08fbe47): `brain_final.py` 추가
- 전면 주차와 평행 주차를 단계별 FSM으로 구성
- 위치·yaw 오차를 이용한 보정 동작과 최대 반복 횟수 적용
- 주차 경로에서 pure pursuit를 사용하고 정지·대기·후진 단계를 명시적으로 분리
- `BrainLogger`를 연결해 제어 상태 기록

### 2. 경로 생성·추종과 디버그 도구

- [`c2c9fb7`](https://github.com/choiYhunn/skku-autonomous-driving-hackathon-2025/commit/c2c9fb76da8c2955963839435fbe0bbbbacc222e): `fsds_final.py` 추가
- cone boundary에서 중심 경로를 구성하고 선택적으로 racing line을 최적화
- 곡률과 현재 속도를 사용해 look-ahead와 목표 속도를 조정
- pure pursuit 기반 조향과 throttle/brake 명령 생성
- 목표점, 좌우 경계, racing line, 조향 화살표, 제어 정보를 RViz marker로 게시

### 3. 저장소 정리

- [`d284aac`](https://github.com/choiYhunn/skku-autonomous-driving-hackathon-2025/commit/d284aac3d8d3b03878a3adc9ec3e62cc523bb6eb): `.gitignore` 추가 및 Python cache 제거

## 해결한 기술 문제와 구현 방법

### 주차 단계의 불명확한 전환

주차 로직을 하나의 연속 제어식으로 두지 않고 접근, 정렬, 정지, 보정, 복귀 단계로 분리했습니다. 각 단계는 위치·자세 임계값 또는 경과 시간으로 전환하며, 보정 반복 횟수에 상한을 두었습니다.

### 곡선에서의 경로 추종과 속도 결정

경로 곡률과 속도에 따라 look-ahead 및 목표 속도를 계산하고, pure pursuit의 조향 결과와 함께 throttle/brake를 결정했습니다. racing-line 최적화는 기본값에서 비활성화되어 있으며 실제 사용 시 계산 비용과 안정성을 별도로 검증해야 합니다.

### 디버깅 가시성

제어 입력만 확인하는 대신 목표점, 경계, 경로, 조향 방향과 제어 상태를 별도 ROS topic으로 게시하도록 구성했습니다. 이를 통해 경로 생성 문제와 추종 문제를 RViz에서 구분할 수 있게 했습니다.

## 테스트 및 결과

- Git 이력으로 두 구현 파일의 추가와 기능 범위를 확인했습니다.
- 2025-11-22 발급 상장에서 4인 팀의 최종 2위를 확인했습니다. 상장 이미지는 팀원 개인정보가 있어 저장소에 포함하지 않았습니다.
- 대회 주행에서 전면·평행 주차 미션 2개를 완료했습니다. `brain_final.py`에는 대응하는 P1/P3 FSM과 웨이포인트가 남아 있습니다.
- 일반 구간 목표 속도 `8.0`, 조향량 기반 감속 구간 등 고속 주행 전략이 코드에 남아 있습니다.
- 저장소에는 재현 가능한 자동 테스트, 완주율, 랩타임 비교 로그가 없습니다.
- 시뮬레이션 실행 화면과 설치 이미지는 팀 저장소 자산이며 개인 구현의 정량 검증 자료로 사용하지 않았습니다.
- 따라서 속도 전략의 단독 성능 향상 폭이나 구간별 랩타임은 주장하지 않습니다.

## 실행 방법

원본 팀 환경 전체가 필요합니다. 아래 명령은 저장소에 기록된 실행 절차이며, 현재 환경에서 재검증하지 않았습니다.

```bash
sudo apt-get install ros-noetic-tf2-geometry-msgs \
  ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server

cd ~/catkin_ws
catkin_make
source devel/setup.bash

cd ~/catkin_ws/src/2025_HEVEN_Hackathon/Simulation
bash simulate.sh <map-number>
bash brain.sh
```

Formula/FSDS 제어 노드는 다음 launch 구성을 사용합니다.

```bash
roslaunch formula control.launch
```

## 주요 파일

```text
brain_final.py                  # 개인 커밋: 주차 FSM 및 경로 추종 통합본
fsds_final.py                   # 개인 커밋: racing-line·속도 제어·디버그 통합본
Simulation/                    # 팀 시뮬레이터와 미션 환경
Formula/                       # 팀/대회 Formula·FSDS 환경
README.md                      # 개인 기여와 제한을 명시한 포트폴리오 설명
```

## 한계

- `brain_final.py`와 `fsds_final.py`는 팀/대회 실행 환경의 메시지, 패키지, baseline 구조에 의존합니다.
- 커밋 작성자는 확인했지만 각 파일의 모든 기반 코드가 개인 원저작물이라는 뜻은 아닙니다.
- 자동 테스트와 정량 로그가 없어 제어 안정성이나 성능 개선 폭을 주장할 수 없습니다.
- 센서 노이즈, 실차 지연, actuator saturation을 검증한 프로젝트가 아닙니다.

## Attribution 및 공개 범위

- 원본 팀 저장소: [`jooeonjohn/2025_HEVEN_Hackathon`](https://github.com/jooeonjohn/2025_HEVEN_Hackathon)
- 이 저장소는 원본의 fork이며 다수 팀원의 커밋과 대회 제공 환경을 포함합니다.
- 개인 기여 범위는 위의 세 커밋으로 한정해 설명했습니다.
- 저장소에 별도 라이선스가 확인되지 않습니다. 이 README는 팀 또는 제3자 코드에 새로운 사용 허가를 부여하지 않습니다.
