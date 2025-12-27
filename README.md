# MuJoCo × LeRobot Tutorials

물리 엔진(MuJoCo) 기초부터 AI 로보틱스 프레임워크(LeRobot) 연동까지 단계별로 학습할 수 있는 튜토리얼 저장소입니다.

## 🚀 시작하기

### 1. 프로젝트 설치
이 프로젝트는 `uv`를 사용합니다.

```bash
# 0. uv 설치 (없을 경우)
# https://github.com/astral-sh/uv 에서 설치 안내 확인

# 1. 의존성 설치
uv sync
```

### 2. 예제 실행 방법
각 예제는 독립적인 폴더로 구성되어 있습니다. 해당 폴더로 이동하여 실행하세요.

```bash
cd ex11_freefall
uv run main.py
```

## 📚 예제 목차

### Phase 1: 물리 기초 (Physics)
* `ex11_freefall`: 중력 하에서 두 공의 낙하 비교 (탄성 차이).
* `ex12_pendulum`: 힌지 관절을 가진 진자.
* `ex13_collision_filtering`: 충돌 필터링 설정.

### Phase 2: 제어 및 센서 (Control & Sensors)
* `ex21_motor_control`: 2자유도 로봇 팔 실시간 키보드 제어.
* `ex22_pid_control`: PID 각도 추종 제어.
* `ex23_motor_encoder`: 엔코더 피드백 및 데이터 노이즈 시뮬레이션.
* `ex24_contact_forces`: 충돌 힘 계산 및 시각화.

### Phase 3: 운동학 기초 (Kinematics) [DONE]
* `ex31_fk_2dof`: 2자유도 로봇 팔의 순운동학 시각화.
* `ex32_ik_site`: 사이드바 슬라이더를 이용한 역운동학 제어.
* `ex33_trajectory_tracking`: 원형 궤적 추종 및 정밀도 분석.

### Phase 4: 비전 (Vision) [DONE]
* `ex41_camera_setup`: 다중 카메라 설정 및 단축키 시점 전환.
* `ex42_rgb_render`: 오프스크린 RGB 렌더링 및 PNG 저장.
* `ex43_depth_render`: 깊이 정보 시각화 (JET 컬러맵).

### Phase 5: LeRobot 기구 이해 (Mechanical Understanding) [IN PROGRESS]
* `ex51_lerobot_base`: 베이스 회전 동작 및 좌표계 이해.
* `ex52_lerobot_arms`: 주요 관절(Shoulder/Elbow/Wrist) 가동 범위 이해.
* `ex53_lerobot_gripper`: 그리퍼 메커니즘 및 물리 이해.

### Phase 6: LeRobot 통합 (AI Training Ready)
* `ex61_gym_env`: Gymnasium 환경 구현.
* `ex62_teleop_dataset`: 조작 데이터 녹화 및 로컬 저장.
* `ex63_policy_eval`: 정책 로드 및 평가.

---

## 🛠 규칙
* 모든 예제는 단독으로 실행 가능해야 합니다.
* 상대 경로를 사용하여 경로를 관리합니다 (`os.path.join(__file__, ...)`).
* 코드 주석은 한국어로 작성합니다.
* OOP보다는 절차지향적이고 간결한 코드를 지향합니다.
