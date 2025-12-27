# PRD: MuJoCo × LeRobot Tutorials (Agent-Ready Spec)

## 1. 개요 (Objective)

물리 엔진(MuJoCo) 기초부터 AI 로보틱스 프레임워크(LeRobot) 연동까지 단계별로 학습할 수 있는 독립 실행형 튜토리얼 저장소를 구축한다.

## 2. 핵심 규칙 (Core Rules)

1. **독립성:** 모든 `exXX_name` 폴더는 자체 `main.py`와 `*.xml`을 포함하며, 단독으로 실행 가능해야 한다.
2. **실행 방식:** 모든 예제는 `cd 폴더 && uv run main.py`로 실행한다.
3. **경로 관리:** `os.path.join` 및 `__file__` 기반 상대 경로를 사용한다. 절대 경로 금지.
4. **언어:** 코드 주석은 한국어로 작성하며, 명사형이나 간결한 문장으로 마무리한다. (예: '~함', '~임', '~함 설정' 등. '합니다'와 같은 경어체 금지)
5. **스타일:** OOP 지양, 절차지향적이고 간결한 코드 유지. 린트(Lint)가 적용된 깨끗한 코드. **가독성을 위한 빈 줄은 허용하되, 빈 줄의 화이트스페이스는 절대 금지.**
6. **문서화:** 각 예제 폴더에는 `README.md`와 실행 화면 캡처(`result.png`)가 반드시 포함되어야 한다.
7. **시각화 표준:**
    - 원점에 **1m 길이의 화살표 축** (빨강:X, 초록:Y, 파랑:Z) 표시 필수.
    - 주요 수치(예: EE 위치)는 **cm 단위**로 뷰어 내 3D 텍스트로 실시간 표시.
    - 상호작용은 'mocap' 드래그보다 **사이드바 슬라이더(actuator)**를 우선적으로 사용함.

## 3. 기술 스택 (Tech Stack)

* **Language:** Python 3.10
* **Manager:** `uv` (pyproject.toml 기반)
* **Libraries:** `mujoco`, `gymnasium`, `lerobot`, `numpy`, `opencv-python`, `matplotlib`, `torch`

---

## 4. 단계별 구현 명세 (Execution Tasks)

### Phase 0: 프로젝트 루트 구성 (Bootstrap)

* **Task 0.1:** `.python-version` 파일 생성 (내용: `3.10`)
* **Task 0.2:** `pyproject.toml` 생성 (필수 의존성: `mujoco`, `gymnasium`, `lerobot`, `numpy`, `opencv-python`, `matplotlib`, `torch`)
* **Task 0.3:** `.gitignore` 생성 (Python 표준 + `.venv`, `__pycache__`, `*.mp4`, `*.png`)
* **Task 0.4:** 루트 `README.md` 작성 (설치 방법 및 단계별 예제 목차 포함)

### Phase 1: 물리 기초 (Physics)

* **ex11_freefall:** 중력 하에서 두 공의 낙하 비교. `solref` 값을 다르게 설정하여 탄성 차이 구현.
* **ex12_pendulum:** 힌지 관절을 가진 진자. 초기 각도 설정 및 댐핑(`damping`) 파라미터 적용.
* **ex13_collision_filtering:** `contype`, `conaffinity`를 이용한 충돌 필터링. 특정 물체끼리만 충돌하도록 설정.

### Phase 2: 제어 및 센서 (Control & Sensors)

* **ex21_motor_control:** 2-DOF 로봇 암 키보드 제어.
* **ex22_pid_control:** PID 제어기를 이용한 위치 제어.
* **ex23_motor_encoder:** 엔코더 피드백 및 데이터 노이즈 시뮬레이션.
* **ex24_contact_forces:** 접촉력 감지 및 그리퍼 기초.

### Phase 3: 운동학 기초 (Kinematics) [DONE]

* **ex31_fk_2dof:** 2자유도 로봇 팔의 순운동학(FK). 각도에 따른 End-effector 위치 계산 및 시각화.
* **ex32_ik_site:** 역운동학(IK) 기초. `mocap` 바디나 `site`를 활용하여 목표 위치를 추종하는 IK 구현.
* **ex33_trajectory_tracking:** 원형 또는 사각형 궤적 추종. 로봇 팔 끝단이 지정된 경로를 따라 움직이도록 제어.

### Phase 4: 비전 (Vision) [DONE]

* **ex41_camera_setup:** 다중 카메라 정의 및 단축키 시점 전환.
* **ex42_rgb_render:** 오프스크린 RGB 이미지 획득 및 PNG 저장.
* **ex43_depth_render:** 깊이 정보 시각화 및 컬러맵 적용.

### Phase 5: LeRobot 기구 이해 (Mechanical Understanding) [IN PROGRESS]

* **ex51_lerobot_base:** SO-100 베이스 회전 메커니즘 및 로컬 좌표계 이해.
* **ex52_lerobot_arms:** 어깨, 팔꿈치, 손목 등 주요 조인트의 틸트 동작 및 가동 범위(Limit) 설정.
* **ex53_lerobot_gripper:** 그리퍼의 물리적 구조(Parallel jaw/Geared) 및 물체 파지(Grasping) 시뮬레이션.

### Phase 6: LeRobot 통합 (AI Training Ready)

* **ex61_gym_env:** `gymnasium.Env` 상속 클래스 구현. (Observation: RGB + Joint, Action: Torque). LeRobot 규격 준수.
* **ex62_teleop_dataset:** 조작 데이터를 `LeRobotDataset` 형식으로 10 에피소드 이상 녹화 및 로컬 저장.
* **ex63_policy_eval:** 저장된/학습된 정책(`Policy`)을 로드하여 환경에서 실행하고 성공률 측정.

---

## 5. 폴더 구조 (Directory Layout)

```text
mujoco-tutorials/
├── pyproject.toml
├── README.md
├── ex11_freefall/
│   ├── main.py
│   └── scene.xml
├── ex12_pendulum/
│   ├── main.py
│   └── pendulum.xml
... (중략) ...
└── ex43_policy_eval/
    └── main.py

```

## 6. 에이전트 행동 가이드 (Final Instruction)

1. **순차적 구현:** Phase 0부터 4까지 순서대로 구현할 것.
2. **검증:** 각 폴더 구현 직후 `uv run main.py`가 에러 없이 작동하는지 확인할 것.
3. **간결성:** 불필요한 추상화(Class Overuse)를 피하고, 독자가 코드를 읽고 바로 이해할 수 있게 할 것.
