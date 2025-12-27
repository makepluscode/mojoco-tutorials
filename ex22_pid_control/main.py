import mujoco
import mujoco.viewer
import os
import time
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # PID 컨트롤러 초기화
    pid1 = PIDController(kp=100.0, ki=0.0, kd=10.0)
    pid2 = PIDController(kp=50.0, ki=0.0, kd=5.0)

    # 목표 각도 리스트
    targets = [
        [0.0, 0.0],
        [0.5, 0.5],
        [-0.5, 1.0],
        [1.0, -0.5],
    ]
    target_idx = 0
    last_target_change = 0

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()

        print("시뮬레이션 시작: PID 위치 제어")
        print("  - 3초마다 목표 위치가 자동으로 변경됨")

        while viewer.is_running():
            step_start = time.time()

            # 목표 위치 자동 변경
            if data.time - last_target_change > 3.0:
                target_idx = (target_idx + 1) % len(targets)
                last_target_change = data.time
                print(f"목표 변경: {targets[target_idx]}")

            # PID 제어 적용
            dt = model.opt.timestep
            target = targets[target_idx]
            
            # 현재 상태 및 토크 계산
            tau1 = pid1.compute(target[0], data.qpos[0], dt)
            tau2 = pid2.compute(target[1], data.qpos[1], dt)

            # 제어 입력 적용
            with viewer.lock():
                data.ctrl[0] = tau1
                data.ctrl[1] = tau2

            # 물리 엔진 스텝 진행
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            # 터미널 상태 표시
            err1 = abs(target[0] - data.qpos[0])
            err2 = abs(target[1] - data.qpos[1])
            print(f"\rTime: {data.time:5.2f} | Tgt: {target} | Err: [{err1:6.4f}, {err2:6.4f}]", end="")

            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    main()
