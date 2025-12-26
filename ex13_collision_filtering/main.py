import mujoco
import mujoco.viewer
import os
import time

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 초기 상태 설정: 중앙 충돌 및 탄성 비교 유도
    def reset_state():
        mujoco.mj_resetData(model, data)

        # 빨간 공 그룹 (X축 이동, 탄성 있음)
        data.qpos[0:3] = [-1.5, 0, 0.2] # a1
        data.qvel[0:3] = [3.0, 0, 0]
        data.qpos[7:10] = [1.5, 0, 0.2] # a2
        data.qvel[6:9] = [-3.0, 0, 0]

        # 파란 공 그룹 (Y축 이동, 탄성 없음)
        data.qpos[14:17] = [0, -1.5, 0.2] # b1
        data.qvel[12:15] = [0, 3.0, 0]
        data.qpos[21:24] = [0, 1.5, 0.2]  # b2
        data.qvel[18:21] = [0, -3.0, 0]

        mujoco.mj_forward(model, data)

    reset_state()

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()

        while viewer.is_running():
            step_start = time.time()

            # 실제 시간 동기화
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

                # 3초마다 리셋
                if data.time > 3.0:
                    reset_state()
                    start_real = time.time()
                    break

            # 뷰어 동기화
            viewer.sync()

            # CPU 대기
            time.sleep(0.001)

if __name__ == "__main__":
    main()
