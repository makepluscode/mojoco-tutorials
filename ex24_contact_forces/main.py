import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()

        print("시뮬레이션 시작: 접촉력(Contact Force) 감지")
        print("  - 빨간색 푸셔가 파란색 박스를 밀면서 발생하는 힘을 측정합니다.")

        while viewer.is_running():
            # 푸셔를 박스 방향으로 이동 (주기적 왕복)
            # 0.45m를 기준으로 +/- 0.1m 사인 왕복 운동
            target_pos = 0.45 + 0.1 * np.sin(data.time * 2.0)
            
            with viewer.lock():
                data.ctrl[0] = target_pos

            # 시뮬레이션 진행
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            # 센서 데이터 읽기 (법선 방향 힘)
            force = data.sensor("contact_sensor").data[0]

            # 터미널 출력
            print(f"\rTime: {data.time:5.2f} | Target X: {target_pos:5.2f} | Contact Force: {force:7.2f} N", end="")

            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    main()
