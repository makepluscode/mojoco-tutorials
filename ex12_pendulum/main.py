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

    # 초기 각도 설정 (약 90도)
    data.qpos[0] = 1.57
    mujoco.mj_forward(model, data)

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 스크린샷용 렌더러
        renderer = mujoco.Renderer(model, height=480, width=640)
        
        start_real = time.time()
        screenshot_taken = False

        while viewer.is_running():
            step_start = time.time()

            # 시뮬레이션 시간과 실제 시간 동기화 루프
            # 실제 흐른 시간만큼 물리 스텝 진행
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

                # 10초 주기 리셋
                if data.time > 10.0:
                    mujoco.mj_resetData(model, data)
                    data.qpos[0] = 1.57
                    mujoco.mj_forward(model, data)
                    start_real = time.time()
                    break

            # 스크린샷 캡처 (0.5초 시점)
            if not screenshot_taken and data.time > 0.5:
                vopt = mujoco.MjvOption()
                renderer.update_scene(data, scene_option=vopt)
                pixels = renderer.render()
                import cv2
                cv2.imwrite(os.path.join(current_dir, "result.png"), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
                screenshot_taken = True

            # 화면 갱신
            viewer.sync()

            # CPU 부하 감소 및 높은 프레임률 유지를 위한 미세 대기
            time.sleep(0.001)

if __name__ == "__main__":
    main()
