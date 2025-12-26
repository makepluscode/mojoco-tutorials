import mujoco
import mujoco.viewer
import os
import time

def main():
    # 현재 스크립트 디렉토리 경로 획득
    current_dir = os.path.dirname(__file__)
    # XML 파일 경로 설정
    xml_path = os.path.join(current_dir, "scene.xml")

    # MuJoCo 모델 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    # 시뮬레이션 데이터 객체 생성
    data = mujoco.MjData(model)

    # 시각화 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:

        # 뷰어 실행 중 루프 반복
        while viewer.is_running():
            step_start = time.time()

            # 물리 시뮬레이션 한 단계 진행
            mujoco.mj_step(model, data)

            # 10초마다 시뮬레이션 초기화 (반복)
            if data.time > 10.0:
                mujoco.mj_resetData(model, data)

            # 뷰어 동기화
            viewer.sync()

            # 실제 시간과 시뮬레이션 속도 동기화
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
