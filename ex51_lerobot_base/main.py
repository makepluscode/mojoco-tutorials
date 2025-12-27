import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def add_visual_elements(viewer):
    """원점에 1m 길이의 축 화살표 추가 (Phase 표준)"""
    # 0, 1, 2: X, Y, Z 축 화살표
    for i, color, end_pt in zip([0, 1, 2], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.01, 0.01, 1.0],
                            pos=[0, 0, 0], mat=np.eye(3).flatten(), rgba=color)
        mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.01, np.array([0, 0, 0]), np.array(end_pt))
    viewer.user_scn.ngeom = 3

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 1. 패시브 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()
        
        print("-" * 60)
        print("시뮬레이션 시작: LeRobot SO-100 베이스 이해 (Base Understanding)")
        print(" [조작 안내]")
        print("  - 사이드바 슬라이더를 통해 'Rotation' 관절을 제어할 수 있습니다.")
        print("-" * 60)

        while viewer.is_running():
            step_start = time.time()

            # 2. 비주얼 업데이트 (축 표시)
            add_visual_elements(viewer)

            # 3. 물리 시뮬레이션 (현실 시간과 동기화)
            real_time = time.time() - start_real
            while data.time < real_time:
                mujoco.mj_step(model, data)

            # 4. 뷰어 동기화
            viewer.sync()
            
            # 뷰어 FPS 조절을 위한 대기 (60FPS 수준)
            elapsed = time.time() - step_start
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)

if __name__ == "__main__":
    main()
