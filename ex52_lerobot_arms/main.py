import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def add_visual_elements(viewer, model, data):
    """원점 및 로봇 손끝(Hand)에 좌표축 화살표 추가"""
    # 0, 1, 2: 원점 축 (1m)
    for i, color, end_pt in zip([0, 1, 2], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.01, 0.01, 1.0],
                            pos=[0, 0, 0], mat=np.eye(3).flatten(), rgba=color)
        mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.01, np.array([0, 0, 0]), np.array(end_pt))
    
    # 3, 4, 5: 손끝(hand_axis site) 축 (0.1m)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hand_axis")
    if site_id != -1:
        site_pos = data.site_xpos[site_id]
        site_mat = data.site_xmat[site_id].reshape(3, 3)
        for i, color in zip([3, 4, 5], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]):
            axis_dir = site_mat[:, i-3] * 0.1
            mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.002, 0.002, 0.1],
                                pos=site_pos, mat=np.eye(3).flatten(), rgba=color)
            mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.002, site_pos, site_pos + axis_dir)
        viewer.user_scn.ngeom = 6
    else:
        viewer.user_scn.ngeom = 3

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 초기 자세 설정 (home keyframe)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    # 1. 패시브 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()
        
        print("-" * 60)
        print("시뮬레이션 시작: LeRobot SO-100 5-DOF 팔 구조 이해 (Arm Structure)")
        print(" [조작 안내]")
        print("  - 사이드바 슬라이더를 통해 각 관절(Rotation, Pitch, Elbow, ...)을 제어하세요.")
        print("  - 'home' 자세로 초기화되어 시작됩니다.")
        print("-" * 60)

        while viewer.is_running():
            step_start = time.time()

            # 2. 비주얼 업데이트 (축 표시)
            add_visual_elements(viewer, model, data)

            # 3. 물리 시뮬레이션 (현실 시간 동기화)
            real_time = time.time() - start_real
            while data.time < real_time:
                mujoco.mj_step(model, data)

            # 4. 뷰어 동기화
            viewer.sync()
            
            # FPS 조절 (~60FPS)
            elapsed = time.time() - step_start
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)

if __name__ == "__main__":
    main()
