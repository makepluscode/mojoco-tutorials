import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def add_visual_elements(viewer, curr_pos, ee_cm):
    """뷰어에 축 화살표(1m) 및 좌표 라벨 추가"""
    # 0: EE 좌표 라벨
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.01, 0, 0],
        pos=[curr_pos[0], curr_pos[1], curr_pos[2] + 0.1], 
        mat=np.eye(3).flatten(),
        rgba=[0, 0, 0, 0]
    )
    viewer.user_scn.geoms[0].label = f"EE: {ee_cm[0]:.1f}, {ee_cm[1]:.1f}, {ee_cm[2]:.1f}cm"

    # 1, 2, 3: X, Y, Z 축 화살표
    for i, color, end_pt in zip([1, 2, 3], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[i],
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            size=[0.015, 0.015, 1.0],
            pos=[0, 0, 0],
            mat=np.eye(3).flatten(),
            rgba=color
        )
        mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 
                            0.01, np.array([0, 0, 0]), np.array(end_pt))
    
    viewer.user_scn.ngeom = 4

def main():
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    ee_id = model.site("ee").id
    target_mocap_id = model.body("target").mocapid[0]
    step_size = 0.5
    damping = 0.02

    # 키보드 콜백 정의
    def key_callback(keycode):
        if chr(keycode) == '1':
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            print("View: Free View")
        elif chr(keycode) == '2':
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = 0
            print("View: Fixed View")
        elif chr(keycode) == '3':
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = 1
            print("View: Top View")
        elif chr(keycode) == '4':
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = 2
            print("View: Hand View")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        start_real = time.time()
        print("-" * 60)
        print("시뮬레이션 시작: 카메라 설정 (Camera Setup)")
        print(" [키보드 조작 안내]")
        print("  - '1': Free View (자유 시점)")
        print("  - '2': Fixed View (측면 고정 시점)")
        print("  - '3': Top View (상단 조감도)")
        print("  - '4': Hand View (로봇 팔 1인칭 시점)")
        print("-" * 60)

        while viewer.is_running():
            with viewer.lock():
                # 1. 타켓 움직임
                target_x = 0.45 + 0.2 * np.cos(data.time * 0.5)
                target_y = 0.2 * np.sin(data.time * 0.5)
                target_z = 0.2
                data.mocap_pos[target_mocap_id] = [target_x, target_y, target_z]

                # 2. IK 계산
                curr_pos = data.site_xpos[ee_id]
                error = data.mocap_pos[target_mocap_id] - curr_pos
                jacp = np.zeros((3, model.nv))
                jacr = np.zeros((3, model.nv))
                mujoco.mj_jacSite(model, data, jacp, jacr, ee_id)
                J = jacp[:, :2]
                dq = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(3)) @ error
                data.ctrl[0] = data.qpos[0] + dq[0] * step_size
                data.ctrl[1] = data.qpos[1] + dq[1] * step_size

                # 3. 비주얼 업데이트 (심플 축/라벨)
                add_visual_elements(viewer, curr_pos, curr_pos * 100.0)

            # 4. 시뮬레이션 스텝
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(0.005)

if __name__ == "__main__":
    main()
