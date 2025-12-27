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
    viewer.user_scn.geoms[0].label = f"X:{ee_cm[0]:.1f} Y:{ee_cm[1]:.1f} Z:{ee_cm[2]:.1f}cm"

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

def save_screenshot(model, data, path):
    """오프스크린 렌더링을 통한 스크린샷 저장"""
    try:
        renderer = mujoco.Renderer(model, height=480, width=640)
        cam = mujoco.MjvCamera()
        cam.azimuth = 135; cam.elevation = -30; cam.distance = 2.5
        cam.lookat = [0.3, 0.1, 0.2]
        renderer.update_scene(data, camera=cam)
        
        # 축 및 라벨 수동 그리기 (렌더러 씬용)
        for i, color, end_pt in zip([1, 2, 3], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
            mujoco.mjv_initGeom(renderer.scene.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.015, 0.015, 1.0],
                                pos=[0, 0, 0], mat=np.eye(3).flatten(), rgba=color)
            mujoco.mjv_connector(renderer.scene.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.01, np.array([0,0,0]), np.array(end_pt))
        renderer.scene.ngeom = 4
        
        pixels = renderer.render()
        import cv2
        cv2.imwrite(path, cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
        renderer.close()
        return True
    except Exception as e:
        print(f"\n스크린샷 저장 오류: {e}")
        return False

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # ID 설정
    radius_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "[Param] Radius")
    freq_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "[Param] Freq")
    ee_id = model.site("ee").id
    target_mocap_id = model.body("target").mocapid[0]

    # 초기값
    data.ctrl[radius_act_id] = 0.2
    data.ctrl[freq_act_id] = 0.5
    step_size = 0.5
    damping = 0.02

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        screenshot_taken = False
        start_real = time.time()
        print("-" * 60)
        print("시뮬레이션 시작: 궤적 추종 (Trajectory Tracking)")
        print(" [GUI 조작 안내]")
        print("  - 'Control' 탭에서 [Param] Radius, Freq 슬라이더를 조작하세요.")
        print("  - 로봇 팔이 원형 궤적을 실시간으로 추종합니다.")
        print("-" * 60)

        while viewer.is_running():
            with viewer.lock():
                # 1. 궤적 생성
                radius = data.ctrl[radius_act_id]
                freq = data.ctrl[freq_act_id]
                cx, cy = 0.45, 0.0
                target_x = cx + radius * np.cos(2 * np.pi * freq * data.time)
                target_y = cy + radius * np.sin(2 * np.pi * freq * data.time)
                target_z = 0.2
                data.mocap_pos[target_mocap_id] = [target_x, target_y, target_z]

                # 2. IK 계산
                curr_pos = data.site_xpos[ee_id]
                target_pos = data.mocap_pos[target_mocap_id]
                error = target_pos - curr_pos
                jacp = np.zeros((3, model.nv))
                jacr = np.zeros((3, model.nv))
                mujoco.mj_jacSite(model, data, jacp, jacr, ee_id)
                J = jacp[:, :2]
                dq = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(3)) @ error
                data.ctrl[0] = data.qpos[0] + dq[0] * step_size
                data.ctrl[1] = data.qpos[1] + dq[1] * step_size

                # 3. 비주얼 업데이트 (라벨 + 축)
                add_visual_elements(viewer, curr_pos, curr_pos * 100.0)

            # 4. 물리 시뮬레이션
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            # 5. 스크린샷 저장 (3초 시점)
            if not screenshot_taken and data.time > 3.0:
                img_path = os.path.join(current_dir, "result.png")
                if save_screenshot(model, data, img_path):
                    print("\n스크린샷 저장 완료")
                    screenshot_taken = True

            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    main()
