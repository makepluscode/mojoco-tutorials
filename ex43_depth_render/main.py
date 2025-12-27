import mujoco
import mujoco.viewer
import os
import time
import numpy as np
import cv2

def add_visual_elements(viewer, curr_pos, ee_cm):
    """뷰어에 축 화살표 및 좌표 라벨 추가"""
    # EE 좌표
    mujoco.mjv_initGeom(viewer.user_scn.geoms[0], type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.01, 0, 0],
                        pos=[curr_pos[0], curr_pos[1], curr_pos[2] + 0.1], mat=np.eye(3).flatten(), rgba=[0, 0, 0, 0])
    viewer.user_scn.geoms[0].label = f"EE: {ee_cm[0]:.1f}, {ee_cm[1]:.1f}, {ee_cm[2]:.1f}cm"

    # X, Y, Z 축
    for i, color, end_pt in zip([1, 2, 3], [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        mujoco.mjv_initGeom(viewer.user_scn.geoms[i], type=mujoco.mjtGeom.mjGEOM_ARROW, size=[0.015, 0.015, 1.0],
                            pos=[0, 0, 0], mat=np.eye(3).flatten(), rgba=color)
        mujoco.mjv_connector(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, 0.01, np.array([0, 0, 0]), np.array(end_pt))
    viewer.user_scn.ngeom = 4

def main():
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # 렌더러 생성
    renderer = mujoco.Renderer(model, height=480, width=640)

    ee_id = model.site("ee").id
    target_mocap_id = model.body("target").mocapid[0]
    
    # 캡처 요청 플래그
    should_capture = False

    def key_callback(keycode):
        nonlocal should_capture
        if chr(keycode).upper() == 'S':
            should_capture = True

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        start_real = time.time()
        print("-" * 60)
        print("시뮬레이션 시작: 깊이 렌더링 (Depth Render)")
        print(" [조작 안내]")
        print("  - 'S' 키: 'Fixed View'와 'Hand View'의 깊이 맵을 저장합니다.")
        print("-" * 60)

        while viewer.is_running():
            step_start = time.time()

            with viewer.lock():
                # 1. 타겟 움직임
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
                dq = J.T @ np.linalg.inv(J @ J.T + 0.02 * np.eye(3)) @ error
                data.ctrl[0] = data.qpos[0] + dq[0] * 0.5
                data.ctrl[1] = data.qpos[1] + dq[1] * 0.5

                # 3. 비주얼 업데이트 (뷰어용)
                add_visual_elements(viewer, curr_pos, curr_pos * 100.0)

                # 4. 깊이 맵(Depth Map) 캡처 처리
                if should_capture:
                    for cam in ["Fixed View", "Hand View"]:
                        # 렌더링을 위해 깊이 옵션 활성화
                        renderer.enable_depth_rendering()
                        renderer.update_scene(data, camera=cam)
                        depth = renderer.render() # (H, W) float32 배열

                        # 0~1m 범위를 시각화하기 위해 정규화 (1.0 이상은 1.0으로 고정)
                        # MuJoCo의 depth는 실제 거리(Meter) 단위임
                        depth_copy = np.clip(depth, 0.0, 2.0)
                        
                        # 시각화를 위해 0~255 스케일로 변환 (가까우면 밝게, 멀면 어둡게)
                        # Inverse Depth: 255 * (1 - normalized_depth)
                        depth_norm = (depth_copy / 2.0 * 255).astype(np.uint8)
                        # 좀 더 직관적인 시각화를 위해 JET 컬러맵 적용
                        depth_color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_JET)

                        out_path = os.path.join(current_dir, f"depth_{cam.split()[0].lower()}.png")
                        cv2.imwrite(out_path, depth_color)
                        print(f"깊이 맵 저장 완료: {out_path}")
                        
                        renderer.disable_depth_rendering()
                        
                    should_capture = False

            # 5. 물리 시뮬레이션
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            viewer.sync()
            
            elapsed = time.time() - step_start
            if elapsed < 0.005:
                time.sleep(0.005 - elapsed)

    renderer.close()

if __name__ == "__main__":
    main()
