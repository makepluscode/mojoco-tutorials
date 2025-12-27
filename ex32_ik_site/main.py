import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def add_arrow_axes(viewer):
    """뷰어에 X, Y, Z 축 화살표 추가"""
    # X축 (빨강)
    idx = 1 # 0번은 EE 라벨용
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[idx],
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=[0.01, 0.01, 1.0],
        pos=[0, 0, 0],
        mat=np.eye(3).flatten(),
        rgba=[1, 0, 0, 1]
    )
    mujoco.mjv_connector(viewer.user_scn.geoms[idx], mujoco.mjtGeom.mjGEOM_ARROW, 
                        0.01, np.array([0, 0, 0]), np.array([1.0, 0, 0]))
    
    # Y축 (초록)
    idx += 1
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[idx],
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=[0.01, 0.01, 1.0],
        pos=[0, 0, 0],
        mat=np.eye(3).flatten(),
        rgba=[0, 1, 0, 1]
    )
    mujoco.mjv_connector(viewer.user_scn.geoms[idx], mujoco.mjtGeom.mjGEOM_ARROW, 
                        0.01, np.array([0, 0, 0]), np.array([0, 1.0, 0]))

    # Z축 (파랑)
    idx += 1
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[idx],
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=[0.01, 0.01, 1.0],
        pos=[0, 0, 0],
        mat=np.eye(3).flatten(),
        rgba=[0, 0, 1, 1]
    )
    mujoco.mjv_connector(viewer.user_scn.geoms[idx], mujoco.mjtGeom.mjGEOM_ARROW, 
                        0.01, np.array([0, 0, 0]), np.array([0, 0, 1.0]))
    
    viewer.user_scn.ngeom = idx + 1

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # ID 설정
    ee_id = model.site("ee").id
    target_site_id = model.site("target_site").id

    # IK 파라미터
    step_size = 0.5
    damping = 0.01

    # 초기 타겟 위치 설정 (X=0.5, Y=0.5)
    data.ctrl[model.actuator("[Tgt] X").id] = 0.5
    data.ctrl[model.actuator("[Tgt] Y").id] = 0.5

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()

        print("-" * 60)
        print("시뮬레이션 시작: 역운동학(IK) 구현")
        print(" [GUI 조작 안내]")
        print("  1. 'Control' 탭에서 X, Y, Z 슬라이더로 타겟을 움직이세요.")
        print("  2. 원점의 [빨강:X, 초록:Y, 파랑:Z] 화살표 축을 확인하세요.")
        print("-" * 60)

        while viewer.is_running():
            with viewer.lock():
                # 1. IK 계산
                curr_pos = data.site_xpos[ee_id]
                target_pos = data.site_xpos[target_site_id]
                error = target_pos - curr_pos
                
                jacp = np.zeros((3, model.nv))
                jacr = np.zeros((3, model.nv))
                mujoco.mj_jacSite(model, data, jacp, jacr, ee_id)
                J = jacp[:, :2]
                
                # DLS 기반 dq 계산
                dq = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(3)) @ error
                
                # 제어 입력 업데이트
                data.ctrl[model.actuator("arm_pos1").id] = data.qpos[0] + dq[0] * step_size
                data.ctrl[model.actuator("arm_pos2").id] = data.qpos[1] + dq[1] * step_size

                # 2. 3D 시각화 (좌표 라벨 + 화살표 축)
                ee_cm = curr_pos * 100.0
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[0],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.01, 0, 0],
                    pos=[curr_pos[0], curr_pos[1], curr_pos[2] + 0.1], 
                    mat=np.eye(3).flatten(),
                    rgba=[0, 0, 0, 0]
                )
                viewer.user_scn.geoms[0].label = f"X:{ee_cm[0]:.1f} Y:{ee_cm[1]:.1f} Z:{ee_cm[2]:.1f}cm"
                
                # 화살표 축 추가
                add_arrow_axes(viewer)

            # 3. 물리 시뮬레이션
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            dist = np.linalg.norm(error)
            print(f"\rTime: {data.time:5.2f} | Dist Error: {dist:6.4f} m", end="")

            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    main()
