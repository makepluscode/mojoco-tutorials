import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def forward_kinematics(q, l1, l2):
    """
    2자유도 평면 로봇 팔의 순운동학(FK) 계산
    q: [q1, q2] 관절 각도 (라디안)
    l1: 0.5m, l2: 0.4m
    """
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    l1, l2 = 0.5, 0.4

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 초기 시각화 옵션: 사이트 라벨 활성화
        with viewer.lock():
            viewer.opt.label = mujoco.mjtLabel.mjLABEL_SITE

        start_real = time.time()

        print("-" * 60)
        print("시뮬레이션 시작: 순운동학(FK) 계산 (cm 단위 표시)")
        print(" [GUI 안내]")
        print("  - 로봇 끝단 옆에 'X, Y, Z' 좌표가 cm 단위로 표시됩니다.")
        print("-" * 60)

        while viewer.is_running():
            # 제어 입력
            with viewer.lock():
                data.ctrl[0] = 1.0 * np.sin(data.time)
                data.ctrl[1] = 0.5 * np.cos(data.time * 2.0)

            # 시뮬레이션 진행
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            # 1. FK 계산 (m) 및 Z값 획득 -> cm 변환
            calc_x, calc_y = forward_kinematics(data.qpos[:2], l1, l2)
            ee_id = model.site("ee").id
            ee_pos = data.site_xpos[ee_id]
            calc_z = ee_pos[2]

            calc_x_cm = calc_x * 100.0
            calc_y_cm = calc_y * 100.0
            calc_z_cm = calc_z * 100.0
            label_str = f"X:{calc_x_cm:5.1f} Y:{calc_y_cm:5.1f} Z:{calc_z_cm:5.1f}cm"

            # 2. 뷰어 및 데이터 업데이트
            with viewer.lock():
                # 센서 데이터 주입 (사이드바 Watch 창용)
                data.sensordata[model.sensor('calc_x_cm').adr] = calc_x_cm
                data.sensordata[model.sensor('calc_y_cm').adr] = calc_y_cm
                data.sensordata[model.sensor('calc_z_cm').adr] = calc_z_cm

                # 뷰어 3D 화면에 실시간 좌표 라벨 표시
                viewer.user_scn.ngeom = 1
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[0],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.01, 0, 0],
                    pos=[ee_pos[0], ee_pos[1], ee_pos[2] + 0.1], 
                    mat=np.eye(3).flatten(),
                    rgba=[0, 0, 0, 0] # 투명 배경
                )
                viewer.user_scn.geoms[0].label = label_str

            # 터미널 상태 표시
            print(f"\rTime: {data.time:5.2f} | Calc (cm): ({calc_x_cm:5.1f}, {calc_y_cm:5.1f}, {calc_z_cm:5.1f})", end="")

            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    main()
