import mujoco
import mujoco.viewer
import os
import time
import numpy as np

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 엔코더 해상도 설정 (2048 PPR)
    ppr = 2048
    rad_per_pulse = (2 * np.pi) / ppr

    def quantize(val, step):
        return np.round(val / step) * step

    # 뷰어 실행
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()

        print("시뮬레이션 시작: 모터 엔코더 및 노이즈 시뮬레이션")
        print(f"엔코더 해상도: {ppr} PPR ({rad_per_pulse:.5f} rad/pulse)")

        while viewer.is_running():
            # 임의의 토크 입력 (사인파)
            with viewer.lock():
                data.ctrl[0] = 2.0 * np.sin(data.time * 2.0)
                data.ctrl[1] = 1.0 * np.cos(data.time * 2.0)

            # 시뮬레이션 진행
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            # 센서 데이터 비교
            gt_q1 = data.sensor("gt_pos1").data[0]       # 실제 값
            noisy_q1 = data.sensor("noisy_pos1").data[0] # 노이즈 포함 값
            enc_q1 = quantize(gt_q1, rad_per_pulse)      # 양자화(해상도 제한) 값

            # 터미널 출력: GT(Ground Truth) vs Noisy vs Enc(Quantized)
            print(f"\rT: {data.time:5.2f} | GT: {gt_q1:7.4f} | Noisy: {noisy_q1:7.4f} | Enc: {enc_q1:7.4f}", end="")

            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    main()
