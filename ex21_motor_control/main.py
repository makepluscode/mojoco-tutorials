import mujoco
import mujoco.viewer
import os
import time
from pynput import keyboard

# 전역 변수: 제어 상태 및 수치
ctrl_torque = [0.0, 0.0]
torque_step = 0.5

def on_press(key):
    global ctrl_torque
    try:
        # 단축키: 숫자키 사용 (MuJoCo 기본 단축키와 충돌 방지)
        if key.char == '1': # 어깨 +
            ctrl_torque[0] += torque_step
        elif key.char == '2': # 어깨 -
            ctrl_torque[0] -= torque_step
        elif key.char == '3': # 꿈치 +
            ctrl_torque[1] += torque_step
        elif key.char == '4': # 꿈치 -
            ctrl_torque[1] -= torque_step
    except AttributeError:
        # 특수 키: Space (초기화)
        if key == keyboard.Key.space:
            ctrl_torque[0] = 0.0
            ctrl_torque[1] = 0.0

def main():
    # 경로 설정
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, "scene.xml")

    # 모델 및 데이터 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 키보드 리스너 시작
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # 뷰어 실행 (Passive)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_real = time.time()

        print("시뮬레이션 시작 (Ctrl+C 또는 ESC로 종료)")
        print("제어 방법:")
        print("  1 / 2 : 1번 관절(어깨) +/- 토크")
        print("  3 / 4 : 2번 관절(꿈치) +/- 토크")
        print("  Space : 모든 제어 입력 초기화")
        print("\n[TIP] 뷰어의 'Sensor' 또는 'Watch' 탭에서 실시간 토크값을 확인하세요.")
        print("-" * 50)

        while viewer.is_running():
            # 현재 토크값 적용
            with viewer.lock():
                data.ctrl[0] = ctrl_torque[0]
                data.ctrl[1] = ctrl_torque[1]

            # 실제 시간 동기화 (리셋 로직 제거)
            now = time.time()
            while data.time < (now - start_real):
                mujoco.mj_step(model, data)

            # 터미널 상태 표시 (옵션)
            print(f"\rTorque - J1: {ctrl_torque[0]:6.2f} | J2: {ctrl_torque[1]:6.2f}", end="")

            # 뷰어 동기화
            viewer.sync()
            
            # CPU 점유율 조절
            time.sleep(0.001)

    listener.stop()

if __name__ == "__main__":
    main()
