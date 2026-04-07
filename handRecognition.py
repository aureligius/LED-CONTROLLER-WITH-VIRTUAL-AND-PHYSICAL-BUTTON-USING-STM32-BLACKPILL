import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import serial
import os
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# CH340 PORT
COM_PORT = 'COM7'

try:
    bt_serial = serial.Serial(COM_PORT, 9600, timeout=1)
    print(f"Connected to STM32 on {COM_PORT}")
except Exception as e:
    print(f"Connection failed: {e}. Running in simulation mode.")
    bt_serial = None

lock           = threading.Lock()
latest_fingers = 0
last_mode      = -1
last_sent      = 0
DEBOUNCE_SEC   = 1.5

def count_fingers(hand):
    fingers = []

    # 4 fingers using correct pip landmarks
    finger_pairs = [(8,6), (12,10), (16,14), (20,18)]
    for tip, pip in finger_pairs:
        fingers.append(1 if hand[tip].y < hand[pip].y else 0)

    # Thumb
    fingers.append(1 if hand[4].x < hand[2].x else 0)

    return sum(fingers)

def on_result(result, output_image, timestamp_ms):
    global latest_fingers, last_mode, last_sent

    fingers_up = 0
    if result.hand_landmarks:
        for hand in result.hand_landmarks:
            fingers_up = count_fingers(hand)

    with lock:
        latest_fingers = fingers_up

    # Only send 1-4 (your 4 modes), debounced, only on change
    now = time.time()
    if (fingers_up in [1, 2, 3, 4]
            and fingers_up != last_mode
            and (now - last_sent) > DEBOUNCE_SEC
            and bt_serial):
        try:
            bt_serial.write(str(fingers_up).encode())
            print(f"Sent Mode {fingers_up} → STM32")
            last_sent = now
            last_mode = fingers_up
        except Exception as e:
            print(f"BT send failed: {e}")

BaseOptions           = mp.tasks.BaseOptions
HandLandmarker        = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode     = mp.tasks.vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(
        model_asset_path=os.path.join(BASE_DIR, 'hand_landmarker.task')
    ),
    running_mode=VisionRunningMode.LIVE_STREAM,
    num_hands=1,
    min_hand_detection_confidence=0.5,
    min_hand_presence_confidence=0.5,
    min_tracking_confidence=0.5,
    result_callback=on_result
)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

cv2.namedWindow('Hand Tracking', cv2.WINDOW_NORMAL)
frame_count = 0

# Mode name display
mode_names = {
    -1: 'None',
    1:  'Mode 1 — Light Chaser',
    2:  'Mode 2 — Counter',
    3:  'Mode 3 — ADC',
    4:  'Mode 4 — Jingle Bells',
    5:  'Mode 5 — Interrupt'
}

with HandLandmarker.create_from_options(options) as landmarker:
    while True:
        success, img = cap.read()
        if not success:
            continue

        frame_count += 1
        img = cv2.flip(img, 1)

        # Send every frame — no skip, full size for best detection
        rgb    = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        landmarker.detect_async(mp_img, frame_count)

        with lock:
            display_fingers = latest_fingers

        # --- UI ---
        cv2.putText(img, f'Fingers : {display_fingers}',
                    (10, 50),  cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
        cv2.putText(img, f'Mode    : {mode_names[last_mode]}',
                    (10, 90),  cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0),   2)
        cv2.putText(img, 'Q = quit',
                    (10, 130), cv2.FONT_HERSHEY_PLAIN, 1, (200, 200, 200), 1)

        # Connection status
        status_color = (0, 255, 0) if bt_serial else (0, 0, 255)
        status_text  = f'BT: {COM_PORT} OK' if bt_serial else 'BT: NOT CONNECTED'
        cv2.putText(img, status_text,
                    (10, 170), cv2.FONT_HERSHEY_PLAIN, 1, status_color, 1)

        cv2.imshow('Hand Tracking', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.getWindowProperty('Hand Tracking', cv2.WND_PROP_VISIBLE) < 1:
            break

cap.release()
cv2.destroyAllWindows()
if bt_serial:
    bt_serial.close()
print("Exited cleanly.")