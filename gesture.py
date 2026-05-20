# =============================================================
# GESTURE ARM CONTROL → FPGA → ROBOTIC ARM
# =============================================================
# Maps real hand/arm movements to robotic arm servo positions
# Uses MediaPipe pose (for elbow) + hand (for wrist & grip)
# =============================================================

import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
import threading
import math
from collections import deque
from enum import Enum, auto

# ── MediaPipe ──
try:
    import mediapipe as mp
except ImportError:
    import subprocess, sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "mediapipe"])
    import mediapipe as mp

try:
    import pyttsx3
    VOICE_AVAILABLE = True
except ImportError:
    VOICE_AVAILABLE = False

print("✅ All packages loaded!")


# ============================================================
# ANGLE CALCULATION UTILITIES
# ============================================================
def calculate_angle(point_a, point_b, point_c):
    """
    Calculate angle at point_b formed by line AB and line BC.
    
    Points are (x, y) tuples.
    Returns angle in degrees (0-180).
    
        A
         \
          \ angle
           B ──── C
    """
    a = np.array(point_a)
    b = np.array(point_b)
    c = np.array(point_c)

    ba = a - b
    bc = c - b

    cosine = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    cosine = np.clip(cosine, -1.0, 1.0)
    angle = np.degrees(np.arccos(cosine))
    return angle


def calculate_angle_3d(point_a, point_b, point_c):
    """
    Calculate angle at point_b in 3D space.
    Points are (x, y, z) tuples.
    """
    a = np.array(point_a)
    b = np.array(point_b)
    c = np.array(point_c)

    ba = a - b
    bc = c - b

    cosine = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    cosine = np.clip(cosine, -1.0, 1.0)
    angle = np.degrees(np.arccos(cosine))
    return angle


def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another, clamped."""
    value = max(in_min, min(in_max, value))
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min + 1e-6)


def smooth_value(current, target, factor=0.3):
    """Exponential smoothing to reduce jitter."""
    return current + factor * (target - current)


# ============================================================
# ENHANCED UART FOR ARM CONTROL
# ============================================================
class ArmUart:
    """
    Extended UART protocol for robotic arm control.
    
    Protocol (PC → FPGA):
        0xA0-0xAF : Gesture/finger count (existing)
        0xB0-0xBF : Base rotation     (0-15 → 0°-180°)
        0xC0-0xCF : Elbow angle       (0-15 → 0°-180°)
        0xD0-0xDF : Wrist angle        (0-15 → 0°-180°)
        0xE0-0xEF : Gripper            (0=closed, 15=open)
    """

    def __init__(self, port=None, baud=9600):
        self.serial = None
        self.connected = False
        self.baud = baud
        self.port = port
        self.send_count = 0
        self.last_sent = {}  # Track last sent value per joint

    def find_fpga_port(self):
        ports = serial.tools.list_ports.comports()
        print("\n📡 Available serial ports:")
        for p in ports:
            print(f"   {p.device}: {p.description} [VID:PID={p.vid}:{p.pid}]")
            if p.vid == 0x0403 or "FTDI" in (p.description or "").upper():
                return p.device
        if ports:
            return ports[0].device
        return None

    def connect(self):
        port = self.port or self.find_fpga_port()
        if not port:
            print("❌ No serial port found!")
            return False
        try:
            self.serial = serial.Serial(
                port=port, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1
            )
            time.sleep(0.2)
            self.connected = True
            print(f"✅ Connected to FPGA on {port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"❌ Serial error: {e}")
            return False

    def send_joint(self, joint_cmd, value_4bit):
        """
        Send a joint position command.
        joint_cmd: 0xB0 (base), 0xC0 (elbow), 0xD0 (wrist), 0xE0 (gripper)
        value_4bit: 0-15
        """
        if not self.connected or not self.serial:
            return False

        value_4bit = max(0, min(15, int(value_4bit)))
        data_byte = joint_cmd | value_4bit

        # Only send if value changed for this joint
        if self.last_sent.get(joint_cmd) == data_byte:
            return True

        try:
            self.serial.write(bytes([data_byte]))
            self.last_sent[joint_cmd] = data_byte
            self.send_count += 1
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def send_base(self, val):
        return self.send_joint(0xB0, val)

    def send_elbow(self, val):
        return self.send_joint(0xC0, val)

    def send_wrist(self, val):
        return self.send_joint(0xD0, val)

    def send_gripper(self, val):
        return self.send_joint(0xE0, val)

    def send_all(self, base, elbow, wrist, gripper):
        """Send all joint positions. Only sends if values changed."""
        self.send_base(base)
        self.send_elbow(elbow)
        self.send_wrist(wrist)
        self.send_gripper(gripper)

    def read_ack(self):
        if not self.connected or not self.serial:
            return None
        try:
            if self.serial.in_waiting > 0:
                return self.serial.read(self.serial.in_waiting)
        except:
            pass
        return None

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("🔌 UART disconnected")
        self.connected = False


# ============================================================
# ARM ANGLE TRACKER
# ============================================================
class ArmAngleTracker:
    """
    Tracks your real arm angles using MediaPipe Pose + Hands.
    
    What we track:
    ┌─────────────────────────────────────────────────┐
    │  SHOULDER (pose landmark 12)                    │
    │      │                                          │
    │      │  ← upper arm                             │
    │      │                                          │
    │  ELBOW (pose landmark 14)                       │
    │      │        ← elbow_angle: angle at elbow     │
    │      │  ← forearm                               │
    │      │                                          │
    │  WRIST (pose landmark 16 / hand landmark 0)     │
    │      │        ← wrist_angle: angle at wrist     │
    │      │                                          │
    │  FINGERS (hand landmarks)                       │
    │              ← grip: open (0°) or closed (180°) │
    └─────────────────────────────────────────────────┘
    """

    def __init__(self):
        # MediaPipe Pose — for shoulder, elbow, wrist positions
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5
        )

        # MediaPipe Hands — for finger tracking (grip detection)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5
        )

        self.mp_draw = mp.solutions.drawing_utils

        # Smoothed values (reduces jitter)
        self.smooth_elbow = 90.0
        self.smooth_wrist = 90.0
        self.smooth_grip = 0.0
        self.smooth_base = 90.0

        # Finger tip and pip IDs for grip calculation
        self.TIP_IDS = [4, 8, 12, 16, 20]
        self.PIP_IDS = [3, 6, 10, 14, 18]

    def process(self, frame):
        """
        Process one frame and return arm angles.
        
        Returns dict with:
            elbow_angle:  0-180° (straight=180, fully bent=~30)
            wrist_angle:  0-180° (extension/flexion)
            grip_amount:  0-100  (0=open, 100=closed fist)
            base_angle:   0-180° (left-right position of hand)
            detected:     True if arm is visible
        """
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = {
            'detected': False,
            'elbow_angle': 90,
            'wrist_angle': 90,
            'grip_amount': 0,
            'base_angle': 90,
            'pose_landmarks': None,
            'hand_landmarks': None,
            'hand_data': None
        }

        # ── POSE DETECTION (shoulder, elbow, wrist) ──
        pose_results = self.pose.process(rgb)

        if pose_results.pose_landmarks:
            lm = pose_results.pose_landmarks.landmark
            result['pose_landmarks'] = pose_results.pose_landmarks

            # Get RIGHT arm landmarks (use left arm if you prefer)
            # Landmark IDs: 11=L shoulder, 12=R shoulder,
            #               13=L elbow, 14=R elbow,
            #               15=L wrist, 16=R wrist
            shoulder = lm[12]  # Right shoulder
            elbow = lm[14]     # Right elbow
            wrist = lm[16]     # Right wrist

            # Check visibility
            if (shoulder.visibility > 0.5 and
                    elbow.visibility > 0.5 and
                    wrist.visibility > 0.5):

                result['detected'] = True

                # ── ELBOW ANGLE ──
                # Angle at the elbow joint (shoulder-elbow-wrist)
                # Straight arm = ~170°, fully bent = ~30°
                shoulder_px = (int(shoulder.x * w), int(shoulder.y * h))
                elbow_px = (int(elbow.x * w), int(elbow.y * h))
                wrist_px = (int(wrist.x * w), int(wrist.y * h))

                elbow_angle = calculate_angle(shoulder_px, elbow_px, wrist_px)
                self.smooth_elbow = smooth_value(self.smooth_elbow, elbow_angle, 0.35)
                result['elbow_angle'] = self.smooth_elbow

                # ── BASE ROTATION ──
                # Horizontal position of wrist → base rotation
                # Wrist at left edge = 0°, right edge = 180°
                # (Remember: frame is flipped, so left in image = your right)
                base_angle = map_range(wrist.x, 0.2, 0.8, 0, 180)
                self.smooth_base = smooth_value(self.smooth_base, base_angle, 0.3)
                result['base_angle'] = self.smooth_base

        # ── HAND DETECTION (wrist angle + grip) ──
        hand_results = self.hands.process(rgb)

        if hand_results.multi_hand_landmarks:
            hand_lm = hand_results.multi_hand_landmarks[0]
            result['hand_landmarks'] = hand_lm

            # Get hand landmarks in pixel coordinates
            lm_px = []
            for landmark in hand_lm.landmark:
                px = int(landmark.x * w)
                py = int(landmark.y * h)
                lm_px.append((px, py))

            result['hand_data'] = lm_px

            # ── WRIST ANGLE ──
            # Angle between forearm direction and hand direction
            # Use: elbow → wrist → middle_finger_mcp(9)
            #
            #   Elbow (from pose)
            #      \
            #       \ forearm
            #        \
            #    Wrist (landmark 0) ─── angle ─── Middle MCP (landmark 9)
            #                                        │
            #                                      hand direction

            if result['detected']:  # Need pose data for elbow position
                wrist_pt = lm_px[0]       # Hand wrist
                mid_mcp = lm_px[9]        # Middle finger MCP

                # Use pose elbow as the "forearm origin"
                pose_lm = pose_results.pose_landmarks.landmark
                elbow_pt = (int(pose_lm[14].x * w), int(pose_lm[14].y * h))

                wrist_angle = calculate_angle(elbow_pt, wrist_pt, mid_mcp)
                self.smooth_wrist = smooth_value(self.smooth_wrist, wrist_angle, 0.35)
                result['wrist_angle'] = self.smooth_wrist

            # ── GRIP AMOUNT ──
            # Average distance between fingertips and palm center
            # Closed fist = small distance = 100% grip
            # Open hand = large distance = 0% grip
            palm_center = np.mean([lm_px[0], lm_px[5], lm_px[9],
                                   lm_px[13], lm_px[17]], axis=0)

            # Calculate average tip-to-palm distance
            tip_distances = []
            for tip_id in [8, 12, 16, 20]:  # Skip thumb for now
                tip = np.array(lm_px[tip_id])
                dist = np.linalg.norm(tip - palm_center)
                tip_distances.append(dist)

            avg_tip_dist = np.mean(tip_distances)

            # Also check finger curl (tip below pip = curled)
            fingers_curled = 0
            for i in range(1, 5):  # Index through pinky
                tip = lm_px[self.TIP_IDS[i]]
                pip = lm_px[self.PIP_IDS[i]]
                if tip[1] > pip[1]:  # tip is BELOW pip = finger curled
                    fingers_curled += 1

            # Combine distance and curl for robust grip detection
            # Normalize distance: ~30px = closed, ~120px = open (varies with hand size)
            grip_from_dist = map_range(avg_tip_dist, 30, 120, 100, 0)
            grip_from_curl = (fingers_curled / 4.0) * 100

            grip = grip_from_dist * 0.4 + grip_from_curl * 0.6
            grip = max(0, min(100, grip))

            self.smooth_grip = smooth_value(self.smooth_grip, grip, 0.35)
            result['grip_amount'] = self.smooth_grip

        return result


# ============================================================
# VISUALIZATION
# ============================================================
def draw_arm_display(frame, data, uart):
    """Draw arm tracking visualization with angle info."""
    vis = frame.copy()
    h, w = vis.shape[:2]

    # Draw pose skeleton
    if data['pose_landmarks']:
        mp.solutions.drawing_utils.draw_landmarks(
            vis, data['pose_landmarks'],
            mp.solutions.pose.POSE_CONNECTIONS,
            mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=3),
            mp.solutions.drawing_utils.DrawingSpec(color=(0, 200, 0), thickness=2)
        )

    # Draw hand landmarks
    if data['hand_landmarks']:
        mp.solutions.drawing_utils.draw_landmarks(
            vis, data['hand_landmarks'],
            mp.solutions.hands.HAND_CONNECTIONS,
            mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 255), thickness=2, circle_radius=2),
            mp.solutions.drawing_utils.DrawingSpec(color=(255, 255, 0), thickness=2)
        )

    # ── Info Panel ──
    panel_h = 280
    cv2.rectangle(vis, (10, 10), (350, panel_h), (0, 0, 0), -1)
    cv2.rectangle(vis, (10, 10), (350, panel_h), (100, 100, 100), 2)

    y = 30
    cv2.putText(vis, "ARM MIRROR -> FPGA -> ROBOT", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    y += 30
    status = "TRACKING" if data['detected'] else "NO ARM DETECTED"
    color = (0, 255, 0) if data['detected'] else (0, 0, 255)
    cv2.putText(vis, f"Status: {status}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

    if data['detected']:
        # Elbow angle bar
        y += 30
        elbow = data['elbow_angle']
        cv2.putText(vis, f"Elbow:   {elbow:.0f} deg", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        bar_w = int(map_range(elbow, 30, 170, 0, 150))
        cv2.rectangle(vis, (200, y - 12), (200 + bar_w, y + 2), (0, 200, 255), -1)

        # Wrist angle bar
        y += 30
        wrist = data['wrist_angle']
        cv2.putText(vis, f"Wrist:   {wrist:.0f} deg", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        bar_w = int(map_range(wrist, 30, 170, 0, 150))
        cv2.rectangle(vis, (200, y - 12), (200 + bar_w, y + 2), (255, 200, 0), -1)

        # Grip bar
        y += 30
        grip = data['grip_amount']
        grip_label = "CLOSED" if grip > 70 else "OPEN" if grip < 30 else "PARTIAL"
        cv2.putText(vis, f"Grip:    {grip:.0f}% ({grip_label})", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        bar_w = int(map_range(grip, 0, 100, 0, 150))
        grip_color = (0, 0, 255) if grip > 70 else (0, 255, 0) if grip < 30 else (0, 255, 255)
        cv2.rectangle(vis, (200, y - 12), (200 + bar_w, y + 2), grip_color, -1)

        # Base rotation bar
        y += 30
        base = data['base_angle']
        cv2.putText(vis, f"Base:    {base:.0f} deg", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        bar_w = int(map_range(base, 0, 180, 0, 150))
        cv2.rectangle(vis, (200, y - 12), (200 + bar_w, y + 2), (200, 100, 255), -1)

    # UART status
    y += 30
    uart_status = "CONNECTED" if uart.connected else "DISCONNECTED"
    uart_color = (0, 255, 0) if uart.connected else (0, 0, 255)
    cv2.putText(vis, f"UART: {uart_status}  (#{uart.send_count})", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, uart_color, 1)

    # ── Draw angle arcs on the body ──
    if data['detected'] and data['pose_landmarks']:
        lm = data['pose_landmarks'].landmark

        # Draw elbow angle arc
        elbow_px = (int(lm[14].x * w), int(lm[14].y * h))
        cv2.putText(vis, f"{data['elbow_angle']:.0f}", 
                    (elbow_px[0] + 10, elbow_px[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    return vis


# ============================================================
# MAIN — ARM MIRROR MODE
# ============================================================
def start_arm_mirror(seconds=300, port=None, baud=9600):
    """
    Start arm mirroring mode.
    
    Your real arm movements are tracked and sent to the FPGA,
    which drives servo motors to mirror your movements.
    
    Controls:
        - Move your right arm → robot arm follows
        - Bend elbow → robot elbow bends
        - Tilt wrist → robot wrist tilts
        - Close fist → robot gripper closes
        - Open hand → robot gripper opens
        - Move hand left/right → robot base rotates
    
    Keys:
        ESC = quit
        R   = reset/recalibrate
    """
    print("=" * 60)
    print("🤖 ARM MIRROR MODE → FPGA → ROBOTIC ARM")
    print(f"   Baud: {baud}, Duration: {seconds}s")
    print("=" * 60)
    print("\n   Move your RIGHT arm in front of the camera!")
    print("   The robot will mirror your movements.\n")

    tracker = ArmAngleTracker()
    uart = ArmUart(port=port, baud=baud)

    if not uart.connect():
        print("\n⚠️  Running WITHOUT FPGA (preview mode)")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ No webcam found!")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    t0 = time.time()
    frame_count = 0
    fps_timer = time.time()
    fps = 0
    send_timer = time.time()
    SEND_INTERVAL = 0.05 

    try:
        while time.time() - t0 < seconds:
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, 1)

            # Track arm angles
            data = tracker.process(frame)

            # ── Send to FPGA at controlled rate ──
            now = time.time()
            if data['detected'] and (now - send_timer) >= SEND_INTERVAL:
                send_timer = now

                # Map angles to 0-15 range for UART protocol
                # Elbow: 30°-170° → 0-15
                elbow_val = int(map_range(data['elbow_angle'], 30, 170, 0, 15))

                # Wrist: 30°-170° → 0-15
                wrist_val = int(map_range(data['wrist_angle'], 30, 170, 0, 15))

                # Gripper: 0-100% → 15(open) to 0(closed)
                grip_val = int(map_range(data['grip_amount'], 0, 100, 15, 0))

                # Base: 0°-180° → 0-15
                base_val = int(map_range(data['base_angle'], 0, 180, 0, 15))

                uart.send_all(base_val, elbow_val, wrist_val, grip_val)

           
            uart.read_ack()

          
            vis = draw_arm_display(frame, data, uart)

            
            frame_count += 1
            if now - fps_timer > 1.0:
                fps = frame_count / (now - fps_timer + 0.001)
                fps_timer = now
                frame_count = 0
            cv2.putText(vis, f"FPS: {fps:.0f}", (vis.shape[1] - 100, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        
            cv2.imshow('Arm Mirror -> FPGA -> Robot', vis)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('r'):  # Reset
                tracker = ArmAngleTracker()
                print("🔄 Reset!")

    except KeyboardInterrupt:
        print("\n🛑 Stopped!")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        uart.disconnect()

    print(f"\n{'=' * 60}")
    print(f"📊 UART sent {uart.send_count} packets")
    print("=" * 60)

if __name__ == "__main__":
    print("=" * 60)
    print("🤖 ARM MIRROR MODE READY!")
    print("=" * 60)
    print("\n   start_arm_mirror()                  # Auto-detect port")
    print("   start_arm_mirror(port='COM4')       # Specify port")
    print("=" * 60)
    start_arm_mirror()
