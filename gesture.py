# ============================================================
# GESTURE → FPGA UART SENDER
# ============================================================
# Sends finger count (0-5) to Nexys 4 DDR FPGA via UART
# Uses the built-in FTDI USB-UART on the Nexys 4 DDR board
#
# Install: pip install pyserial mediapipe opencv-python
# ============================================================

import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
from collections import deque, Counter
from enum import Enum, auto
import threading

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
# UART COMMUNICATION
# ============================================================
class FPGAUart:
    """
    UART communication with Nexys 4 DDR FPGA.

    Protocol (PC → FPGA):
        Byte format: 0xA<n>  where n = finger count (0-5)
        Examples:
            0xA0 = fist    (0 fingers)
            0xA1 = one     (1 finger)
            0xA2 = two     (2 fingers)
            0xA3 = three   (3 fingers)
            0xA4 = four    (4 fingers)
            0xA5 = five    (5 fingers)
            0xAF = no hand detected

        Upper nibble 0xA acts as a sync/header marker.
        Lower nibble carries the gesture data.

    FPGA → PC (optional acknowledgment):
        Echoes back the received byte.
    """

    def __init__(self, port=None, baud=9600):
        self.serial = None
        self.connected = False
        self.baud = baud
        self.port = port
        self.last_sent = None
        self.send_count = 0

    def find_fpga_port(self):
        """Auto-detect the Nexys 4 DDR FTDI port."""
        ports = serial.tools.list_ports.comports()
        print("\n📡 Available serial ports:")
        for p in ports:
            print(f"   {p.device}: {p.description} [VID:PID={p.vid}:{p.pid}]")
            # Digilent/FTDI typically has VID 0x0403
            if p.vid == 0x0403 or "FTDI" in (p.description or "").upper() or \
               "Digilent" in (p.description or ""):
                print(f"   ✅ Likely FPGA port: {p.device}")
                return p.device

        # If auto-detect fails, list ports for manual selection
        if ports:
            print(f"\n   ⚠️ Auto-detect failed. Using first port: {ports[0].device}")
            return ports[0].device
        return None

    def connect(self):
        """Connect to the FPGA's UART port."""
        port = self.port or self.find_fpga_port()
        if not port:
            print("❌ No serial port found! Is the FPGA connected via USB?")
            return False

        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            time.sleep(0.1)  # Let the port settle
            self.connected = True
            print(f"✅ Connected to FPGA on {port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"❌ Serial error: {e}")
            return False

    def send_gesture(self, finger_count):
        """
        Send finger count to FPGA.
        Encodes as: 0xA0-0xA5 for 0-5 fingers, 0xAF for no hand.
        """
        if not self.connected or not self.serial:
            return False

        if finger_count < 0 or finger_count > 5:
            data_byte = 0xAF  # No hand / invalid
        else:
            data_byte = 0xA0 | (finger_count & 0x0F)

        # Only send if value changed (reduce UART traffic)
        if data_byte == self.last_sent:
            return True

        try:
            self.serial.write(bytes([data_byte]))
            self.last_sent = data_byte
            self.send_count += 1
            return True
        except serial.SerialException:
            print("⚠️ UART write failed")
            self.connected = False
            return False

    def read_ack(self):
        """Read acknowledgment from FPGA (optional)."""
        if not self.connected or not self.serial:
            return None
        try:
            if self.serial.in_waiting > 0:
                return self.serial.read(1)
        except:
            pass
        return None

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("🔌 UART disconnected")
        self.connected = False


# ============================================================
# VOICE ENGINE (from your original code)
# ============================================================
class VoiceEngine:
    def __init__(self):
        self.engine = None
        self.enabled = VOICE_AVAILABLE
        self.last_spoken = ""
        self.last_time = 0
        self.cooldown = 3.0
        self.speaking = False
        if not VOICE_AVAILABLE:
            return
        try:
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)
        except:
            self.enabled = False

    def speak(self, text):
        if not self.enabled or not self.engine or not text:
            return
        now = time.time()
        if text == self.last_spoken and (now - self.last_time) < self.cooldown:
            return
        if self.speaking:
            return
        self.last_spoken = text
        self.last_time = now
        def _r():
            self.speaking = True
            try:
                self.engine.say(text)
                self.engine.runAndWait()
            except:
                pass
            self.speaking = False
        threading.Thread(target=_r, daemon=True).start()


# ============================================================
# STATE MACHINE (from your original code — unchanged)
# ============================================================
class GestureState(Enum):
    IDLE = auto()
    DETECTING = auto()
    LOCKED = auto()
    TRANSITIONING = auto()
    COOLDOWN = auto()

class StableStateMachine:
    def __init__(self):
        self.state = GestureState.IDLE
        self.locked_gesture = -1
        self.no_hand_count = 0
        self.locked_frame_count = 0
        self.vote_window = deque(maxlen=25)
        self.transition_votes = deque(maxlen=25)
        self.detect_window = 10
        self.lock_majority = 0.55
        self.transition_window = 15
        self.transition_majority = 0.65
        self.min_hold_frames = 10
        self.idle_frames = 20
        self.cooldown_frames = 6
        self.cooldown_counter = 0

    def update(self, gesture, hand_detected):
        if self.state == GestureState.IDLE:
            if hand_detected and gesture >= 0:
                self.state = GestureState.DETECTING
                self.vote_window.clear()
                self.vote_window.append(gesture)
                self.no_hand_count = 0
                return (-1, False, "DETECTING")
            return (-1, False, "IDLE")

        elif self.state == GestureState.DETECTING:
            if not hand_detected:
                self.no_hand_count += 1
                if self.no_hand_count > 8:
                    self.state = GestureState.IDLE
                    self.vote_window.clear()
                    self.no_hand_count = 0
                    return (-1, False, "IDLE")
                return (-1, False, "DETECTING")
            self.no_hand_count = 0
            self.vote_window.append(gesture)
            if len(self.vote_window) >= self.detect_window:
                mg, r = self._maj(self.vote_window)
                if r >= self.lock_majority and mg >= 0:
                    self.locked_gesture = mg
                    self.locked_frame_count = 0
                    self.state = GestureState.COOLDOWN
                    self.cooldown_counter = self.cooldown_frames
                    return (self.locked_gesture, True, "LOCKED")
            return (-1, False, "DETECTING")

        elif self.state == GestureState.LOCKED:
            self.locked_frame_count += 1
            if not hand_detected:
                self.no_hand_count += 1
                if self.no_hand_count > self.idle_frames:
                    self.state = GestureState.IDLE
                    self.locked_gesture = -1
                    self.no_hand_count = 0
                    self.locked_frame_count = 0
                    return (-1, False, "IDLE")
                return (self.locked_gesture, True, "LOCKED")
            self.no_hand_count = 0
            if self.locked_frame_count < self.min_hold_frames:
                return (self.locked_gesture, True, "LOCKED")
            if gesture != self.locked_gesture:
                self.state = GestureState.TRANSITIONING
                self.transition_votes.clear()
                self.transition_votes.append(gesture)
                return (self.locked_gesture, True, "TRANSITIONING")
            return (self.locked_gesture, True, "LOCKED")

        elif self.state == GestureState.TRANSITIONING:
            if not hand_detected:
                self.no_hand_count += 1
                if self.no_hand_count > self.idle_frames:
                    self.state = GestureState.IDLE
                    self.locked_gesture = -1
                    return (-1, False, "IDLE")
                return (self.locked_gesture, True, "TRANSITIONING")
            self.no_hand_count = 0
            self.transition_votes.append(gesture)
            if len(self.transition_votes) >= 5:
                mg, r = self._maj(self.transition_votes)
                if mg == self.locked_gesture and r > 0.4:
                    self.state = GestureState.LOCKED
                    self.transition_votes.clear()
                    return (self.locked_gesture, True, "LOCKED")
                if (len(self.transition_votes) >= self.transition_window and
                        r >= self.transition_majority and
                        mg != self.locked_gesture and mg >= 0):
                    self.locked_gesture = mg
                    self.locked_frame_count = 0
                    self.state = GestureState.COOLDOWN
                    self.cooldown_counter = self.cooldown_frames
                    self.transition_votes.clear()
                    return (self.locked_gesture, True, "LOCKED")
            return (self.locked_gesture, True, "TRANSITIONING")

        elif self.state == GestureState.COOLDOWN:
            self.cooldown_counter -= 1
            if self.cooldown_counter <= 0:
                self.state = GestureState.LOCKED
                self.vote_window.clear()
                self.locked_frame_count = 0
            return (self.locked_gesture, True, "COOLDOWN")

        return (-1, False, "?")

    def _maj(self, w):
        if not w:
            return (-1, 0.0)
        c = Counter(w)
        g, n = c.most_common(1)[0]
        return (g, n / len(w))

    def reset(self):
        self.state = GestureState.IDLE
        self.locked_gesture = -1
        self.no_hand_count = 0
        self.locked_frame_count = 0
        self.vote_window.clear()
        self.transition_votes.clear()
        self.cooldown_counter = 0


# ============================================================
# MEDIAPIPE HAND DETECTOR (from your original code)
# ============================================================
class HandDetector:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.TIP_IDS = [4, 8, 12, 16, 20]
        self.PIP_IDS = [3, 6, 10, 14, 18]

    def process(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        data = {
            'detected': False,
            'landmarks': None,
            'finger_count': 0,
            'fingers_up': [False] * 5,
            'bbox': None,
            'centroid': None,
            'handedness': 'Right'
        }

        if not results.multi_hand_landmarks:
            return data

        hand_lm = results.multi_hand_landmarks[0]
        h, w = frame.shape[:2]

        if results.multi_handedness:
            data['handedness'] = results.multi_handedness[0].classification[0].label

        lm_pixels = []
        for lm in hand_lm.landmark:
            px = int(lm.x * w)
            py = int(lm.y * h)
            lm_pixels.append((px, py))

        data['detected'] = True
        data['landmarks'] = lm_pixels

        xs = [p[0] for p in lm_pixels]
        ys = [p[1] for p in lm_pixels]
        margin = 20
        data['bbox'] = (
            max(0, min(xs) - margin), max(0, min(ys) - margin),
            min(w, max(xs) + margin), min(h, max(ys) + margin)
        )
        data['centroid'] = (sum(xs) // 21, sum(ys) // 21)

        fingers = [False] * 5
        if data['handedness'] == 'Right':
            fingers[0] = lm_pixels[4][0] < lm_pixels[3][0]
        else:
            fingers[0] = lm_pixels[4][0] > lm_pixels[3][0]

        for i in range(1, 5):
            tip = lm_pixels[self.TIP_IDS[i]]
            pip_joint = lm_pixels[self.PIP_IDS[i]]
            fingers[i] = tip[1] < pip_joint[1]

        data['fingers_up'] = fingers
        data['finger_count'] = sum(fingers)
        return data

    def draw_landmarks(self, frame, hand_data):
        if not hand_data['detected'] or hand_data['landmarks'] is None:
            return frame

        vis = frame.copy()
        lm = hand_data['landmarks']
        connections = [
            (0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
            (0,9),(9,10),(10,11),(11,12),(0,13),(13,14),(14,15),(15,16),
            (0,17),(17,18),(18,19),(19,20),(5,9),(9,13),(13,17)
        ]
        for c in connections:
            if c[0] < len(lm) and c[1] < len(lm):
                cv2.line(vis, lm[c[0]], lm[c[1]], (0, 255, 0), 2)
        for i, pt in enumerate(lm):
            color = (0, 0, 255) if i == 0 else (0, 255, 255)
            radius = 6 if i in [4, 8, 12, 16, 20] else 3
            cv2.circle(vis, pt, radius, color, -1)
        if hand_data['bbox']:
            x1, y1, x2, y2 = hand_data['bbox']
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return vis


# ============================================================
# MAIN RECOGNIZER WITH UART
# ============================================================
class GestureRecognizerUART:
    def __init__(self):
        self.detector = HandDetector()
        self.state_machine = StableStateMachine()
        self.uart = FPGAUart()
        self.voice = VoiceEngine()
        self.gesture_names = {
            -1: "No Hand", 0: "FIST", 1: "ONE",
            2: "TWO", 3: "THREE", 4: "FOUR", 5: "FIVE"
        }

    def process_frame(self, frame):
        hand = self.detector.process(frame)
        raw = hand['finger_count'] if hand['detected'] else -1

        if hand['detected']:
            g, s, st = self.state_machine.update(hand['finger_count'], True)
        else:
            g, s, st = self.state_machine.update(-1, False)

        return {
            'hand': hand,
            'raw_gesture': raw,
            'stable_gesture': g,
            'is_stable': s,
            'state': st
        }

    def reset(self):
        self.state_machine.reset()


# ============================================================
# VISUALIZATION WITH UART STATUS
# ============================================================
def draw_display(frame, result, rec):
    vis = rec.detector.draw_landmarks(frame, result['hand'])

    # Info panel
    cv2.rectangle(vis, (10, 10), (320, 220), (0, 0, 0), -1)
    cv2.rectangle(vis, (10, 10), (320, 220), (100, 100, 100), 2)

    y = 30
    cv2.putText(vis, "GESTURE -> FPGA (UART)", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    y += 25
    state_colors = {
        'IDLE': (128,128,128), 'DETECTING': (0,255,255),
        'LOCKED': (0,255,0), 'TRANSITIONING': (0,165,255),
        'COOLDOWN': (255,255,0)
    }
    cv2.putText(vis, f"State: {result['state']}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                state_colors.get(result['state'], (255,255,255)), 2)

    y += 25
    cv2.putText(vis, f"Raw fingers: {result['raw_gesture']}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    y += 25
    sg = result['stable_gesture']
    gn = rec.gesture_names.get(sg, "?")
    cv2.putText(vis, f"Stable: {gn} ({sg})", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    y += 25
    uart_status = "CONNECTED" if rec.uart.connected else "DISCONNECTED"
    uart_color = (0, 255, 0) if rec.uart.connected else (0, 0, 255)
    cv2.putText(vis, f"UART: {uart_status}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, uart_color, 2)

    y += 25
    if rec.uart.connected:
        sent_hex = f"0x{rec.uart.last_sent:02X}" if rec.uart.last_sent else "---"
        cv2.putText(vis, f"Last TX: {sent_hex}  (#{rec.uart.send_count})", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    y += 25
    if result['is_stable']:
        cv2.putText(vis, ">> FPGA ACTIVE <<", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    else:
        cv2.putText(vis, "... stabilizing ...", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    return vis


# ============================================================
# MAIN ENTRY POINT
# ============================================================
def start(seconds=120, port=None, baud=9600):
    """
    Start gesture recognition with FPGA UART output.

    Args:
        seconds:  Duration in seconds (default 120)
        port:     Serial port (auto-detect if None)
                  Windows: 'COM3', 'COM4', etc.
                  Linux:   '/dev/ttyUSB0', '/dev/ttyUSB1'
                  Mac:     '/dev/tty.usbserial-xxxxx'
        baud:     Baud rate (must match FPGA, default 9600)
    """
    print("=" * 60)
    print("🤖 GESTURE → FPGA (Nexys 4 DDR) via UART")
    print(f"   Baud: {baud}, Duration: {seconds}s")
    print("=" * 60)

    rec = GestureRecognizerUART()

    # Connect UART
    rec.uart.port = port
    rec.uart.baud = baud
    if not rec.uart.connect():
        print("\n⚠️  Running WITHOUT FPGA (display only mode)")
        print("    Connect FPGA and restart, or specify port manually:")
        print("    start(port='COM3')  # Windows")
        print("    start(port='/dev/ttyUSB1')  # Linux")
        # Continue anyway for testing without FPGA

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ No webcam found!")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rec.reset()
    rec.voice.speak("Ready. Show your hand.")

    t0 = time.time()
    frame_count = 0
    last_announced = -1
    last_uart_gesture = -2  # Track what we last sent
    arm_log = []
    fps_timer = time.time()
    fps = 0

    try:
        while time.time() - t0 < seconds:
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, 1)

            # Process gesture
            result = rec.process_frame(frame)
            sg = result['stable_gesture']

            # ── Send to FPGA via UART ──
            if result['is_stable'] and sg >= 0:
                rec.uart.send_gesture(sg)

                if sg != last_announced:
                    name = rec.gesture_names.get(sg, "?")
                    rec.voice.speak(name)
                    print(f"📤 UART TX: 0x{0xA0 | sg:02X} → {name} ({sg} fingers)")
                    last_announced = sg
                    arm_log.append({
                        't': time.time() - t0,
                        'g': sg,
                        'n': name
                    })
            elif not result['is_stable'] and not result['hand']['detected']:
                # Send "no hand" to FPGA
                rec.uart.send_gesture(-1)

            # Check for FPGA acknowledgment
            ack = rec.uart.read_ack()
            if ack:
                print(f"   📥 FPGA ACK: 0x{ack[0]:02X}")

            # Draw visualization
            vis = draw_display(frame, result, rec)

            # FPS
            frame_count += 1
            if time.time() - fps_timer > 1.0:
                fps = frame_count / (time.time() - fps_timer + 0.001)
                fps_timer = time.time()
                frame_count = 0

            cv2.putText(vis, f"FPS: {fps:.0f}", (vis.shape[1] - 100, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display
            try:
                from IPython.display import display, Image, clear_output
                _, enc = cv2.imencode('.jpg', vis)
                clear_output(wait=True)
                display(Image(data=enc.tobytes()))
            except ImportError:
                cv2.imshow('Gesture → FPGA', vis)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

    except KeyboardInterrupt:
        print("\n🛑 Stopped!")
    finally:
        cap.release()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        rec.uart.disconnect()
        rec.voice.speak("Stopped.")

    # Summary
    print(f"\n{'=' * 60}")
    print(f"📊 UART sent {rec.uart.send_count} packets, {len(arm_log)} gesture changes")
    for e in arm_log:
        print(f"   t={e['t']:.1f}s → {e['n']} ({e['g']})")
    print("=" * 60)


# ============================================================
# READY
# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("🤖 GESTURE → FPGA UART SENDER READY!")
    print("=" * 60)
    print("\n   start()                  # Auto-detect port")
    print("   start(port='COM3')       # Windows")
    print("   start(port='/dev/ttyUSB1')  # Linux")
    print("   start(baud=115200)       # Faster baud")
    print("=" * 60)
    start()
