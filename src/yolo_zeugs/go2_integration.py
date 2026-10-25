import json
import logging
import time
from ultralytics import YOLO

# ==============================================================================
# 1. KONFIGURATION & LOGGING INITIALISIEREN
# ==============================================================================
# Config laden (mit Fallback-Werten)
try:
    with open("config.json", "r") as f:
        config = json.load(f)
except FileNotFoundError:
    config = {
        "model_path": "best.onnx",
        "conf_threshold": 0.35,
        "required_frames": 3,
        "log_to_file": True
    }

# Logging in 'mission_log.txt' einrichten
if config.get("log_to_file", True):
    logging.basicConfig(
        filename="mission_log.txt",
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

def write_log(message):
    """Gibt Nachrichten im Terminal aus und schreibt sie gleichzeitig ins Logfile."""
    print(message)
    if config.get("log_to_file", True):
        logging.info(message)


# ==============================================================================
# 2. ROBOTER-STEUERUNG (MOCK CONTROLLER)
# ==============================================================================
class Go2RobotController:
    def __init__(self, simulation=True):
        self.simulation = simulation

    def stop_movement(self):
        msg = "🛑 [ROBOTER-ACTION] -> STOPP! Geschlossene Tür verifiziert."
        write_log(msg)
        if not self.simulation:
            # Echter SDK-Aufruf (später vor Ort):
            # self.sport_client.StopMove()
            pass

    def continue_movement(self):
        msg = "🟢 [ROBOTER-ACTION] -> WEITERFAHREN. Weg frei verifiziert."
        write_log(msg)
        if not self.simulation:
            # Echter SDK-Aufruf (später vor Ort):
            # self.sport_client.Move(0.3, 0, 0)
            pass

    def log_event(self, event_text):
        write_log(f"ℹ️ [ROBOTER-LOG] -> Statusmeldung: {event_text}")


# ==============================================================================
# 3. VISION SYSTEM MIT ENTPRELLUNG & LOGGING
# ==============================================================================
class Go2VisionSystem:
    def __init__(self, model_path="best.onnx", conf_threshold=0.35, required_frames=3):
        write_log(f"Lade ONNX-Modell '{model_path}' (Conf: {conf_threshold}, Frames: {required_frames})...")
        self.model = YOLO(model_path)
        self.conf = conf_threshold
        self.required_frames = required_frames
        self.robot = Go2RobotController(simulation=True)

        self.door_closed_counter = 0
        self.door_opened_counter = 0

    def process_frame(self, frame_source):
        results = self.model(frame_source, conf=self.conf, verbose=False)

        detected_classes = []
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                confidence = float(box.conf[0])
                detected_classes.append(class_name)
                write_log(f"  [Kamera] Erkannt: '{class_name}' ({confidence:.1%})")

        self.evaluate_rules(detected_classes)

    def evaluate_rules(self, detected_classes):
        # Regel 1: Geschlossene Tür
        if "closed door" in detected_classes:
            self.door_closed_counter += 1
            write_log(f"  ⏳ [Entprellung] Geschlossene Tür ({self.door_closed_counter}/{self.required_frames})")
            if self.door_closed_counter >= self.required_frames:
                self.robot.stop_movement()
        else:
            self.door_closed_counter = 0

        # Regel 2: Offene Tür
        if "opened door" in detected_classes:
            self.door_opened_counter += 1
            write_log(f"  ⏳ [Entprellung] Offene Tür ({self.door_opened_counter}/{self.required_frames})")
            if self.door_opened_counter >= self.required_frames:
                self.robot.continue_movement()
        else:
            self.door_opened_counter = 0

        # Regel 3: Fenster
        if "opened window" in detected_classes:
            self.robot.log_event("Offenes Fenster erkannt")
        elif "closed window" in detected_classes:
            self.robot.log_event("Geschlossenes Fenster erkannt")

        if not detected_classes:
            write_log("  [Kamera] Keine Türen oder Fenster erkannt.")


# ==============================================================================
# 4. AUSFÜHRUNG
# ==============================================================================
if __name__ == "__main__":
    vision = Go2VisionSystem(
        model_path=config.get("model_path", "best.onnx"),
        conf_threshold=config.get("conf_threshold", 0.35),
        required_frames=config.get("required_frames", 3)
    )

    write_log("--- Start Testlauf ---")
    vision.process_frame("IMG_1339.JPEG")
