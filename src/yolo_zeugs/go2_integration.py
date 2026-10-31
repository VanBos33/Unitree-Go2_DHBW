import cv2
import json
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple
from ultralytics import YOLO


class GO2InspectionDetector:
    """
    Inspektionsklasse für die automatische Zustandsanalyse von Fenstern und Türen
    auf dem GO2 Roboter. Unterstützt 6 Klassen, Mehrfach-Fenster-Auswahl und
    Negative Samples (Bilder ohne Türen/Fenster).
    """

    CLASS_NAMES = [
        "closed door",
        "door handle",
        "opened door",
        "window",
        "window handle horizontal",
        "window handle vertical",
    ]

    def __init__(
        self,
        model_path: str = "best.onnx",
        conf_threshold: float = 0.20,
        imgsz: int = 1024,
    ):
        """
        Initialisiert das Inspektions-Modell.

        :param model_path: Pfad zur trainierten Modelldatei (.pt oder .onnx)
        :param conf_threshold: Confidence Threshold für die Inferenz
        :param imgsz: Bildauflösung für die Inferenz (1024 empfohlen)
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.imgsz = imgsz
        self.model = YOLO(self.model_path)

    @staticmethod
    def _calculate_box_area(bbox: List[float]) -> float:
        """Berechnet die Fläche einer Bounding Box [x1, y1, x2, y2]."""
        width = max(0.0, bbox[2] - bbox[0])
        height = max(0.0, bbox[3] - bbox[1])
        return width * height

    @staticmethod
    def _get_box_center(bbox: List[float]) -> Tuple[float, float]:
        """Berechnet den Mittelpunkt (cx, cy) einer Bounding Box."""
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        return cx, cy

    def _is_point_in_or_near_box(
        self,
        point: Tuple[float, float],
        bbox: List[float],
        margin_percent: float = 0.15,
    ) -> bool:
        """
        Prüft, ob ein Punkt (z. B. Griff-Zentrum) innerhalb oder knapp neben
        einer Bounding Box (z. B. Fenster) liegt.
        """
        px, py = point
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1

        # Toleranzrand hinzufügen, falls der Griff leicht über den Fensterrahmen steht
        margin_x = w * margin_percent
        margin_y = h * margin_percent

        return (x1 - margin_x) <= px <= (x2 + margin_x) and (
            y1 - margin_y
        ) <= py <= (y2 + margin_y)

    def analyze_image(
        self,
        image_path: str,
        save_annotated: bool = True,
        output_dir: str = "output",
    ) -> Dict[str, Any]:
        """
        Führt die Inferenz auf einem Bild aus und wertet den Status von Fenster und Tür aus.
        """
        image_file = Path(image_path)
        if not image_file.exists():
            raise FileNotFoundError(f"Bild nicht gefunden: {image_path}")

        # Inferenz mit YOLO durchführen
        results = self.model.predict(
            source=str(image_file),
            conf=self.conf_threshold,
            imgsz=self.imgsz,
            verbose=False,
        )[0]

        # Erkennungen nach Klassen kategorisieren
        windows: List[Dict[str, Any]] = []
        handles_hor: List[Dict[str, Any]] = []
        handles_ver: List[Dict[str, Any]] = []
        opened_doors: List[Dict[str, Any]] = []
        closed_doors: List[Dict[str, Any]] = []
        door_handles: List[Dict[str, Any]] = []
        all_detections: List[Dict[str, Any]] = []

        for box in results.boxes:
            class_id = int(box.cls[0])
            class_name = self.model.names.get(class_id, f"class_{class_id}")
            confidence = float(box.conf[0])
            xyxy = [round(c, 2) for c in box.xyxy[0].tolist()]

            det_obj = {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": round(confidence, 4),
                "bbox": xyxy,
                "area": round(self._calculate_box_area(xyxy), 2),
            }

            all_detections.append(det_obj)

            if class_name == "window":
                windows.append(det_obj)
            elif class_name == "window handle horizontal":
                handles_hor.append(det_obj)
            elif class_name == "window handle vertical":
                handles_ver.append(det_obj)
            elif class_name == "opened door":
                opened_doors.append(det_obj)
            elif class_name == "closed door":
                closed_doors.append(det_obj)
            elif class_name == "door handle":
                door_handles.append(det_obj)

        # ==========================================
        # 1. FENSTER-LOGIK (Mehrfach-Fenster-Auswahl)
        # ==========================================
        target_window: Optional[Dict[str, Any]] = None
        window_state = "NICHT_VORHANDEN"
        window_reason = "Kein Fenster auf dem Bild erkannt."

        if windows:
            # Das Hauptfenster auswählen: Größtes Fenster im Vordergrund
            target_window = max(windows, key=lambda w: w["area"])
            target_bbox = target_window["bbox"]

            # Prüfen, ob Griffe dem Zielfenster zugeordnet werden können
            has_matching_hor_handle = any(
                self._is_point_in_or_near_box(
                    self._get_box_center(h["bbox"]), target_bbox
                )
                for h in handles_hor
            )
            has_matching_ver_handle = any(
                self._is_point_in_or_near_box(
                    self._get_box_center(h["bbox"]), target_bbox
                )
                for h in handles_ver
            )

            # Fallback: Wenn genau 1 Fenster und 1 Griff erkannt wurde, diese verknüpfen
            if not has_matching_hor_handle and not has_matching_ver_handle:
                if len(windows) == 1 and len(handles_hor) > 0:
                    has_matching_hor_handle = True
                elif len(windows) == 1 and len(handles_ver) > 0:
                    has_matching_ver_handle = True

            # Zustand bestimmen
            if has_matching_hor_handle:
                window_state = "OFFEN / GEKIPPT"
                window_reason = f"Hauptfenster ({len(windows)} erkannt) hat einen horizontalen Griff."
            elif has_matching_ver_handle:
                window_state = "GESCHLOSSEN"
                window_reason = f"Hauptfenster ({len(windows)} erkannt) hat einen vertikalen Griff."
            else:
                window_state = "UNKLAR"
                window_reason = f"Hauptfenster erkannt, aber kein zugehöriger Griff identifiziert."

        elif handles_hor or handles_ver:
            # Spezialfall: Griff erkannt, aber Fensterrahmen nicht als Box erfasst
            if handles_hor:
                window_state = "OFFEN / GEKIPPT"
                window_reason = (
                    "Horizontaler Fenstergriff ohne Fensterrahmen erkannt."
                )
            else:
                window_state = "GESCHLOSSEN"
                window_reason = (
                    "Vertikaler Fenstergriff ohne Fensterrahmen erkannt."
                )

        # ==========================================
        # 2. TÜR-LOGIK
        # ==========================================
        door_state = "NICHT_VORHANDEN"
        door_reason = "Keine Tür auf dem Bild erkannt."

        all_doors = opened_doors + closed_doors
        if all_doors:
            # Haupttür im Vordergrund wählen
            target_door = max(all_doors, key=lambda d: d["area"])
            if target_door["class_name"] == "opened door":
                door_state = "OFFEN"
                door_reason = "Geöffnete Tür / Durchgang im Vordergrund erkannt."
            else:
                door_state = "GESCHLOSSEN"
                door_reason = "Geschlossene Tür im Vordergrund erkannt."

        # ==========================================
        # 3. ZUSAMMENFASSUNG & SPEICHERUNG
        # ==========================================
        report = {
            "image_name": image_file.name,
            "window_status": {
                "state": window_state,
                "reason": window_reason,
                "detected_windows_count": len(windows),
                "target_window_bbox": target_window["bbox"]
                if target_window
                else None,
            },
            "door_status": {
                "state": door_state,
                "reason": door_reason,
                "detected_doors_count": len(all_doors),
            },
            "door_handles_count": len(door_handles),
            "total_detections_count": len(all_detections),
            "detections": all_detections,
        }

        # Visualisiertes Bild mit Bounding Boxes speichern
        if save_annotated:
            out_path = Path(output_dir)
            out_path.mkdir(parents=True, exist_ok=True)
            annotated_frame = results.plot()
            saved_file = out_path / f"annotated_{image_file.name}"
            cv2.imwrite(str(saved_file), annotated_frame)
            report["annotated_image_path"] = str(saved_file)

        return report


def print_inspection_report(report: Dict[str, Any]) -> None:
    """Gibt den Inspektionsbericht sauber strukturiert in der Konsole aus."""
    print("=" * 65)
    print(f" GO2 INSPEKTIONSERGEBNIS: {report['image_name']}")
    print("=" * 65)

    win = report["window_status"]
    print(f" FENSTER-STATUS : {win['state']}")
    print(f"   -> Details   : {win['reason']}")
    print(f"   -> Anzahl    : {win['detected_windows_count']} Fenster erkannt")

    door = report["door_status"]
    print(f" TÜR-STATUS     : {door['state']}")
    print(f"   -> Details   : {door['reason']}")
    print(f"   -> Anzahl    : {door['detected_doors_count']} Tür(en) erkannt")

    print(
        f" TÜRKLINKEN     : {report['door_handles_count']} Stück (ignoriert für Fenster)"
    )
    print("-" * 65)

    if report["detections"]:
        print(" Erkannte Objekte im Detail:")
        for det in report["detections"]:
            print(
                f"  - [{det['class_name']}] Conf: {det['confidence']*100:.1f}% | Box: {det['bbox']}"
            )
    else:
        print("  - (Keine Objekte erkannt / Negative Sample)")

    print("=" * 65 + "\n")


if __name__ == "__main__":
    import sys

    # Standarddateien für den manuellen Aufruf
    MODEL_FILE = "best.onnx"
    TEST_IMAGE = "test.jpg"

    if len(sys.argv) > 1:
        TEST_IMAGE = sys.argv[1]
    if len(sys.argv) > 2:
        MODEL_FILE = sys.argv[2]

    print(
        f"Starte GO2-Inspektion | Modell: '{MODEL_FILE}' | Bild: '{TEST_IMAGE}'"
    )

    try:
        detector = GO2InspectionDetector(
            model_path=MODEL_FILE, conf_threshold=0.20, imgsz=1024
        )
        inspection_result = detector.analyze_image(
            TEST_IMAGE, save_annotated=True
        )
        print_inspection_report(inspection_result)
    except Exception as err:
        print(f"Fehler bei der Inspektion: {err}")
