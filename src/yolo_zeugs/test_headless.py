import cv2
from ultralytics import YOLO

# 1. ONNX-Modell laden
model = YOLO("best.onnx")

# 2. Inferenz auf ein Bild oder Video ausführen
# (Ersetzen Sie 'test.jpg' durch ein Bild in Ihrem Ordner oder einer URL)
results = model("IMG_1944.JPEG", conf=0.20)

# 3. Ergebnisse im Terminal ausgeben
for result in results:
    for box in result.boxes:
        cls_id = int(box.cls[0])
        class_name = model.names[cls_id]
        conf = float(box.conf[0])
        print(f"-> Erkannt: {class_name} ({conf:.1%})")

# 4. Angetagtes Bild lokal auf dem Pi speichern (ohne Anzeige)
annotated_frame = results[0].plot()
cv2.imwrite("output_result.jpg", annotated_frame)
print("Ergebnisbild wurde unter 'output_result.jpg' gespeichert.")
