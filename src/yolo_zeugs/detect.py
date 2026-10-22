import cv2
from ultralytics import YOLO

# 1. Modell laden (best.pt oder best.onnx)
model = YOLO('best.pt')

# 2. Kamerastream öffnen (0 = USB/CSI-Kamera oder RTSP-URL des Go2)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Kamerastream unterbrochen oder Frame nicht verfügbar.")
        break

    # 3. Objekt-Erkennung ausführen (50% Schwellenwert)
    results = model(frame, conf=0.5)

    # 4. Bounding Boxes auf das Bild zeichnen
    annotated_frame = results[0].plot()

    # 5. Live-Bild anzeigen
    cv2.imshow("Go2 YOLO Detection", annotated_frame)

    # Beenden mit der Taste 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
