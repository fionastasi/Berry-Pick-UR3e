# pod_capture_then_detect_v4_auto.py (modificado con recuadro verde)
import os, cv2, json, time, numpy as np
import depthai as dai
from ultralytics import YOLO
import paho.mqtt.client as mqtt

# === Configuración MQTT ===
BROKER = "localhost"   # o la IP de tu broker
PORT   = 1883
TOPIC_CMD = "UR3e/cmd"

# Crear cliente MQTT
client = mqtt.Client()
client.connect(BROKER, PORT, 60)
client.loop_start()  # inicia el loop en hilo separado

IMG_W = IMG_H = 640
SAVE_DIR = "captures"
SAVE_DIR2 = "C:/Users/jorge/Vision"
RAW_PATH = os.path.join(SAVE_DIR, "shot.png")
DET_PATH = os.path.join(SAVE_DIR, "shot_detected.png")
DET_PATH = os.path.join(SAVE_DIR2, "shot_detected.png")
DEPTH_PATH = os.path.join(SAVE_DIR, "shot_depth.png")
K_PATH = os.path.join(SAVE_DIR, "intrinsics.json")
POSE_PATH = os.path.join(SAVE_DIR, "pose.txt")
POSE_MATRIX_JSON = os.path.join(SAVE_DIR, "pose_matrix.json")
MODEL_PATH = "runs/detect/strawberry_exp/weights/best.pt"
CONF_THR = 0.25
DEPTH_KERNEL = 7
os.makedirs(SAVE_DIR, exist_ok=True)

MODE = "delay"
DELAY_SECONDS = 3

def _median_depth(depth_mm, u, v, k=7, bbox=None):
    h, w = depth_mm.shape[:2]
    if bbox is not None:
        x1,y1,x2,y2 = map(int, bbox)
        bw, bh = x2-x1, y2-y1
        ex,ey = max(0,int(0.25*bw)), max(0,int(0.25*bh))
        xa, xb = max(0,x1+ex), min(w-1,x2-ex)
        ya, yb = max(0,y1+ey), min(h-1,y2-ey)
        if xb>xa and yb>ya:
            roi = depth_mm[ya:yb, xa:xb]
            valid = roi[(roi>0)&(roi<10000)]
            if valid.size>=20:
                return float(np.percentile(valid,20))
    k2 = k//2
    xa, xb = max(0,u-k2), min(w-1,u+k2)
    ya, yb = max(0,v-k2), min(h-1,v+k2)
    roi = depth_mm[ya:yb+1, xa:xb+1]
    valid = roi[(roi>0)&(roi<10000)]
    if valid.size:
        return float(np.percentile(valid,30))
    return 0.0

def _deproject(u, v, Z, fx, fy, cx, cy):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return float(X), float(Y), float(Z)

def take_photo():
    with dai.Pipeline() as p:
        cam = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        preview = cam.requestOutput((IMG_W, IMG_H))
        monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        monoL_out = monoL.requestOutput((640, 400))
        monoR_out = monoR.requestOutput((640, 400))
        stereo = p.create(dai.node.StereoDepth)
        monoL_out.link(stereo.left)
        monoR_out.link(stereo.right)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(IMG_W, IMG_H)

        q_rgb   = preview.createOutputQueue(maxSize=4, blocking=False)
        q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        print("Iniciando cámara…")
        p.start()

        frame = None
        depth = None
        start_time = None

        while True:
            imgFrame = q_rgb.tryGet()
            dmsg     = q_depth.tryGet()
            if imgFrame is not None:
                frame = imgFrame.getCvFrame()
            if dmsg is not None:
                depth_raw = dmsg.getFrame()
                depth = cv2.resize(depth_raw, (IMG_W, IMG_H), interpolation=cv2.INTER_NEAREST) if depth_raw.shape[:2] != (IMG_H, IMG_W) else depth_raw

            if frame is not None and depth is not None:
                disp = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(depth,0,3000)/3000.0*255), cv2.COLORMAP_TURBO)
                cv2.imshow("OAK-D | Preview RGB + Depth", np.hstack((frame, disp)))

            if MODE=="delay":
                if start_time is None:
                    start_time = time.time()
                    print(f"Esperando {DELAY_SECONDS}s antes de capturar…")
                if (time.time() - start_time) >= DELAY_SECONDS:
                    print("Capturando foto automáticamente…")
                    break
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Cancelado por el usuario.")
                    cv2.destroyAllWindows()
                    return None, None, None

        cv2.destroyAllWindows()
        cv2.imwrite(RAW_PATH, frame)
        cv2.imwrite(DEPTH_PATH, depth)
        print(f"Foto guardada: {RAW_PATH}")
        print(f"Depth guardado: {DEPTH_PATH}")

    try:
        with dai.Device() as dev:
            calib = dev.readCalibration()
            K = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, IMG_W, IMG_H)
            Fx, Fy, Cx, Cy = float(K[0][0]), float(K[1][1]), float(K[0][2]), float(K[1][2])
    except Exception:
        Fx = Fy = 615.0; Cx = Cy = 320.0

    with open(K_PATH, "w") as f:
        json.dump({"fx": Fx, "fy": Fy, "cx": Cx, "cy": Cy}, f, indent=2)
    print(f"Intrínsecos guardados: {K_PATH}")
    return frame, depth, (Fx, Fy, Cx, Cy)

def detect_on_file(img_path, depth_mm, K):
    print("Procesando con YOLOv8…")
    model = YOLO(MODEL_PATH)
    img = cv2.imread(img_path)
    results = model.predict(img, imgsz=IMG_W, conf=CONF_THR, verbose=False)
    annotated = results[0].plot()
    found_all = []
    Fx, Fy, Cx, Cy = K

    ripe_detections = []
    unripe_detections = []

    if results and len(results[0].boxes) > 0:
        boxes = results[0].boxes
        xyxy = boxes.xyxy.cpu().numpy()
        cls  = boxes.cls.cpu().numpy().astype(int)
        conf = boxes.conf.cpu().numpy()
        names = model.names if hasattr(model, 'names') else {}

        for i, ((x1, y1, x2, y2), c, s) in enumerate(zip(xyxy, cls, conf)):
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            u = int((x1 + x2)/2)
            v = int((y1 + y2)/2)
            Z = _median_depth(depth_mm, u, v, DEPTH_KERNEL, bbox=(x1,y1,x2,y2))
            X, Y, Zf = _deproject(u, v, Z, Fx, Fy, Cx, Cy)

            w, h = x2-x1, y2-y1
            aspect = w/h if h>0 else 1.0
            yaw = pitch = roll = 0.0
            if aspect>1.2: yaw = np.deg2rad(10)
            elif aspect<0.8: yaw = np.deg2rad(-10)
            Rx = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
            Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
            Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
            R = Rz @ Ry @ Rx

            R_corr = np.array([[0,1,0],[0,0,-1],[-1,0,0]])
            R_final = R @ R_corr

            T = np.eye(4)
            T[:3,:3] = R_final
            T[:3,3] = [X, Y, Zf]

            cv2.putText(annotated, f"Z:{Zf:.0f}mm", (x1,max(0,y1-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

            cls_name = str(names.get(c, str(c))).lower()

            det = {"cls": int(c),"cls_name": cls_name,"conf": float(s),"bbox": [x1,y1,x2,y2],"u": u, "v": v,"Z_mm": float(Zf),"X_cam_mm": X,"Y_cam_mm": Y,"pose_matrix": T.tolist()}
            found_all.append(det)

            if 'unripe' in cls_name:
                unripe_detections.append(det)
            elif 'ripe' in cls_name:
                ripe_detections.append(det)
            else:
                if 'un' in cls_name and 'ripe' in cls_name:
                    unripe_detections.append(det)
                elif 'ripe' in cls_name:
                    ripe_detections.append(det)
                else:
                    ripe_detections.append(det)

    cv2.imwrite(DET_PATH, annotated)

    if len(found_all) == 0:
        with open(POSE_PATH, "w") as f:
            f.write(f"K: {K}\nDetections: 0\n")
        print(f"Detecciones guardadas: {DET_PATH}")
        print(f"Log XYZ + matriz 4x4 guardado: {POSE_PATH}")
        return annotated, results[0], found_all

    if len(ripe_detections) == 0 and len(unripe_detections) > 0:
        print("Se detectaron solo fresas NO maduras (unripe). No se guardará la pose.")
        print("fresa no lista/madura")
        cv2.imshow("Detección YOLOv8 + Z/XYZ", annotated)
        print("Presiona 'q' para cerrar la vista de la fresa no madura.")
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
        return annotated, results[0], []

    if len(ripe_detections) > 0:
        best = max(ripe_detections, key=lambda d: d['conf'])

        # Dibujar recuadro verde en la fresa ripe seleccionada
        x1, y1, x2, y2 = best['bbox']
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(annotated, "RIPE SELECTED", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imwrite(DET_PATH, annotated)

        with open(POSE_MATRIX_JSON, "w") as f:
            json.dump(best['pose_matrix'], f, indent=2)
        print(f"Pose 4x4 de la fresa (ripe) guardada en {POSE_MATRIX_JSON}")

        with open(POSE_PATH, "w") as f:
            f.write(f"K: {K}\nDetections_saved: 1\n")
            f.write(json.dumps(best, indent=2) + "\n")

        print(f"Detecciones guardadas: {DET_PATH}")
        print(f"Log XYZ + matriz 4x4 guardado: {POSE_PATH}")

        return annotated, results[0], [best]

    return annotated, results[0], []

if __name__ == "__main__":
    import paho.mqtt.client as mqtt

    # Configuración MQTT local
    client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.loop_start()
    TOPIC_CMD = "UR3e/cmd"

    start_time = time.time()
    timeout_sec = 30  # Timeout máximo para detectar fresa

    while True:
        frame, depth_mm, K = take_photo()  # No se cambia la función
        if frame is None:
            # Timeout interno de take_photo() o cancelación
            print("[INFO] Timeout interno o captura cancelada.")
            client.publish(TOPIC_CMD, "Reset")
            break

        annotated, res, found = detect_on_file(RAW_PATH, depth_mm, K)

        if found:
            # Se detectó fresa
            cv2.imshow("Detección YOLOv8 + Z/XYZ", annotated)
            print("Detección exitosa. Cerrando ventana automáticamente en 1 segundo...")
            cv2.waitKey(1000)  # Espera 1 segundo (1000 ms)
            cv2.destroyAllWindows()
            break
        else:
            # No se detectó fresa; revisamos si pasó el timeout
            if (time.time() - start_time) > timeout_sec:
                print(f"[TIMEOUT] No se detectó fresa en {timeout_sec} segundos.")
                client.publish(TOPIC_CMD, "Reset")
                break
            else:
                print("No se detectó nada o sólo había fresas no maduras -> reintentando...")

    # limpieza final
    client.loop_stop()
    client.disconnect()
    print("Cliente MQTT finalizado. Exit.")


