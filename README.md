# Vision — Strawberry Harvesting with Cobot UR3 + Soft Gripper

## Description
This branch contains the **computer vision module** for the strawberry harvesting project using a UR3 cobot and a soft gripper.  
The goal is to decide, based on a strawberry dataset and a detection model, **whether the robot should harvest a strawberry** within a **dynamic Region of Interest (ROI)**.

### Workflow:
1. **Initial Detection**  
   - The *eye-in-hand* camera (mounted above the gripper) captures the scene.  
   - YOLOv8 detects strawberries and defines a dynamic ROI.  
2. **Movement & Harvesting**  
   - The cobot moves toward the detected strawberry.  
   - The soft gripper collects it.  
3. **Post-harvest Verification**  
   - The ROI is adjusted to focus inside the gripper.  
   - The system verifies whether the strawberry is actually inside the gripper.  
4. **State Flags**  
   - `strawberry_detected`  
   - `movement_executed`  
   - `harvest_successful`  
   - `cycle_reset`  

---

## Branch Contents
- `dataset/` → Strawberry dataset (organized by classes and annotations).  
- `train/` → YOLOv8 training scripts.  
- `logs/` → Training results, epoch metrics.  
- `weights/` → Trained weights (e.g., `best.pt`).  
- `notebooks/` → Jupyter notebooks for experiments.  
- `src/` → Inference code + UR3 integration.  

---

## YOLOv8 Metrics
(To be filled after training)

- mAP50: `...`  
- mAP50-95: `...`  
- Precision: `...`  
- Recall: `...`  
- Loss (box/cls/obj): `...`  

---

## Dataset
- Source: strawberry dataset collected in the lab.  
- Approx. size: `N images`.  
- Annotation format: YOLO.  
- Split: `train / val / test`.  
- Notes: different ripeness levels, partial occlusions due to foliage.  

---

## How to Train
```bash
# Train YOLOv8
yolo detect train data=dataset/data.yaml model=yolov8n.pt epochs=100 imgsz=640
