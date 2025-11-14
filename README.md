# Vision Module — Berry-Pick-UR3e  
**Strawberry Harvesting with UR3 Cobot + Soft Gripper**

## Overview

This branch contains the complete computer vision pipeline used for object detection and 3D localization in the berry-picking system. It includes the trained YOLO model, training metrics, inference scripts, dataset tools, and all generated outputs.

The goal of the vision subsystem is to:

- Capture RGB and depth frames from the OAK-D camera
- Detect ripe/unripe objects using a custom YOLO model
- Compute real-world 3D coordinates using the pinhole model and depth
- Provide the robot with accurate poses through JSON messaging

This branch operates independently but is fully integrated with the main robotic workflow and the dashboard simulation.

---

## Core Pipeline

The harvesting sequence calls the script `test_POD_PnP5.py`, which performs the following steps:

1. **Image Acquisition**
   - Connects to the **OAK-D Pro** camera (eye-in-hand configuration).
   - Captures synchronized **RGB (pinhole)** and **depth** frames.
   - Generates a color map for visualization.

2. **Strawberry Detection**
   - Runs inference using a trained **YOLOv8** model.
   - Detects strawberries and classifies them as `ripe` or `unripe`.
   - If multiple ripe strawberries are found, selects the one with the **highest confidence** and highlights it with a **green bounding box**.
   - Unripe-only cases automatically block the harvesting action and request a new frame.

3. **Pose Estimation**
   - Retrieves camera **intrinsics**.
   - Extracts the **center of the selected bounding box**
   - Computes the 3D position using **pinhole + depth**.
   - This matrix is used to guide the UR3 robot to the target strawberry.

4. **Harvesting Sequence**
   - The robot moves toward the selected strawberry.
   - The soft gripper performs the harvesting action.

5. **Post-Harvest Verification**
   - The system re-scans the scene to validate harvesting success.

---

## Branch Contents

- `dataset/` → Custom strawberry dataset (annotated by ripeness level).  
- `train/` → YOLOv8 training scripts and configs.  
- `weights/` → Trained YOLOv8 weights (`best.pt`).    
- `test_POD_PnP5.py` → Main script for detection + localization + harvesting trigger.

---

## YOLOv8 Metrics  

- **mAP50:** ~0.72
- **mAP50-95:** ~0.47
- **Precision:** ~0.72–0.75
- **Recall:** ~0.73
- **Validation Box Loss:** ~1.03
- **Validation Cls Loss:** ~1.00
- **Validation DFL Loss:** ~1.12



