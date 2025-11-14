# Berry-Pick-UR3e

This repository hosts the development of a **computer vision and robotics pipeline** for automated strawberry harvesting.  
The project integrates a **UR3e collaborative robot**, a **custom-designed gripper**, and a **vision system** based on YOLOv8 to detect ripe strawberries for autonomous picking.

---

## Branches

Two main branches are maintained:

- `vision` → Dataset preparation, model training, and evaluation.  
- `gripper` → ISO drawings and specifications

---

## Project Overview

- **Vision**: Real-time detection of ripe strawberries using YOLOv8.  
- **Robot**: UR3e robot arm for precise manipulation.  
- **End-Effector**: Pneumatically actuated gripper with coupler and camera housing.  
- **Integration**: Vision-based detection triggers robot commands via ROS/Matlab API.  
- **IoT Monitoring**: Node-RED dashboard with MQTT integration to track **temperature and humidity** during tests.

---

## Workflow

1. Collect and annotate strawberry datasets.  
2. Train YOLOv8 models for maturity classification.  
3. Validate and optimize detection performance.  
4. Integrate detection with UR3e motion planning, pneumatic control, and gripper actuation.  
5. Deploy system for real-time strawberry picking tests.  
6. Monitor environment (temperature & humidity) via Node-RED dashboard using MQTT.

## System Demonstration Video
[![System Demo](preview.gif)](https://www.youtube.com/watch?v=Yzso3GYcB5o)
[![System Demo](https://img.youtube.com/vi/Yzso3GYcB5o/maxresdefault.jpg)](https://www.youtube.com/watch?v=Yzso3GYcB5o)
