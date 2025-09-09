# Berry-Pick-UR3e
This repository hosts the development of a computer vision and robotics pipeline for automated strawberry harvesting.  
The project integrates a **UR3e collaborative robot**, a **custom-designed gripper**, and a **vision system** based on YOLOv8 to detect ripe strawberries for autonomous picking.  

---

Two main branches are maintained:  
- `vision` → version control for dataset preparation, training, and model evaluation.  
- `matlab` → control and simulation scripts for UR3e integration.  

---

## About the Project  

- **Vision**: Detect ripe strawberries in real-time using YOLOv8.  
- **Robot**: UR3e robot arm for precise manipulation.  
- **End-Effector**: Pneumatically actuated gripper with coupler and camera housing.  
- **Integration**: Vision-based detection to robot commands via ROS/Matlab API.  
- **IoT Monitoring**: Node-RED dashboard with MQTT integration to track plant **temperature and humidity** during tests.  

---

## Workflow  

1. Collect and annotate strawberry datasets.  
2. Train YOLOv8 models for maturity classification.  
3. Validate and optimize detection performance.  
4. Integrate with UR3e motion planning, pneumatic control, and gripper actuation.  
5. Deploy system for real-time strawberry picking tests.  
6. Monitor environment (temperature & humidity) in a Node-RED dashboard via MQTT.  

