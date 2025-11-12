# Berry-Pick-UR3e — Dashboard de Control

Este dashboard forma parte del proyecto **Berry-Pick-UR3e**, un sistema funcional de cosecha automatizada de fresas desarrollado por el equipo **Alfresi** del **Tec de Monterrey, campus Querétaro**. La interfaz permite monitorear y controlar los distintos subsistemas del robot recolector, integrando visión artificial, control robótico, y sistemas electroneumáticos.

## Propósito del Dashboard

Este flujo en Node-RED centraliza el control y monitoreo del sistema, facilitando la interacción entre los módulos de hardware y software. Está diseñado para operar en tiempo real, tanto en modo manual como automático, y proporciona visualización clara del estado del sistema.

## Módulos del Dashboard

### 1. Estado General del Sistema
- **UR3e**: Estado de conexión y fase operativa
- **ESP32**: Estado de conexión y control del gripper neumático
- **Sistema**: Indicador de estado global (reposo, activo, error)
- **Gripper**: Estado del actuador (abierto/cerrado)

### 2. Sistema Electroneumático
- **Presión**: Lectura en tiempo real desde el sensor conectado al ESP32
- **PWM**: Control de señal para válvulas neumáticas
- **Botones de control**: Abrir / Cerrar el gripper

### 3. Visión Artificial
- **Última imagen procesada**: Captura de la cámara **OAK-D Pro** con detección de fresas
- **Transmisión en vivo**: Visualización continua del entorno de trabajo

### 4. Control del Robot
- **Botones de operación**: INICIAR, DETENER, RESET, AUTO
- **Integración con RoboDK**: Envío de comandos al UR3e para ejecutar trayectorias

## Integraciones

- **Node-RED**: Orquestación de flujos lógicos y visualización
- **Python**: Procesamiento de imágenes y lógica de detección
- **RoboDK**: Simulación y ejecución de trayectorias robóticas
- **Arduino + ESP32**: Control del sistema neumático
- **Mosquitto (MQTT)**: Comunicación entre módulos

## Estructura del flujo

El flujo está dividido en secciones temáticas:
- `Estado general del sistema`
- `Sistema electroneumático`
- `Visión`
- `Control del robot`

Cada sección contiene nodos de entrada (sensores, botones), funciones de procesamiento, y salidas visuales (LEDs, gráficos, imágenes).

## Estado del Proyecto

Este dashboard corresponde a un **prototipo funcional** en fase de validación académica. La rama `gripper` contiene el desarrollo del actuador neumático suave diseñado por el equipo.

## Créditos

Desarrollado por el equipo **Alfresi**  
**Tecnológico de Monterrey, Campus Querétaro**

---

