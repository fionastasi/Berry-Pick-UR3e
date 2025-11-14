#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

// ---------------------
// CONFIGURACIÓN WIFI
// ---------------------
//const char *ssid = "INFINITUM3IZ6_2.4";
//const char *password = "cPX5XShdr3";
const char *ssid = "esp32";
const char *password = "1234567890";
// ---------------------
// CONFIGURACIÓN MQTT
// ---------------------
const char* mqtt_server = "192.168.1.20";
const int mqtt_port = 1883;

const char* TOPIC_CMD = "gripper/cmd";
const char* TOPIC_STATUS = "gripper/estado";
const char* TOPIC_ESP32 = "ESP32";
const char* TOPIC_PRESION = "Presion";
const char* TOPIC_PWM = "PWM";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_ADS1115 ads;

// ---------------------
// Pines
// ---------------------
const int pumpPin = 2;     // Bomba (PWM)
const int valvePin = 5;    // Válvula (cerrada = LOW, abierta = HIGH)

// ---------------------
// Parámetros del control
// ---------------------
float pressure = 0.0f;
float press_min_tanque  = 53.0f;  
float press_min_gripper = 27.0f;  
float pres_deseada_tanque  = 60.0f;  
float pres_deseada_gripper = 40.0f;  

float kp_tanque = 0.25f, ki_tanque = 0.03f;
float kp_gripper = 0.35f, ki_gripper = 0.06f;

float error = 0.0f, p_signal = 0.0f, i_signal = 0.0f;
float i_error_tanque = 0.0f, i_error_gripper = 0.0f;
float controlSignal = 0.0f;   
float pwm_conv = 0.0f;
float dt = 0.0f;
unsigned long tiempoPrevio = 0, lastPrint = 0;

// Estado del sistema
bool modoGripper = false;
bool pumpEnabled = false;
bool closed_sent = false;

const float TOLERANCIA = 0.5f;
unsigned long gripperStartTime = 0;  
const unsigned long DELAY_VALIDACION_GRIPPER = 2000;

// ---------------------
// WIFI / MQTT
// ---------------------
void conectarWiFi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.print("IP local: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n No se pudo conectar a WiFi");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String mensaje;
  for (unsigned int i = 0; i < length; i++) mensaje += (char)payload[i];
  mensaje.trim();
  mensaje.toLowerCase();
  Serial.printf("[MQTT] Mensaje recibido en %s: %s\n", topic, mensaje.c_str());

  if (mensaje == "on") {
    pumpEnabled = true;
    modoGripper = false;
    i_error_tanque = 0.0f;
    closed_sent = false;
    digitalWrite(valvePin, LOW);
    Serial.println(">> ON: bomba encendida, control del TANQUE activado.");
  } 
  else if (mensaje == "cerrar") {
    modoGripper = true;
    i_error_gripper = 0.0f;
    closed_sent = false;
    gripperStartTime = millis();

    if (!pumpEnabled) pumpEnabled = true;
    digitalWrite(valvePin, HIGH);
    Serial.println(">> Received cerrar: válvula ABIERTA, control GRIPPER activado.");
  } 
  else if (mensaje == "abrir") {
    modoGripper = false;
    closed_sent = false;
    digitalWrite(valvePin, LOW);
    pumpEnabled = false;
    analogWrite(pumpPin, 0);
    delay(1000);
    client.publish(TOPIC_STATUS, "open");
    Serial.println(">> abrir: válvula CERRADA (libera aire), bomba apagada.");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32_Gripper")) {
      Serial.println(" conectado");
      client.subscribe(TOPIC_CMD);
      client.publish(TOPIC_ESP32, "ON");  // 🔹 Enviar ON al conectar
    } else {
      Serial.print(" fallo, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5s...");
      delay(5000);
    }
  }
}

// ---------------------
// SETUP
// ---------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW);
  analogWrite(pumpPin, 0);

  ads.begin();
  tiempoPrevio = millis();

  conectarWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("\n=== Control dual TANQUE / GRIPPER con MQTT ===");
  Serial.println("Comandos disponibles: ON, cerrar, abrir");
}

// ---------------------
// LOOP PRINCIPAL
// ---------------------
void loop() {
  if (!client.connected()) {
    client.publish(TOPIC_ESP32, "OFF"); // 🔹 Enviar OFF si se desconecta
    reconnect();
  }
  client.loop();

  unsigned long tiempoActual = millis();
  dt = (tiempoActual - tiempoPrevio) / 1000.0;
  tiempoPrevio = tiempoActual;

  int16_t adc0 = ads.readADC_SingleEnded(0);
  float volts_reading = ads.computeVolts(adc0);
  pressure = (50.0f * volts_reading) - 125.0f;

  float pres_deseada = modoGripper ? pres_deseada_gripper : pres_deseada_tanque;
  float kp = modoGripper ? kp_gripper : kp_tanque;
  float ki = modoGripper ? ki_gripper : ki_tanque;
  float press_min = modoGripper ? press_min_gripper : press_min_tanque;

  if (pumpEnabled) {
    error = pres_deseada - pressure;
    p_signal = kp * error;
    if (modoGripper) {
      i_error_gripper += error * dt;
      i_signal = ki * i_error_gripper;
    } else {
      i_error_tanque += error * dt;
      i_signal = ki * i_error_tanque;
    }

    controlSignal = p_signal + i_signal;
    float desiredPressure = press_min + controlSignal;

    if (modoGripper)
      pwm_conv = (5.28f * desiredPressure) + 6.99f;
    else
      pwm_conv = (3.79f * desiredPressure) + 17.63f;

    pwm_conv = constrain(pwm_conv, 0.0f, 255.0f);
    analogWrite(pumpPin, (int)pwm_conv);
  } else {
    pwm_conv = 0.0f;
    analogWrite(pumpPin, 0);
  }

  if (modoGripper && !closed_sent && millis() - gripperStartTime >= DELAY_VALIDACION_GRIPPER) {
    if (fabs(error) <= TOLERANCIA) {
      client.publish(TOPIC_STATUS, "closed");
      closed_sent = true;
    }
  }

  // 🔹 Publicar cada segundo presión y PWM
  static unsigned long lastMQTT = 0;
  if (millis() - lastMQTT >= 1000) {
    lastMQTT = millis();

    char presBuf[8], pwmBuf[8];
    dtostrf(pressure, 4, 2, presBuf);
    dtostrf(pwm_conv, 4, 0, pwmBuf);
    client.publish(TOPIC_PRESION, presBuf);
    client.publish(TOPIC_PWM, pwmBuf);
  }

  // Mostrar en serial
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print(modoGripper ? "Modo: GRIPPER" : "Modo: TANQUE");
    Serial.print(" | Presión: "); Serial.print(pressure, 2);
    Serial.print(" kPa | PWM: "); Serial.print(pwm_conv, 1);
    Serial.print(" | Error: "); Serial.println(error, 2);
  }
}
