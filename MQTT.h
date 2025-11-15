#include <WiFi.h>
#include <PubSubClient.h>
#include <max6675.h>

// -------------------------------------------------------------------
// --- Configuración Global (WiFi y MQTT) ---
// -------------------------------------------------------------------
const char* ssid = "SSID";
const char* password = "SSID PASWORD";
const char* mqtt_server_ip = "IP MQQT SERVER"; //Server running Mosquitto
const int mqtt_port = 1883; //Probably running in that port

// --- ID Único del Cliente MQTT ---
const char* mqtt_client_id = "mi_proyecto_final_ESP32_98765";

// Objetos globales
WiFiClient espClient;
PubSubClient client(espClient);

// -------------------------------------------------------------------
// --- 1. ABSTRACCIÓN DE ACTUADORES (ESCALABLE) ---
// -------------------------------------------------------------------
// Estructura para definir un actuador
struct Actuator {
  const int pin;
  const char* topic;
  bool state;
};

// --- NUEVO: Estructura para actuadores analógicos (PWM) ---
struct PwmActuator {
  const int pin;
  const char* topic;
  const int pwmChannel; // <-- Ya no se usa
  int value;
};

// (Pin 21, Tema nuevo, Canal PWM 0, Valor inicial 0)
PwmActuator luzLed_pin21 = {21, "area1/luz_pin21", 0, 0};

// --- Definición de tus actuadores ---
Actuator motor = {15, "area1/motor", LOW};
// Actuator ventilador = {16, "area1/ventilador", LOW}; // Ejemplo de cómo escalar

// -------------------------------------------------------------------
// --- 2. ABSTRACCIÓN DE SENSORES (ESCALABLE) ---
// -------------------------------------------------------------------
// Estructura para definir un sensor que publica periódicamente
struct Sensor {
  const char* topic;
  const long interval;
  long previousMillis;
  // Objeto sensor físico
  MAX6675* thermocouple; // Puntero al objeto sensor
};

// --- Objeto sensor físico ---
// ¡CAMBIO APLICADO! (SCK=6, CS=8, SO=7)
MAX6675 max_sensor(18, 5, 19); 

// --- Definición de tus sensores ---
Sensor sensorTemp = {"area1/temperatura", 5000, 0, &max_sensor};
// Sensor sensorLuz = {"area1/luz", 10000, 0, &objeto_sensor_luz}; // Ejemplo

// --- ¡NUEVO! Estructura para sensores analógicos simples ---
struct AnalogSensor {
  const int pin;
  const char* topic;
  const long interval;
  long previousMillis;
};

// ... (definición del max_sensor y sensorTemp) ...

// --- ¡NUEVO! Definición del sensor de luz (en un pin ADC1) ---
AnalogSensor sensorLuz = {34, "area1/luz", 1000, 0}; // Pin 34, publica cada 10 seg

// -------------------------------------------------------------------
// --- 3. LÓGICA DE MANEJO (SEPARACIÓN DE TAREAS) ---
// -------------------------------------------------------------------

// --- Tareas de Sensores ---
void publishTemperature() {
  // Usamos el puntero en la struct para leer el sensor
  double tempC = sensorTemp.thermocouple->readCelsius();

  if (isnan(tempC)) {
    Serial.println("Error al leer del MAX6675");
    return;
  }

  char tempString[8];
  dtostrf(tempC, 4, 2, tempString);

  client.publish(sensorTemp.topic, tempString);
  Serial.print("Publicado en ");
  Serial.print(sensorTemp.topic);
  Serial.print(": ");
  Serial.println(tempString);
}

// ... (Función publishTemperature) ...

// --- ¡NUEVO! Función para publicar el sensor de luz ---
void publishLight() {
  // ¡Así de simple se lee el ADC1!
  int valorRaw = analogRead(sensorLuz.pin); // Lee el pin 34

  char payload[6];
  sprintf(payload, "%d", valorRaw);
  client.publish(sensorLuz.topic, payload);
  
  Serial.print("Publicado en ");
  Serial.print(sensorLuz.topic);
  Serial.print(": ");
  Serial.println(payload);
}

// --- NUEVO: Función para manejar comandos de actuadores PWM ---
void handlePwmActuatorCommand(PwmActuator &act, String message) {
  // Convertir el payload (String) a un número (int)
  int brightness = message.toInt();

  // Limitar el valor al rango PWM (0-255)
  brightness = constrain(brightness, 0, 255);
  
  act.value = brightness;
  
  // --- REEMPLAZO ---
  // Escribe el valor (0-255) directamente al pin
  analogWrite(act.pin, act.value);

  Serial.print("Actuador PWM [");
  Serial.print(act.topic);
  Serial.print("] establecido a ");
  Serial.println(act.value);
}

// --- Esta función revisa TODOS los sensores y publica si es necesario ---
void handleSensorTasks() {
  unsigned long currentMillis = millis();

  // Revisar Sensor de Temperatura
  if (currentMillis - sensorTemp.previousMillis >= sensorTemp.interval) {
    sensorTemp.previousMillis = currentMillis; // Reiniciar temporizador
    publishTemperature();
  }

  // --- ¡NUEVO! Revisar Sensor de Luz ---
  if (currentMillis - sensorLuz.previousMillis >= sensorLuz.interval) {
    sensorLuz.previousMillis = currentMillis;
    publishLight();
  }
}

// --- Tareas de Actuadores ---
// Esta función genérica maneja el comando para CUALQUIER actuador
void handleActuatorCommand(Actuator &act, String message) {
  if (message == "ON") {
    act.state = HIGH;
    Serial.print("Actuador [");
    Serial.print(act.topic);
    Serial.println("] ENCENDIDO");
  } else if (message == "OFF") {
    act.state = LOW;
    Serial.print("Actuador [");
    Serial.print(act.topic);
    Serial.println("] APAGADO");
  } else if (message == "ALERTA") {
    Serial.print("ALERTA TEMPERATURA ELEVADA!!");
    digitalWrite(25, HIGH);
    delay(500);
    digitalWrite(25, LOW);
    act.state = HIGH;
  } else if (message == "NORMAL") {
    Serial.print("TEMPERATURA NORMAL");
    act.state = LOW;
  }
  digitalWrite(act.pin, act.state);
}

// -------------------------------------------------------------------
// --- 4. FUNCIONES DE CONECTIVIDAD (MQTT Y WIFI) ---
// -------------------------------------------------------------------

// El Callback ahora es un "despachador". Solo reenvía el trabajo.
void callback(char* topic, byte* payload, unsigned int length) {
  String s_topic = String(topic);
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Mensaje recibido en: ");
  Serial.println(s_topic);

  // Revisa qué actuador coincide con el tema
  if (s_topic == motor.topic) {
    handleActuatorCommand(motor, message);
  } else if (s_topic == sensorTemp.topic) {
    handleActuatorCommand(motor, message);
  } // ... (después de los otros 'else if')
    else if (s_topic == luzLed_pin21.topic) {
    handlePwmActuatorCommand(luzLed_pin21, message);
  }
  // Para escalar, añade un 'else if'
  // else if (s_topic == ventilador.topic) { ... }
}

void setup_wifi() {
  delay(10);
  Serial.print("\nConectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n¡WiFi conectado!");
  Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("¡Conectado!");
      
      // --- Suscripción dinámica ---
      client.subscribe(motor.topic);
      Serial.print("Suscrito a: ");
      Serial.println(motor.topic);

      // --- Suscripción dinámica ---
      client.subscribe(sensorTemp.topic);
      Serial.print("Suscrito a: ");
      Serial.println(sensorTemp.topic);

      // --- Suscripción al control de luz (Pin 21) ---
      client.subscribe(luzLed_pin21.topic);
      Serial.print("Suscrito a: ");
      Serial.println(luzLed_pin21.topic);

      // client.subscribe(ventilador.topic); // Ejemplo
      // Serial.println(ventilador.topic); // Ejemplo

    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" ... Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

// -------------------------------------------------------------------
// --- 5. INICIALIZACIÓN DE HARDWARE (CORREGIDO) ---
// -------------------------------------------------------------------

// --- Función para inicializar TODOS los actuadores ---
void setupActuators() {
  pinMode(motor.pin, OUTPUT);
  digitalWrite(motor.pin, motor.state);
  //Buzzer
  pinMode(25, OUTPUT);
  // pinMode(ventilador.pin, OUTPUT); // Ejemplo
  // --- Configurar el actuador PWM (Luz Pin 21) ---
  // analogWrite() en el ESP32 configura el pin, el canal 
  // y la frecuencia automáticamente.
  // Empezamos con el valor inicial (apagado).
  analogWrite(luzLed_pin21.pin, luzLed_pin21.value);
}

// --- ¡FUNCIÓN CORREGIDA Y AÑADIDA! ---
void setupSensors() {
  // Configurar pines del MAX6675
  pinMode(5, OUTPUT); // SCK 
  pinMode(18, OUTPUT); // CS (Pin 8)
  pinMode(19, INPUT);  // SO

  // --- ¡NUEVO! Configurar el pin del sensor analógico ---
  pinMode(sensorLuz.pin, INPUT);
  
  // Asegurarse que el Chip Select comience en ALTO (inactivo)
  digitalWrite(8, HIGH); // (Pin 8)
  
  // Para escalar, aquí irían los pinMode de otros sensores
}


// -------------------------------------------------------------------
// --- 6. SETUP Y LOOP (LIMPIOS Y ORDENADOS) ---
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  // --- Inicializar hardware ---
  setupActuators(); // Configura los pines de salida
  setupSensors();   // Configura los pines del sensor

  // --- Conectar ---
  setup_wifi();
  client.setServer(mqtt_server_ip, mqtt_port);
  client.setCallback(callback); // Configura el "despachador"

  Serial.println("Setup completo. (Escalable y Corregido)");
  delay(500); // Pausa para estabilizar el MAX6675
}

void loop() {
  // 1. Mantener conexión MQTT
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop(); // VITAL: Revisa mensajes entrantes

  // 2. Ejecutar tareas de sensores
  handleSensorTasks(); // VITAL: Revisa y publica datos de sensores
}