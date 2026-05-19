#include <WiFi.h> // Libreria principal para el manejo de conexiones WiFi en el ESP32
#include <MPU6500_WE.h> // Libreria utilizada para controlar y leer el sensor MPU6500
#include <SPI.h> // Libreria necesaria para la comunicacion SPI
#include <Wire.h> // Libreria para comunicacion I2C (aunque aqui se usa SPI, algunas librerias la requieren)
#include <PicoMQTT.h> // Libreria para crear y manejar un servidor MQTT local

//////////////////// MQTT ////////////////////

PicoMQTT::Server mqtt; // Creacion del servidor MQTT local

String IP; // Variable donde se almacenara la direccion IP del Access Point creado por el ESP32

//////////////////// SPI ////////////////////

// Pines usados para la comunicacion SPI entre el ESP32-C6 y el MPU6500
const int csPin   = 7; // Chip Select: habilita el dispositivo SPI
const int mosiPin = 5; // Master Out Slave In: datos enviados del ESP32 al MPU6500
const int misoPin = 4; // Master In Slave Out: datos enviados del MPU6500 al ESP32
const int sckPin  = 6; // Serial Clock: señal de reloj SPI

//////////////////// MOTOR ////////////////////

const int mot = 8; // Pin conectado al controlador o transistor del motor

// PWM ESP32-C6
const int pwmFreq = 1000; // Frecuencia PWM en Hz utilizada para controlar el motor
const int pwmResolution = 8; // Resolucion PWM de 8 bits (valores de 0 a 255)

//////////////////// PID ////////////////////

float ref = 0; // Referencia deseada del sistema (angulo objetivo)

float kp = 1; // Ganancia proporcional
float ki = 0.5; // Ganancia integral
float kd = 2; // Ganancia derivativa

float e  = 0; // Error actual
float ei = 0; // Error integral acumulado
float ea = 0; // Error anterior
float ed = 0; // Error derivativo
float u  = 0; // Salida final del controlador PID

//////////////////// MPU6500 ////////////////////

bool useSPI = true; // Indica que el sensor MPU6500 utilizara comunicacion SPI

// Constructor del objeto MPU6500
// Se le pasa:
// - referencia al bus SPI
// - pin CS
// - pin MOSI
// - pin MISO
// - pin SCK
// - true indicando uso de SPI personalizado
MPU6500_WE myMPU6500(
  &SPI,
  csPin,
  mosiPin,
  misoPin,
  sckPin,
  true
);

//////////////////// ANGULOS ////////////////////

float ang_x = 0; // Angulo filtrado en el eje X
float ang_y = 0; // Angulo filtrado en el eje Y
float ang_yy = 0; // Variable auxiliar del angulo Y invertido

float ang_x_prev = 0; // Valor anterior del angulo X
float ang_y_prev = 0; // Valor anterior del angulo Y

float dt = 0; // Tiempo de muestreo entre iteraciones del loop

unsigned long tiempo_prev = 0; // Variable que almacena el tiempo anterior en milisegundos

///////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200); // Inicializacion de la comunicacion serial a 115200 baudios

  ////////////////// SPI //////////////////

  // Inicializacion manual del bus SPI indicando los pines personalizados
  SPI.begin(sckPin, misoPin, mosiPin, csPin);

  ////////////////// MOTOR PWM //////////////////

  pinMode(mot, OUTPUT); // Configuracion del pin del motor como salida

  // Configuracion del PWM del ESP32-C6
  // Se asigna:
  // - pin del motor
  // - frecuencia PWM
  // - resolucion PWM
  ledcAttach(mot, pwmFreq, pwmResolution);

  ////////////////// WIFI AP //////////////////

  WiFi.mode(WIFI_AP); // Configura el ESP32 como Access Point

  // Creacion de la red WiFi local
  // Parametros:
  // - nombre de la red
  // - contraseña
  // - canal WiFi
  // - ocultar SSID (false = visible)
  // - numero maximo de clientes
  WiFi.softAP(
    "Jhojan_Broker",
    "123456789",
    1,
    false,
    4
  );

  // Obtiene la direccion IP del Access Point
  IPAddress ip = WiFi.softAPIP();

  // Convierte la IP a texto y la guarda
  IP = ip.toString();

  // Muestra la direccion IP en el monitor serial
  Serial.print("IP AP: ");
  Serial.println(IP);

  ////////////////// MPU6500 //////////////////

  // Inicializa el sensor MPU6500
  if (!myMPU6500.init()) {

    // Si el sensor no responde se muestra el mensaje
    Serial.println("MPU6500 no responde");

    // Bucle infinito para detener el programa
    while (1);
  }

  // Mensaje indicando que el sensor fue detectado correctamente
  Serial.println("MPU6500 conectado");

  delay(1000); // Pequeña pausa antes de calibrar

  Serial.println("Calibrando... no mover");

  // Calibracion automatica de offsets del acelerometro y giroscopio
  myMPU6500.autoOffsets();

  Serial.println("Calibracion terminada");

  ////////////////// CONFIGURACION MPU //////////////////

  // Activa el filtro digital pasa bajos del giroscopio
  myMPU6500.enableGyrDLPF();

  // Configura el filtro digital del giroscopio
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);

  // Activa el filtro digital del acelerometro
  myMPU6500.enableAccDLPF(true);

  // Configura el filtro digital del acelerometro
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);

  // Divide la frecuencia de muestreo interna
  myMPU6500.setSampleRateDivider(5);

  // Configura el rango maximo del giroscopio a ±250 grados/segundo
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);

  // Configura el rango maximo del acelerometro a ±2G
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);

  ////////////////// MQTT //////////////////

  // Suscripcion al topic "angulo"
  // Cada vez que llegue un dato se ejecuta esta funcion lambda
  mqtt.subscribe("angulo", [](const char * payload) {

    // Convierte el texto recibido a float y actualiza la referencia
    ref = atof(payload);

  });

  // Inicializa el servidor MQTT
  mqtt.begin();

  ////////////////// TIEMPO //////////////////

  // Guarda el tiempo inicial para el calculo del delta t
  tiempo_prev = millis();
}

///////////////////////////////////////////////////////

void loop() {

  // Mantiene funcionando el servidor MQTT
  mqtt.loop();

  ////////////////// LECTURA MPU //////////////////

  // Lectura de aceleraciones en los ejes X, Y y Z
  xyzFloat acc = myMPU6500.getGValues();

  // Lectura de velocidades angulares del giroscopio
  xyzFloat gyr = myMPU6500.getGyrValues();

  ////////////////// DELTA T //////////////////

  // Obtiene el tiempo actual
  unsigned long tiempo_actual = millis();

  // Calcula el tiempo transcurrido entre iteraciones en segundos
  dt = (tiempo_actual - tiempo_prev) / 1000.0;

  // Actualiza el tiempo anterior
  tiempo_prev = tiempo_actual;

  ////////////////// ACELEROMETRO //////////////////

  // Calcula el angulo en Y usando acelerometro
  // atan2 mejora estabilidad y evita divisiones por cero
  float accel_ang_y =
    atan2(
      acc.y,
      sqrt(acc.x * acc.x + acc.z * acc.z)
    ) * 180.0 / PI;

  // Calcula el angulo en X usando acelerometro
  float accel_ang_x =
    atan2(
      -acc.x,
      sqrt(acc.y * acc.y + acc.z * acc.z)
    ) * 180.0 / PI;

  ////////////////// FILTRO COMPLEMENTARIO //////////////////

  // Fusion del acelerometro y giroscopio para el eje X
  // 98% giroscopio + 2% acelerometro
  ang_x =
    0.98 * (ang_x_prev + gyr.y * dt)
    + 0.02 * accel_ang_x;

  // Fusion del acelerometro y giroscopio para el eje Y
  ang_y =
    0.98 * (ang_y_prev + gyr.x * dt)
    + 0.02 * accel_ang_y;

  // Guarda el valor actual para la siguiente iteracion
  ang_x_prev = ang_x;

  // Guarda el valor actual para la siguiente iteracion
  ang_y_prev = ang_y;

  ////////////////// PID //////////////////

  // Se invierte el eje Y para facilitar el control y visualizacion
  ang_yy = -1 * ang_y;

  // Error actual = referencia - valor actual
  e = ref - ang_yy;

  // Error derivativo
  ed = e - ea;

  // Error integral acumulado corregido usando dt
  ei += e * dt;

  // Ecuacion del controlador PID
  u =
    (kp * e)
    + (ki * ei)
    + (kd * ed);

  ////////////////// LIMITADOR //////////////////

  // Saturacion superior del PWM
  if (u > 255) {
    u = 255;
  }

  // Saturacion inferior del PWM
  if (u < 0) {
    u = 0;
  }

  ////////////////// PWM //////////////////

  // Envia el valor PWM al motor
  ledcWrite(mot, (int)u);

  // Guarda el error actual para la siguiente iteracion
  ea = e;

  ////////////////// SERIAL //////////////////

  // Envio de datos seriales para monitoreo o graficacion
  Serial.print(ref);
  Serial.print(" ");

  Serial.print(e);
  Serial.print(" ");

  Serial.print(u);
  Serial.print(" ");

  Serial.println(ang_yy);

  ////////////////// PEQUEÑO DELAY //////////////////

  // Pequeña pausa para estabilidad del sistema
  delay(2);
}