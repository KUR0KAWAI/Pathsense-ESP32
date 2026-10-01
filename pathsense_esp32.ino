#include "TinyGPS++.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "esp_mac.h"

// =====================================================
// PINES DE HARDWARE (ESP32-S3)
// =====================================================
const int PIN_TRIG   = 4;   // Trigger del sensor de ultrasonido
const int PIN_ECHO   = 5;   // Echo del sensor de ultrasonido
const int PIN_BUZZER = 6;   // Zumbador / Altavoz activo

const int PIN_GPS_RX = 18;  // Conectado al TXD del módulo GPS
const int PIN_GPS_TX = 17;  // Conectado al RXD del módulo GPS

// =====================================================
// CONFIGURACIÓN DE TIEMPOS Y UMBRALES
// =====================================================
const float DISTANCIA_MAX_CM = 150.0;             // Detección inicia a 1.50 m
const float DISTANCIA_MIN_CM = 40.0;              // Alerta máxima a 40 cm

const unsigned long INTERVALO_ULTRASONICO = 100;  // Muestreo rápido cada 100 ms
const unsigned long INTERVALO_ENVIO_JSON  = 1500; // Envío a la App cada 1.5 s

const int FRECUENCIA_TONO = 1000;                 // Tono suave (1000 Hz)

// =====================================================
// OPCIÓN B: UUIDs PROPIOS DEL FIRMWARE
// =====================================================
String bleDeviceName = "";
String deviceID      = "";

#define SERVICE_UUID "4fa1c000-1853-4d89-93db-08801990c98a"
#define TX_UUID      "d804b643-0c48-433b-8517-38f328909d93"
#define RX_UUID      "dcba4321-4321-4321-4321-ba0987654321"

// Objetos de control
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

BLEServer *servidorBLE = nullptr;
BLECharacteristic *txBLE = nullptr;
BLECharacteristic *rxBLE = nullptr;

// Control de estados internos
bool bluetoothConectado = false;
bool listoParaTransmitir = false; // Controla la pausa inicial de 2 segundos
float distanciaActual = -1.0;
bool estadoPanico = false;
unsigned long contadorSecuencia = 0;

unsigned long ultimaMedicionUltrasonico = 0;
unsigned long ultimoEnvioJSON = 0;
unsigned long tiempoConexionBLE = 0;

// Variables de temporización no bloqueante para el zumbador
unsigned long ultimoCambioBuzzer = 0;
bool estadoBuzzerEncendido = false;

// Generar nomenclatura única utilizando la MAC del chip
void generarNomenclatura() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char sufijoMac[5];
  snprintf(sufijoMac, sizeof(sufijoMac), "%02X%02X", mac[4], mac[5]);
  bleDeviceName = "PS-V1P-" + String(sufijoMac);
  deviceID      = "PS-BST-V1P-" + String(sufijoMac);
}

// Lectura del sensor de distancia
float medirDistancia() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, 12000);
  if (duracion == 0) return -1.0;
  return (duracion * 0.0343) / 2.0;
}

// Lógica de pitido progresivo sin bloquear el programa
void actualizarBuzzerProgresivo() {
  unsigned long ahora = millis();

  if (distanciaActual <= 0.0 || distanciaActual > DISTANCIA_MAX_CM) {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
    estadoBuzzerEncendido = false;
    return;
  }

  float d = distanciaActual;
  if (d < DISTANCIA_MIN_CM) d = DISTANCIA_MIN_CM;

  unsigned long intervaloBip = map((long)d, (long)DISTANCIA_MIN_CM, (long)DISTANCIA_MAX_CM, 70, 500);

  if (ahora - ultimoCambioBuzzer >= intervaloBip) {
    ultimoCambioBuzzer = ahora;
    estadoBuzzerEncendido = !estadoBuzzerEncendido;

    if (estadoBuzzerEncendido) {
      tone(PIN_BUZZER, FRECUENCIA_TONO);
    } else {
      noTone(PIN_BUZZER);
    }
  }
}

// Manejador estricto de eventos de conexión y desconexión BLE
class ServidorCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bluetoothConectado = true;
    listoParaTransmitir = false; // Bloquear envíos inmediatos
    contadorSecuencia = 0;       // REINICIO DE SECUENCIA A ZERO EN CADA NUEVA CONEXIÓN
    tiempoConexionBLE = millis(); // Registrar la hora exacta de conexión
    
    uint16_t connId = server->getConnId();
    
    Serial.println("\n==================================================");
    Serial.println("  [BLE] ¡DISPOSITIVO MÓVIL VINCULADO CON ÉXITO!");
    Serial.print("  [BLE] Conectado a ID interno BLE : ");
    Serial.println(connId);
    Serial.print("  [BLE] Dispositivo Local          : ");
    Serial.println(bleDeviceName);
    Serial.println("  [BLE] Secuencia reiniciada a 0.");
    Serial.println("  [BLE] Esperando 2 segundos de estabilización GATT...");
    Serial.println("==================================================");
  }

  void onDisconnect(BLEServer *server) override {
    bluetoothConectado = false;
    listoParaTransmitir = false;
    
    Serial.println("\n==================================================");
    Serial.println("  [BLE] Dispositivo móvil DESCONECTADO.");
    Serial.println("  [BLE] Transmisión JSON DETENIDA.");
    Serial.print("  [BLE] Anunciándose por Bluetooth como: ");
    Serial.println(bleDeviceName);
    Serial.println("  [BLE] Estado: ESPERANDO NUEVA VINCULACIÓN...");
    Serial.println("==================================================");

    delay(200);
    BLEDevice::startAdvertising(); // Reiniciar publicidad BLE
  }
};

// Enviar trama JSON a la App únicamente cuando hay conexión activa y se cumplió la espera de 2s
void enviarTramaJSON() {
  if (!bluetoothConectado || !listoParaTransmitir) return;

  contadorSecuencia++;

  bool gpsValido = gps.location.isValid();
  double latitud  = gpsValido ? gps.location.lat() : 0.0;
  double longitud = gpsValido ? gps.location.lng() : 0.0;
  double altitud  = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  int satelites   = gps.satellites.isValid() ? gps.satellites.value() : 0;

  String json = "{";
  json += "\"seq\":" + String(contadorSecuencia) + ",";
  json += "\"id\":\"" + deviceID + "\",";
  json += "\"nombre\":\"" + bleDeviceName + "\",";
  json += "\"bateria\":\"N/A\",";
  json += "\"ubicacion\":{";
  json += "\"lat\":" + String(latitud, 6) + ",";
  json += "\"lng\":" + String(longitud, 6) + ",";
  json += "\"alt\":" + String(altitud, 1) + ",";
  json += "\"sat\":" + String(satelites) + ",";
  json += "\"fix\":" + String(gpsValido ? "true" : "false");
  json += "},";
  json += "\"panico\":" + String(estadoPanico ? "true" : "false");
  json += "}";

  txBLE->setValue(json.c_str());
  txBLE->notify();

  Serial.print("[TX BLE -> App] ");
  Serial.println(json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("        PATHSENSE - SISTEMA INICIALIZADO          ");
  Serial.println("==================================================");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);

  generarNomenclatura();

  // Inicializar GPS en GPIO 18/17
  SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // Configuración BLE
  BLEDevice::init(bleDeviceName.c_str());
  BLEDevice::setMTU(512);

  servidorBLE = BLEDevice::createServer();
  servidorBLE->setCallbacks(new ServidorCallbacks());

  BLEService *servicio = servidorBLE->createService(SERVICE_UUID);

  txBLE = servicio->createCharacteristic(
            TX_UUID,
            BLECharacteristic::PROPERTY_READ   |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE
          );
  
  BLE2902 *pDescriptor = new BLE2902();
  pDescriptor->setNotifications(true);
  txBLE->addDescriptor(pDescriptor);

  rxBLE = servicio->createCharacteristic(
            RX_UUID,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
          );

  servicio->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("--------------------------------------------------");
  Serial.print("  [BLE] Anunciándose por Bluetooth como: ");
  Serial.println(bleDeviceName);
  Serial.print("  [BLE] Identificador Único del Bastón : ");
  Serial.println(deviceID);
  Serial.println("  [BLE] Estado: ESPERANDO VINCULACIÓN CON LA APP...");
  Serial.println("==================================================\n");
}

void loop() {
  // 1. Decodificación de datos GPS
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  unsigned long ahora = millis();

  // Diagnóstico serie si el GPS no envía datos
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    static unsigned long ultimoAviso = 0;
    if (ahora - ultimoAviso >= 4000) {
      Serial.println("[Aviso GPS] No entran caracteres. Confirma: GPS TXD -> ESP32 GPIO 18 y GPS RXD -> ESP32 GPIO 17.");
      ultimoAviso = ahora;
    }
  }

  // 2. Control de tiempo de espera inicial de 2 segundos tras conectar BLE
  if (bluetoothConectado && !listoParaTransmitir) {
    if (ahora - tiempoConexionBLE >= 2000) { // Pasar 2 segundos exactos
      listoParaTransmitir = true;
      Serial.println("\n>>> [BLE] Tiempo de estabilización (2s) cumplido. INICIANDO TRANSMISIÓN JSON CONTINUA. <<<\n");
    }
  }

  // 3. Medir ultrasonido
  if (ahora - ultimaMedicionUltrasonico >= INTERVALO_ULTRASONICO) {
    distanciaActual = medirDistancia();
    ultimaMedicionUltrasonico = ahora;
  }

  // 4. Control del zumbador progresivo
  actualizarBuzzerProgresivo();

  // 5. Transmitir JSON a la App
  if (ahora - ultimoEnvioJSON >= INTERVALO_ENVIO_JSON) {
    enviarTramaJSON();
    ultimoEnvioJSON = ahora;
  }
}
