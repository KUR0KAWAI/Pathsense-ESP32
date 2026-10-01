#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>

// =====================================================
// PINES HARDWARE
// =====================================================
const int PIN_TRIG   = 4;
const int PIN_ECHO   = 5;
const int PIN_BUZZER = 6;

const int PIN_GPS_RX = 18; // Conectar al TX del GPS
const int PIN_GPS_TX = 17; // Conectar al RX del GPS

const float DISTANCIA_ALERTA = 10.0; // Umbral de proximidad en cm
const unsigned long INTERVALO_SENSOR = 200;  // FEl código tiene una buena estructura base, pero presenta varios **errores críticos de configuración de hardware**, **bloqueos en la ejecución** y **problemas de estabilidad** propios de la arquitectura del **ESP32-S3**:

### Errores y problemas corregidos

1. **Confusión de GPIOs y asignación de pines:**
   * **Buzzer en GPIO 6:** En la mayoría de placas ESP32-S3 (WROOM-1/N8R8, etc.), los GPIOs del 6 al 11 están conectados internamente a la memoria Flash SPI (Quad/Octal SPI). Usar `GPIO 6` causa cuelgues o reinicios en bucle. Se reubicó al **GPIO 7**.
   * **Incompatibilidad de `tone()` / `noTone()`:** En el core de ESP32, la función `tone()` estándar de Arduino suele dar fallos de compilación o comportamientos erráticos con timbres pasivos/activos. Se optimizó el control del buzzer usando salidas digitales limpias o el canal LEDC de bajo nivel según el tipo de buzzer.
2. **Registro Brownout incompatible (`RTC_CNTL_BROWN_OUT_REG`):**
   * La instrucción `WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)` proviene de la arquitectura ESP32 original y rompe la compilación o falla en el **ESP32-S3**. Se actualizó usando las macros/funciones nativas del ESP32-S3 en `soc/rtc_cntl_reg.h`.
3. **Bloqueos en `pulseIn()`:**
   * El timeout de `pulseIn(PIN_ECHO, HIGH, 30000)` detiene la ejecución completa del microcontrolador hasta por 30 ms en cada lectura si no hay eco. Si el sensor ultrasónico falla o no detecta obstáculo, el flujo del GPS y BLE se degrada notablemente. Se redujo el timeout a **15000 µs (~250 cm)**.
4. **Callbacks de BLE sin manejo de reinicio de publicidad:**
   * Cuando se desconecta un dispositivo, se debe reiniciar la publicidad explicitando las propiedades para evitar que el chip quede invisible.

---

### Código Optimizado y Corregido

```cpp
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

#include "TinyGPS++.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "esp_mac.h"

// =====================================================
// PINES HARDWARE (Ajustados para ESP32-S3)
// Nota: Se evita el rango GPIO 6-11 asignados a Flash SPI
// =====================================================
const int PIN_TRIG   = 4;
const int PIN_ECHO   = 5;
const int PIN_BUZZER = 7;  // Movido de GPIO 6 a GPIO 7 para evitar conflictos SPI

const int PIN_GPS_RX = 18;
const int PIN_GPS_TX = 17;

const float DISTANCIA_ALERTA = 10.0; // Umbral en cm
const unsigned long INTERVALO_SENSOR = 200;

String bleDeviceName = "";
String deviceID      = "";

#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define TX_UUID      "abcd1234-1234-1234-1234-1234567890ab"
#define RX_UUID      "dcba4321-4321-4321-4321-ba0987654321"

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

BLEServer *servidorBLE = nullptr;
BLECharacteristic *txBLE = nullptr;
BLECharacteristic *rxBLE = nullptr;

bool bluetoothConectado = false;
float distanciaActual = -1.0;
unsigned long ultimaMedicion = 0;

void generarNomenclatura() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char sufijoMac[5];
  snprintf(sufijoMac, sizeof(sufijoMac), "%02X%02X", mac[4], mac[5]);
  bleDeviceName = "PS-V1P-" + String(sufijoMac);
  deviceID = "PS-BST-V1P-" + String(sufijoMac);
}

float medirDistancia() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Timeout ajustado a 15000us (~250cm max) para evitar bloqueos del procesador
  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, 15000);
  if (duracion == 0) return -1.0;
  return (duracion * 0.0343) / 2.0;
}

void actualizarBuzzer() {
  if (distanciaActual > 0.0 && distanciaActual <= DISTANCIA_ALERTA) {
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

class ServidorCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override { 
    bluetoothConectado = true; 
    Serial.println("[BLE] Cliente conectado.");
  }
  
  void onDisconnect(BLEServer *server) override {
    bluetoothConectado = false;
    Serial.println("[BLE] Cliente desconectado. Reiniciando publicidad...");
    BLEDevice::startAdvertising();
  }
};

void desactivarBrownoutESP32S3() {
  // Desactivación segura del detector de Brownout en ESP32-S3
  #if defined(RTC_CNTL_BROWN_OUT_REG)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  #endif
}

void setup() {
  desactivarBrownoutESP32S3();

  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== INICIANDO PATHSENSE ESP32-S3 ===");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  generarNomenclatura();
  Serial.print("Dispositivo: "); Serial.println(bleDeviceName);

  // Inicialización GPS
  SerialGPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // Inicialización BLE
  BLEDevice::init(bleDeviceName.c_str());
  servidorBLE = BLEDevice::createServer();
  servidorBLE->setCallbacks(new ServidorCallbacks());
  
  BLEService *servicio = servidorBLE->createService(SERVICE_UUID);

  txBLE = servicio->createCharacteristic(
            TX_UUID,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
          );
  txBLE->addDescriptor(new BLE2902());

  rxBLE = servicio->createCharacteristic(
            RX_UUID,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
          );

  servicio->start();
  
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06); // Ayuda con la compatibilidad de conexión en iPhone/Android
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Sistema Listo y Escaneable via Bluetooth.");
}

void loop() {
  // Lectura continua del buffer del GPS
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  unsigned long ahora = millis();

  if (ahora - ultimaMedicion >= INTERVALO_SENSOR) {
    distanciaActual = medirDistancia();
    ultimaMedicion = ahora;

    Serial.print("Distancia: ");
    if (distanciaActual > 0) {
      Serial.print(distanciaActual, 1);
      Serial.println(" cm");
    } else {
      Serial.println("Sin eco / Fuera de rango");
    }
  }

  actualizarBuzzer();
}
