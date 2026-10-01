# PathSense - Firmware ESP32-S3 🧑‍🦯📡

Firmware desarrollado en **Arduino IDE** para el bastón inteligente **PathSense** sobre microcontrolador **ESP32-S3**.

## 📌 Funcionalidades
- Lectura de sensor ultrasónico HC-SR04 para detección de obstáculos cercanos.
- Activación de alerta auditiva mediante buzzer pasivo.
- Lectura de datos de ubicación mediante módulo GPS (TinyGPS++).
- Transmisión periódica de datos (coordenadas, velocidad, batería y distancia) vía **Bluetooth Low Energy (BLE)** en formato JSON hacia la app móvil.

## 🛠️ Tecnologías y Librerías
- **Placa:** ESP32-S3 Dev Module
- **Librerías requeridas:**
  - `TinyGPS++` (Mikal Hart)
  - `ESP32 BLE Arduino` (Incluida en el core de ESP32)

## 🔌 Pinout / Conexiones Hardware
| Componente | Pin ESP32-S3 | Pin del Módulo |
| :--- | :--- | :--- |
| **Ultrasonido TRIG** | GPIO 4 | TRIG |
| **Ultrasonido ECHO** | GPIO 5 | ECHO |
| **Buzzer** | GPIO 6 | VCC / SIG |
| **GPS TX** | GPIO 18 (RX1) | TX |
| **GPS RX** | GPIO 17 (TX1) | RX |
