# ESP32‑C6 + MQTT + Node‑RED Servo Control

## Overview

This guide documents a complete IoT workflow using:

* ESP32‑C6 (DFRobot FireBeetle 2)
* Mosquitto (MQTT broker)
* Node‑RED (dashboard + flows)
* Servo motor (actuator)

The system enables real‑time servo control from a web dashboard via MQTT, with optional feedback.

---

# 1.  System Architecture

```
Node‑RED (Dashboard/UI)
        │  publish: servo/control
        ▼
Mosquitto Broker (PC)
        │  Wi‑Fi (TCP 1883)
        ▼
ESP32‑C6
        │  PWM (GPIO)
        ▼
Servo Motor
```

---

# 2. Prerequisites

* Ubuntu (or similar Linux)
* Arduino IDE (AppImage recommended)
* ESP32 board package installed
* Libraries:

  * PubSubClient (Nick O’Leary)
  * ESP32Servo

---

# 3.  Software Setup

### 1) Install Node‑RED

```bash
sudo apt update
sudo apt install nodejs npm -y
sudo npm install -g --unsafe-perm node-red
node-red
```

Open: [http://localhost:1880](http://localhost:1880)

### 2) Install Mosquitto (MQTT)

```bash
sudo apt install mosquitto mosquitto-clients -y
sudo systemctl enable --now mosquitto
```

Verify locally:

```bash
mosquitto_sub -h localhost -t test
# in another terminal
mosquitto_pub -h localhost -t test -m "hello"
```

### 3) Install Node‑RED Dashboard

* Menu → Manage palette → Install `node-red-dashboard`

---

# 4. Network Setup

### Problem Encountered

* `rc = -2` when ESP32 connects to MQTT

### Cause

* Wrong broker IP and/or network isolation on Wi‑Fi

### Solution

* Use a phone hotspot (both PC and ESP32 on same network)
* Find PC IP:

```bash
hostname -I
```

* Use that IP in ESP32 code (NOT the ESP32 IP)

---

## Hardware Setup

### Servo Wiring

* Red → +5V (external supply or VBUS)
* Brown/Black → GND
* Orange/Yellow → GPIO (use GPIO 6)

### Critical Rule

```
All grounds must be shared (ESP32 GND ↔ supply GND)
```

---

## ESP32 Firmware (Working)

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

const char* ssid = "Reece's";
const char* password = "letmeinn0w";
const char* mqtt_server = "10.187.4.131"; // PC hotspot IP

WiFiClient espClient;
PubSubClient client(espClient);

Servo myServo;
const int servoPin = 6; // stable pin for PWM

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  int angle = constrain(msg.toInt(), 0, 180);
  myServo.write(angle);

  // optional feedback
  client.publish("servo/state", String(angle).c_str());
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32C6Client")) {
      client.subscribe("servo/control");
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  myServo.attach(servoPin, 500, 2400);
  myServo.write(90);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}
```

---

# 5. Node‑RED Flow
### Recommended (no feedback loop)

```
[slider] ──▶ [mqtt out  topic=servo/control]

[mqtt in  topic=servo/state] ──▶ [gauge]
```

### Temporary (simple test)

```
[slider] ──▶ [mqtt out]
   └──────▶ [gauge]
```

---

## Node‑RED Configuration

### Slider

* Min: 0
* Max: 180
* Topic: `servo/control`

### MQTT Out

* Server: `localhost`
* Port: `1883`
* Topic: (blank or `servo/control`)

### MQTT In

* Topic: `servo/state`

### Gauge

* Min: 0
* Max: 180
* Label: `Servo Position`
* Units: `°`
* Value: `{{value}}`

---

## Issues and Fixes

### Arduino (Snap) / GLIBC Error

* Cause: Snap build incompatibility
* Fix: Use AppImage with `--no-sandbox`

### AppImage FUSE Error

* Fix: `sudo apt install libfuse2`

### Port Busy (`/dev/ttyACM0`)

* Fix: Close Serial Monitor or

```bash
fuser -k /dev/ttyACM0
```

### MQTT `rc = -2`

* Cause: wrong IP / network isolation
* Fix: hotspot + correct PC IP

### Servo Not Moving

* Cause: unsuitable GPIO (e.g., GPIO9)
* Fix: use GPIO6

### Power Issues

* Cause: using 3.3V
* Fix: external 5V + shared ground

### Slider Jumping

* Cause: feedback loop on same topic
* Fix: separate topics (`servo/control` vs `servo/state`)

---

## Validation Checklist

* ESP32 connects to Wi‑Fi
* ESP32 connects to MQTT (no rc errors)
* `mosquitto_pub` moves servo
* Node‑RED slider publishes values
* Servo responds to slider
* Gauge displays position

---

## Extensions

* Add CAN bus (Arduino ↔ ESP32)
* Add Bluetooth input
* Publish sensor data (MQTT)
* Cloud broker integration
* Multi‑node architecture

---

## Learning Outcomes

* MQTT publish/subscribe model
* Embedded control (PWM, GPIO)
* Network debugging (IP, ports)
* Node‑RED dashboard design
* End‑to‑end IoT pipeline

---

**End of Document**
