# DIY Ventilator - NodeMCU ESP8266

**Educational Prototype**

## Features
- NodeMCU ESP8266 microcontroller
- MG995 Servo-based AMBU bag ventilation
- 20x4 Parallel LCD (JHD204A)
- DHT11 temperature/humidity sensor
- MAX30100 pulse/SpO2 sensor
- Adjustable Age Group: Adult, Child, Infant
- Start/Stop ventilation via button

## Wiring
See `docs/wiring_diagram.txt`

## Libraries
- LiquidCrystal
- DHT sensor library
- Adafruit_MAX30100

## Usage
1. Connect hardware as per wiring diagram
2. Install libraries in Arduino IDE
3. Select NodeMCU board
4. Upload `src/main.ino`
5. Use buttons to select Age Group
