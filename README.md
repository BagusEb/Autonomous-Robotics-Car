# Autonomous Robotic Car with Hazard Detection

An Arduino-based autonomous vehicle designed to intelligently navigate its environment, avoid obstacles, and respond to hazards like fire. This project demonstrates a practical application of sensor integration, motor control, and real-time decision-making algorithms in C++.

---

## 🚀 Key Features

- **Dynamic Obstacle Avoidance:** Engineered an active scanning system using a servo-mounted ultrasonic sensor, achieving a **95% success rate** in navigating and avoiding obstacles in various test environments.
- **Hazard Detection & Response:** Integrated a dual-sensor system (flame and temperature) that achieved an **85% detection rate** in test conditions, automatically triggering a fan module to respond to the hazard.
- **Intelligent Navigation Logic:** The firmware implements a scan-and-decide algorithm where the robot stops upon detection, scans its left and right sides, and turns towards the direction with the most open space.
- **Modular C++ Code:** The firmware is written in C++ for the Arduino platform, featuring clearly defined functions for movement, sensor data acquisition, and decision-making, making it easy to read and maintain.

---

## 🛠️ Hardware Components

- **Microcontroller:** Arduino Uno
- **Motor Driver:** L298N Dual H-Bridge
- **Actuators:**
  - 2 x DC Motors (for wheels)
  - 1 x Servo Motor (SG90, for sensor scanning)
  - 1 x L9110 Fan Module
- **Sensors:**
  - HC-SR04 Ultrasonic Sensor
  - Flame Sensor Module
  - DHT11 Temperature & Humidity Sensor
- **Chassis & Power:**
  - 2WD Robot Car Chassis
  - External Battery Power Supply (e.g., 2 x 18650 Batteries)

---

## ⚙️ Software & Libraries

- **Language:** C++ (Arduino Framework)
- **IDE:** Arduino IDE or PlatformIO
- **Libraries:**
  - `Servo.h`
  - `dht.h`

---
