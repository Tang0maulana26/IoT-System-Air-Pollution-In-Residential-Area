# IoT-Based Air Quality Monitoring System for CO₂ Detection

A real-time, IoT-based air quality monitoring system designed to measure Carbon Dioxide (CO₂) concentrations and other environmental parameters in a residential setting. This project was developed as part of an undergraduate research thesis to provide an accessible and cost-effective solution for community-level air quality monitoring.

## System Architecture

The system operates by collecting data from various sensors connected to an ESP32 microcontroller. The ESP32 then publishes this data to an MQTT broker, which in turn forwards it to the Thingspeak and Blynk cloud platforms. 

![desain sistem polusi udara_schem](https://github.com/user-attachments/assets/88550bb8-639c-493d-b3ba-b333ce62a19d)

## System Design

![desain sistem polusi udara_bb](https://github.com/user-attachments/assets/1068d23f-7e86-408a-b198-8ef03f68f346)

## Hardware & Software Stack
### Hardware
- **Microcontroller:** ESP32 DEVKIT V1
- **Gas Sensor:** MQ-135 (for CO₂ estimation)
- **Temp/Humidity Sensor:** DHT22
- **Wind Speed Sensor:** Anemometer

### Software & Platforms
- **IDE:** Arduino IDE with ESP32 Core
- **Communication Protocol:** MQTT
- **Cloud Platforms:**
    - **Thingspeak:** For data logging, analysis, and visualization.
    - **Blynk IoT:** For a user-friendly mobile/web dashboard.
- **Key Libraries:** `PubSubClient.h`, `Adafruit_Sensor.h`, `DHT.h`, `MQUnifiedsensor.h`

[![DOI](https://zenodo.org/badge/998599615.svg)](https://doi.org/10.5281/zenodo.15620849)

