//Include the library
#include <MQUnifiedsensor.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>             
#include <Adafruit_GFX.h>     
#include <Adafruit_SSD1306.h> 
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>
#include <EEPROM.h>

//Definitions
#define board "ESP32" // Changed board to ESP32 for clarity, adjust if needed
#define Voltage_Resolution 3.3 // ESP32 ADC voltage reference is typically 3.3V
#define pin 32 // Example ADC pin for ESP32  
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // ESP32 ADC resolution is typically 12 bits
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm

// Define EEPROM address to store the R0 value
#define EEPROM_R0_ADDRESS 0

// Flag to force calibration (set to true for first time use)
#define FORCE_CALIBRATION false

#define MQ135_DEFAULTPPM 426.03 //default ppm of CO2 for calibration
#define MQ135_DEFAULTRO 44532 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2

/// Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857

/// Parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018

//Dht sensor
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);

// WiFi credentials
char ssid[] = "xxxxxxxx"; // Your WiFi SSID
char pass[] = "xxxxxxxx"; // Your WiFi Password

//Blynk MQTT Credentials and Setting
#define BLYNK_AUTH_TOKEN "xxxxxxxxxxxxxxxxxxxxxxxx" // Your Blynk Auth Token
#define BLYNK_DEVICE_ID "xxxxxxxxxxxxxxx"   // Your Blynk Device ID  
#define BLYNK_MQTT_SERVER "blynk.cloud"
#define BLYNK_MQTT_PORT 1883

// ThingSpeak MQTT credentials
#define channelID 2923006
const char mqttUserName[] = "xxxxxxxxxxxxxxxxxxxxxx"; // Your ThingSpeak MQTT username
const char clientID[] = "xxxxxxxxxxxxxxxxxxxxxx"; // Your ThingSpeak MQTT client ID
const char mqttPass[] = "xxxxxxxxxxxxxxxxxxxxxx"; // Your ThingSpeak MQTT password

// MQTT server and connection parameters
const char* server = "mqtt3.thingspeak.com";
#define mqttPort 1883
WiFiClient thingSpeakWifiClient;
PubSubClient thingSpeakMqttClient(thingSpeakWifiClient);

// MQTT WiFiClient for Blynk
WiFiClient blynkWifiClient;
PubSubClient blynkMqttClient(blynkWifiClient);

// Helper macro for periodic actions
#define EVERY_N_MILLIS(interval, code) { \
  static unsigned long previousMillis = 0; \
  unsigned long currentMillis = millis(); \
  if (currentMillis - previousMillis >= interval) { \
    previousMillis = currentMillis; \
    code; \
  } \
}

// MQTT client for thingspeak (time publish)
static unsigned long lastThingSpeakPublishMillis = 0;
const unsigned long thingSpeakUpdateIntervalSeconds = 300UL; // Publish every 300 seconds (5 minutes)
const unsigned long thingSpeakUpdateIntervalMillis = thingSpeakUpdateIntervalSeconds * 1000UL; // count once for unsigned long

// MQTT client for blynk (time publish)
static unsigned long lastBlynkSensorPublishMillis = 0;
const unsigned long blynkSensorUpdateIntervalSeconds = 300UL; // Interval in second (5 menit)
const unsigned long blynkSensorUpdateIntervalMillis = blynkSensorUpdateIntervalSeconds * 1000UL; // Convert to milidetik

// Function to handle messages from MQTT subscription
void mqttSubscriptionCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Subscribe to ThingSpeak channel for updates
void mqttSubscribe(long subChannelID) {
  String myTopic = "channels/" + String(subChannelID) + "/subscribe";
  thingSpeakMqttClient.subscribe(myTopic.c_str());
}

// Publish messages to ThingSpeak channel
void mqttPublish(long pubChannelID, String message) {
  String topicString = "channels/" + String(pubChannelID) + "/publish";
  thingSpeakMqttClient.publish(topicString.c_str(), message.c_str());
}

// Connect to WiFi
void connectWifi() {
  Serial.println("Connecting to Wi-Fi...");
  
  // Start with WiFi disconnected
  WiFi.disconnect(true);
  delay(1000);
  
  // Set WiFi to station mode explicitly
  WiFi.mode(WIFI_STA);
  delay(1000);
  
  // Begin connection
  WiFi.begin(ssid, pass);
  
  // Wait for connection with timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi. Check your credentials.");
  }
}

// Connect to MQTT server
void thingspeakMqttConnect() {
  int attempts = 0;
  while (!thingSpeakMqttClient.connected() && attempts < 3) {
    Serial.print("Attempting MQTT connection...");
    // Connect to the MQTT broker
    if (thingSpeakMqttClient.connect(clientID, mqttUserName, mqttPass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(thingSpeakMqttClient.state());
      Serial.println(" trying again in 2 seconds");
      delay(2000);
      attempts++;
    }
  }
}

// This is called after Blynk MQTT connects successfully
void blynkConnected() {
  // Publish a connection message
  blynkMqttClient.publish("ds/terminal", "Weather Station connected\n");
  
  // Subscribe to command topics
  blynkMqttClient.subscribe("downlink/#");
  
  // Send device info
  char info[128];
  snprintf(info, sizeof(info),
           "{\"type\":\"WeatherStation\",\"ver\":\"1.0.0\",\"build\":\"" __DATE__ " " __TIME__ "\",\"tmpl\":\"%s\"}",
           BLYNK_DEVICE_ID);
  blynkMqttClient.publish("info/mcu", info);
  
  Serial.println("Connected to Blynk MQTT and subscribed to commands");
}

// Modified callback for Blynk MQTT messages - handles both commands and downlinks
void blynkMqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Blynk message received [");
  Serial.print(topic);
  Serial.print("]: ");
  
  // Convert payload to string for easier handling
  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
    Serial.print((char)payload[i]);
  }
  message[length] = '\0';
  Serial.println();
  
  String t = String(topic);
  String v = String(message);
  
  // Handle standard Blynk downlink messages
  if (t == "downlink/reboot") {
    Serial.println("Rebooting...");
    ESP.restart();
  } else if (t == "downlink/ping") {
    // MQTT client automatically sends QoS1 response
  } else if (t == "downlink/diag") {
    Serial.print("Server diagnostic message: ");
    Serial.println(v);
  } else if (t == "downlink/ds/terminal") {
    // Terminal commands from Blynk app
    String reply = String("Command received: ") + v;
    blynkMqttClient.publish("ds/terminal", reply.c_str());
    
    // Add custom command handling here
    if (v == "status") {
      char statusMsg[100];
      snprintf(statusMsg, sizeof(statusMsg), 
               "WiFi RSSI: %d dBm, Uptime: %lu s", 
               WiFi.RSSI(), millis()/1000);
      blynkMqttClient.publish("ds/terminal", statusMsg);
    }
  }
  
  // Original command topic from your implementation
  else if (t == String("device/") + BLYNK_AUTH_TOKEN + "/command") {
    Serial.println("Legacy command format detected");
    // You can add handling for older command format here if needed
  }
}

// Updated connect function using the reference code pattern
void blynkMqttConnect() {
  if (blynkMqttClient.connected()) {
    return;
  }

  int attempts = 0;
  while (!blynkMqttClient.connected() && attempts < 3) {
    Serial.print("Attempting Blynk MQTT connection to ");
    Serial.print(BLYNK_MQTT_SERVER);
    Serial.print(":");
    Serial.print(BLYNK_MQTT_PORT);
    Serial.print("... ");
    
    // Configure MQTT client
    blynkMqttClient.setServer(BLYNK_MQTT_SERVER, BLYNK_MQTT_PORT);
    blynkMqttClient.setCallback(blynkMqttCallback);
    blynkMqttClient.setBufferSize(1024);
    
    // Create a unique client ID
    String clientId = String("device_") + BLYNK_AUTH_TOKEN + "_" + String(millis());
    
    // Connect to Blynk Cloud - using the device/token connection pattern
    if (blynkMqttClient.connect(clientId.c_str(), "device", BLYNK_AUTH_TOKEN)) {
      Serial.println("connected");
      
      // Disable WiFi sleep for better responsiveness
      WiFi.setSleep(false);
      
      // Call our connected function for setup
      blynkConnected();
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.print(blynkMqttClient.state());
      Serial.println(" trying again in 2 seconds");
      delay(2000);
      attempts++;
    }
  }
}

// anemometer sensor parameters
#define GPIO_pulse 14              // ESP32 pin connected to anemometer pulse output
volatile byte rpmcount = 0;         // Counter for anemometer pulses
volatile unsigned long last_micros = 0; // Timestamp of the last pulse detection
unsigned long timeold = 0;          // Used for timing the measurement interval
float timemeasure = 10.00;           // Measurement interval in seconds
int countThing = 0;                 // Counter for periodic actions (e.g., sending data)
float rpm, rotasi_per_detik;        // Revolutions per minute/second
float kecepatan_kilometer_per_jam;  // Speed in km/h
float kecepatan_meter_per_detik;    // Speed in m/s
volatile boolean flag = false;      // Flag set by ISR

// ISR for anemometer pulse detection
void ICACHE_RAM_ATTR rpm_anemometer()
{
  // Basic debouncing: ignore pulses too close together
  if (long(micros() - last_micros) >= 5000) { // 5ms debounce time
      rpmcount++;
      last_micros = micros();
  }
}

// NTP dan Zona Waktu
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

// OLED Display settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // Default I2C address for SSD1306, check your module
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint32_t delayMS;

//Declare Sensor
MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

void setup() {
  Serial.begin(115200); //Init serial port
  while (!Serial); // Wait for serial port to connect (useful for boards with native USB)

  // Connect to Wi-Fi - do this before MQTT setup
  connectWifi();
  
  // Configure ThingSpeak MQTT client
  thingSpeakMqttClient.setServer(server, mqttPort);
  thingSpeakMqttClient.setCallback(mqttSubscriptionCallback);
  thingSpeakMqttClient.setBufferSize(2048);

  // Configure Blynk MQTT client
  blynkMqttClient.setServer(BLYNK_MQTT_SERVER, BLYNK_MQTT_PORT);
  blynkMqttClient.setCallback(blynkMqttCallback);
  blynkMqttClient.setBufferSize(1024); // Set buffer size for Blynk MQTT client
  

  // Configure NTP time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial.println("Setup complete, starting measurements...");

  // --- OLED Setup ---
  Wire.begin(); // Initialize I2C ( SDA, SCL pins for your ESP32 board )
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
  delay(1000); // Pause for initialization message

  // --- DHT Setup ---
  dht.begin();
  //Serial.println(F("DHTxx Unified Sensor Example"));
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  if (delayMS < 2000) { // Ensure at least 2 seconds delay for DHT22
      delayMS = 2000;
  }
  //Serial.print("Sensor polling delay: "); Serial.print(delayMS); Serial.println(" ms");

  // --- MQ135 Setup with proper calibration ---
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(110.47); MQ135.setB(-2.862); // CO2 values
  MQ135.init();
  MQ135.setRL(22); // default is 10kOhm

  // Check if need should load R0 from EEPROM or perform calibration
  float storedR0 = 0;
  boolean performCalibration = FORCE_CALIBRATION;
  
  if (!FORCE_CALIBRATION) {
    EEPROM.begin(512);
    EEPROM.get(EEPROM_R0_ADDRESS, storedR0);
    EEPROM.end();
    
    // If stored R0 is not valid, perform calibration
    if (isnan(storedR0) || storedR0 <= 0) {
      performCalibration = true;
    } else {
      MQ135.setR0(storedR0);
      Serial.print("Loaded R0 from EEPROM: ");
      Serial.println(storedR0);
    }
  }

  // Perform calibration if needed
  if (performCalibration) {
    Serial.println("Calibrating MQ135 in clean air. Please wait...");
    float calcR0 = 0;
    
    // Let the sensor warm up before calibration
    Serial.println("Warming up sensor for 60 seconds...");
    delay(60000); // 60 seconds warm-up time
    
    MQ135.update(); // Update data before calibration
    for(int i = 1; i <= 10; i++) {
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
      Serial.print(".");
      delay(1000); // Longer delay between calibration readings
    }
    calcR0 /= 10;
    MQ135.setR0(calcR0);
    
    // Store R0 in EEPROM
    EEPROM.begin(512);
    EEPROM.put(EEPROM_R0_ADDRESS, calcR0);
    EEPROM.commit();
    EEPROM.end();
    
    Serial.println("\nCalibration completed and saved to EEPROM!");
    Serial.print("MQ135 R0 calibrated value: ");
    Serial.println(calcR0);
  }

  // Check for sensor connection issues
  if(isinf(MQ135.getR0())) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected). Please check your wiring and supply");
    while(1); // Stop execution
  }
  if(MQ135.getR0() == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground). Please check your wiring and supply");
    while(1); // Stop execution
  }

  // --- Anemometer Setup ---
  Serial.println("Initializing Anemometer...");
  pinMode(GPIO_pulse, INPUT_PULLUP); // Set anemometer pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // Attach ISR to the pin
  timeold = millis(); // Initialize time measurement baseline
  Serial.println("Anemometer Initialized.");

}

void loop() {
  // Reconnect to WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    connectWifi();
  }
  
  // Connect if MQTT client is not connected
  if (!thingSpeakMqttClient.connected()) {
    thingspeakMqttConnect();
    if (thingSpeakMqttClient.connected()) {
      mqttSubscribe(channelID);
    }
  }
  // Maintain connection to the ThingSpeak MQTT server
  thingSpeakMqttClient.loop();

  // Connect if Blynk MQTT client is not connected
  if (!blynkMqttClient.connected()) {
    blynkMqttConnect();
  }
  // Maintain connection to the Blynk MQTT server
  blynkMqttClient.loop();

  // Add periodic status updates (every 30 seconds)
  EVERY_N_MILLIS(30000, {
    if (blynkMqttClient.connected()) {
      blynkMqttClient.publish("ds/rssi", String(WiFi.RSSI()).c_str());
      blynkMqttClient.publish("ds/uptime", String(millis() / 1000).c_str());
    }
  });

  // Delay between main sensor readings (DHT, MQ135). Anemometer uses interrupts.
  delay(delayMS);

  // --- Read DHT Sensors ---
  sensors_event_t temp_event;
  dht.temperature().getEvent(&temp_event);
  float temperature = NAN; // Use Not-a-Number as default/error value
  if (!isnan(temp_event.temperature)) {
    temperature = temp_event.temperature;
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.println(F("°C"));
  } else {
    Serial.println(F("Error reading temperature!"));
  }

  sensors_event_t humidity_event;
  dht.humidity().getEvent(&humidity_event);
  float humidity = NAN; // Use Not-a-Number as default/error value
  if (!isnan(humidity_event.relative_humidity)) {
    humidity = humidity_event.relative_humidity;
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.println(F("%"));
  } else {
    Serial.println(F("Error reading humidity!"));
  }

  // --- Read MQ135 Sensor ---
  float cFactor = 1.0; // Default to 1.0 (no correction) if readings are invalid
  if (!isnan(temperature) && !isnan(humidity)) {
      cFactor = getCorrectionFactor(temperature, humidity);
      Serial.print("MQ135 Correction Factor: "); Serial.println(cFactor);
  } else {
      Serial.println("Cannot calculate correction factor due to invalid Temp/Humidity reading.");
  }

  MQ135.update(); // Update data, the ESP32 will read the voltage from the analog pin
  float ppm = MQ135.readSensor(false, cFactor);
  Serial.print("MQ135 Corrected CO2 (approx): "); Serial.print(ppm + 400); Serial.println(" ppm");

  // --- Read, Calculate and Print Anemometer Speed ---
  if ((millis() - timeold) >= (unsigned long)(timemeasure * 1000))
  {
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse));

    rotasi_per_detik = (float)rpmcount / timemeasure;
    kecepatan_meter_per_detik = (-0.0181 * (rotasi_per_detik * rotasi_per_detik)) + (1.3859 * rotasi_per_detik) + 1.4055;

    if (kecepatan_meter_per_detik <= 1.5)
    {
      kecepatan_meter_per_detik = 0.0;
    }
    kecepatan_kilometer_per_jam = kecepatan_meter_per_detik * 3.6;

    Serial.print("Anemometer RPS: "); Serial.print(rotasi_per_detik);
    Serial.print(" | Wind Speed (m/s): "); Serial.print(kecepatan_meter_per_detik);
    Serial.print(" | Wind Speed (km/h): "); Serial.println(kecepatan_kilometer_per_jam);

    rpmcount = 0;
    timeold = millis();

    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING);

    countThing++;
    // if (countThing >= 5)
    // {
    //   //Serial.println("--> Time to send data (Example)");
    //   countThing = 0;
    // }
  }

  // Get current time
  struct tm timeinfo;
  char timeStr[16];
  //char dateStr[15];
  
  if(getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
    //strftime(dateStr, sizeof(dateStr), "%d-%m-%Y", &timeinfo);
  } else {
    strcpy(timeStr, "--:--/--/--");
    //strcpy(dateStr, "--/--/----");
    Serial.println("Failed to obtain time");
  }

  // --- Update OLED Display ---
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  //Display Time and Date
  display.print("Time :");
  display.println(timeStr);

  // Display Wind Speed in km/h
  display.print("\nWind : ");
  if (!isnan(kecepatan_meter_per_detik)) {
    display.print(kecepatan_meter_per_detik, 1);  
    display.println(" km/h\n");
  } else {
    display.println("ERR");
  }

  // Display Temperature
  display.print("Temps : ");
  if (!isnan(temperature)) {
    display.print(temperature, 1);  
    display.println(" C");
  } else {
    display.println("ERR");
  }

  // Display Humidity
  display.print("Humid : ");
  if (!isnan(humidity)) {
    display.print(humidity, 1); 
    display.println(" %\n");
  } else {
    display.println("ERR\n");
  }

  // Display CO2 PPM
  display.print("CO2 : ");
  if (!isnan(ppm)) {
      display.print(ppm + 400, 2);  
      display.println(" ppm");
  } else {
      display.println("ERR");  
  }
  display.display(); // Show buffer on OLED
  Serial.println("------------------------------------");

  //Update for the blynk Mqtt IoT
  if (millis() - lastBlynkSensorPublishMillis >= blynkSensorUpdateIntervalMillis) { // Strong unsigned long arithmetic
    if (blynkMqttClient.connected()) {
      Serial.println("Mengirim data sensor ke Blynk...");
      blynkMqttClient.publish("ds/Temperature", String(temperature, 1).c_str());
      blynkMqttClient.publish("ds/Humidity", String(humidity, 1).c_str());
      blynkMqttClient.publish("ds/CO2", String(ppm + 400, 2).c_str());
      blynkMqttClient.publish("ds/WindSpeed", String(kecepatan_meter_per_detik, 1).c_str());
      lastBlynkSensorPublishMillis = millis(); // Update last publish time
      Serial.println("Sensor data successfully sent to Blynk.");
    } else {
      Serial.println("Blynk MQTT not connecting. Skipping sending sensor data.");
    }
  }

  // Update ThingSpeak channel periodically
  if (millis() - lastThingSpeakPublishMillis >= thingSpeakUpdateIntervalMillis) {
      // Format data for ThingSpeak
      String data = "field1=" + String(temperature, 1) + 
                    "&field2=" + String(humidity, 1) + 
                    "&field3=" + String(ppm + 400, 2) + 
                    "&field4=" + String(kecepatan_meter_per_detik, 1);

      if (thingSpeakMqttClient.connected()){ // mqttClient is now thingSpeakMqttClient
        mqttPublish(channelID, data);
        Serial.println("Data sent to ThingSpeak: " + data);
      } else {
        Serial.println("ThingSpeak MQTT not connected. Skipping publish.");
      }
      
      // Update last publish time
      lastThingSpeakPublishMillis = millis();
    }  
}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity
@param[in] t  The ambient air temperature in Celsius
@param[in] h  The relative humidity in %
@return The calculated correction factor
*/
/**************************************************************************/
float getCorrectionFactor(float t, float h) {
  return CORA * t * t - CORB * t + CORC - (h - 33.) * CORD;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum
@param[in] resvalue The raw sensor resistance (Rs) in Ohms
@param[in] t  The ambient air temperature in Celsius
@param[in] h  The relative humidity in %
@return The corrected sensor resistance kOhm (Note: original function returned kOhm, ensure consistency)
*/
/**************************************************************************/
float getCorrectedResistance(long resvalue, float t, float h) {
  return (float)resvalue / getCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum
@param[in] resvalue The raw sensor resistance (Rs) in Ohms
@param[in] t  The ambient air temperature in Celsius
@param[in] h  The relative humidity in %
@param[in] ro The sensor's resistance in clean air (R0) in Ohms
@return The ppm of CO2 in the air
*/
/**************************************************************************/
float getCorrectedPPM(long resvalue, float t, float h, long ro) {
  float corrected_rs = getCorrectedResistance(resvalue, t, h);
  if (ro <= 0 || corrected_rs <= 0) return 0;
  float ratio = corrected_rs / (float)ro;
  return PARA * pow(ratio, -PARB);
}