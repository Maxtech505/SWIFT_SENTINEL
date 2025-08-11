/*
 * Project: Swift_Sentinel-01
 * Author: Maximo 
 * Date: 6/14/25
 * Description: GPS-enabled WiFi-controlled RC car with Adafruit IO integration
 * Based on GPS lesson L14_00_GPS
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "credentials.h"
#include "GPS_CNM.H"
#include "Wire.h"
#include "Adafruit_GPS.h"
#include "Adafruit_SSD1306/Adafruit_SSD1306.h" 
#include "Adafruit_SSD1306/Adafruit_GFX.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"

// MPU-9250 Register Definitions
#define MPU9250_ADDRESS     0x68
#define AK8963_ADDRESS      0x0C

// MPU-9250 Registers
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define SMPLRT_DIV          0x19
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define WHO_AM_I_MPU9250    0x75
#define USER_CTRL           0x6A
#define I2C_MST_CTRL        0x24

// Accelerometer data registers
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

// Temperature data registers
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

// Gyroscope data registers
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

// Magnetometer AK8963 registers
#define AK8963_WHO_AM_I     0x00
#define AK8963_ST1          0x02
#define AK8963_XOUT_L       0x03
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09
#define AK8963_CNTL         0x0A
#define AK8963_ASTC         0x0C
#define AK8963_ASAX         0x10
#define AK8963_ASAY         0x11
#define AK8963_ASAZ         0x12


// MQTT and GPS Libraries
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);

// GPS Hardware Setup
const int OLED_RESET = -1;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_GPS GPS(&Wire); // Use Wire interface for I2C communication

// MQTT Setup for Adafruit IO
TCPClient tcpClient;
Adafruit_MQTT_SPARK mqtt(&tcpClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// MQTT Feeds
Adafruit_MQTT_Subscribe gpsTargetFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/gps-target");
Adafruit_MQTT_Subscribe carControlFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/car-control"); 
Adafruit_MQTT_Subscribe carModeFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/car-mode");
Adafruit_MQTT_Publish gpsDataFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gps-data");
Adafruit_MQTT_Publish statusFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/car-status");
Adafruit_MQTT_Publish accelerometerFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/accelerometer-data");
Adafruit_MQTT_Publish gyroscopeFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gyroscope-data");
Adafruit_MQTT_Publish magnetometerFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/magnetometer-data");

// GPS Configuration
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000; // GPS update interval from your code
float gpsLat, gpsLon, gpsAlt;
int gpsSat;
bool gpsInitialized = false;
unsigned int lastGPS = 0; // Timer for GPS updates

// MPU-9250 3-Axis Accelerometer Variables
bool accelerometerInitialized = false;

// 3-Axis Accelerometer Data Structure
struct AccelerometerData {
  int16_t accel_x, accel_y, accel_z;    // Accelerometer (raw)
  int16_t temperature;                 // Temperature (raw)
  bool valid;
};

// MPU-9250 3-Axis Gyroscope Variables
bool gyroscopeInitialized = false;

// 3-Axis Gyroscope Data Structure
struct GyroscopeData {
  int16_t gyro_x, gyro_y, gyro_z;      // Gyroscope (raw)
  bool valid;
};

// Magnetometer (3-Axis) Variables
bool magnetometerInitialized = false;
float magAdjustment[3] = {0, 0, 0};  // Magnetometer sensitivity adjustment values

// 3-Axis Magnetometer Data Structure
struct MagnetometerData {
  int16_t mag_x, mag_y, mag_z;         // Magnetometer (raw)
  float heading;                       // Calculated heading in degrees
  bool valid;
};

AccelerometerData accelerometer = {0, 0, 0, 0, false};
GyroscopeData gyroscope = {0, 0, 0, false};
MagnetometerData magnetometer = {0, 0, 0, 0.0, false};
unsigned long lastAccelerometerRead = 0;
unsigned long lastGyroscopeRead = 0;
unsigned long lastMagnetometerRead = 0;
#define ACCELEROMETER_UPDATE_INTERVAL 50  // Read accelerometer every 50ms (20Hz)
#define GYROSCOPE_UPDATE_INTERVAL 100     // Read gyroscope every 100ms (10Hz)
#define MAGNETOMETER_UPDATE_INTERVAL 200  // Read magnetometer every 200ms (5Hz)

// GPS and Navigation variables
struct GPSLocation {
  double latitude;
  double longitude;
  bool valid;
};

GPSLocation currentLocation = {0.0, 0.0, false};
GPSLocation targetLocation = {0.0, 0.0, false};
bool navigationMode = false;  // True when navigating to coordinates

// Movement control variables
bool autoMode = true;  // Toggle between automatic and manual control
unsigned long lastMoveTime = 0;
int currentDirection = 0;  // 0=stop, 1=forward, 2=backward, 3=left, 4=right

// GPS and Navigation timing
unsigned long lastMQTTCheck = 0;
unsigned long lastMQTTPublish = 0;

// Timing constants
#define MQTT_PUBLISH_INTERVAL 10000  // Publish to MQTT every 10 seconds
#define MQTT_CHECK_INTERVAL 1000     // Check MQTT every 1 second
#define MQTT_KEEP_ALIVE 30           // MQTT keepalive in seconds

// Navigation constants
#define ARRIVAL_THRESHOLD 20.0       // Distance threshold for arrival (in meters)
#define BEARING_THRESHOLD 10.0       // Threshold for turning (in degrees)

// Function prototypes
int controlCar(String command);
int setMode(String mode);
int setTarget(String coordinates);
void moveForward();
void moveBackward();
void moveLeft();
void moveRight();
void stopCar();
void publishStatus();
void updateGPS();
void navigateToTarget();
double calculateDistance(GPSLocation from, GPSLocation to);
double calculateBearing(GPSLocation from, GPSLocation to);
void publishGPSData();
void checkAdafruitCommands();
void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
void handleAdafruitGPS(const char *event, const char *data);
void updateDisplay();
void MQTT_connect();
void MQTT_ping();
void setupWiFi();
void handleMQTTFeeds();
void scanI2C();

// MPU-9250 3-Axis Accelerometer Functions
bool initAccelerometer();
void readAccelerometerData();

// MPU-9250 3-Axis Gyroscope Functions
bool initGyroscope();
void readGyroscopeData();

// AK8963 Magnetometer Functions (3-Axis)
bool initMagnetometer();
void readMagnetometerData();

// Common I2C Functions
uint8_t readByte(uint8_t address, uint8_t subAddress);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
float calculateHeading(int16_t mx, int16_t my);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  
  // Print device information
  Serial.println("=== SWIFT SENTINEL RC CAR ===");
  Serial.printf("Firmware Version: %s\n", System.version().c_str());
  Serial.printf("Free Memory: %lu bytes\n", System.freeMemory());
  Serial.println("==============================");
  
  // Scan I2C devices before initialization
  Wire.begin(); // Initialize I2C before scanning
  scanI2C();
  
  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.printf("Swift Sentinel\nRC Car Starting...");
  display.display();
  
  // Initialize GPS module for I2C communication
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.sendCommand(PMTK_Q_RELEASE);
  // GPS.println(PMTK_Q_RELEASE);  // Removed: Adafruit_GPS has no 'println' method
  gpsInitialized = true;
  
  // Initialize MPU-9250 3-Axis Accelerometer
  if (initAccelerometer()) {
    Serial.println("MPU-9250 3-Axis Accelerometer initialized successfully!");
    accelerometerInitialized = true;
  } else {
    Serial.println("MPU-9250 3-Axis Accelerometer initialization failed!");
  }
  
  // Initialize MPU-9250 3-Axis Gyroscope separately
  if (initGyroscope()) {
    Serial.println("MPU-9250 3-Axis Gyroscope initialized successfully!");
    gyroscopeInitialized = true;
  } else {
    Serial.println("MPU-9250 3-Axis Gyroscope initialization failed!");
  }
  
  // Initialize AK8963 Magnetometer (3-Axis) separately
  if (initMagnetometer()) {
    Serial.println("AK8963 3-Axis Magnetometer initialized successfully!");
    magnetometerInitialized = true;
  } else {
    Serial.println("AK8963 3-Axis Magnetometer initialization failed!");
  }
  
  // Initialize motor control pins
  pinMode(D4, OUTPUT);  // Forward
  pinMode(D5, OUTPUT);  // Backward
  pinMode(D6, OUTPUT);  // Left
  pinMode(D7, OUTPUT);  // Right
  
  // Initialize all pins to LOW (stopped)
  stopCar();
  
  // Setup WiFi connection
  setupWiFi();
  
  // Subscribe to MQTT feeds
  mqtt.subscribe(&gpsTargetFeed);
  mqtt.subscribe(&carControlFeed);
  mqtt.subscribe(&carModeFeed);
  
  // Set timezone
  Time.zone(TIMEZONE);
  
  // Register cloud functions for remote control (backup to MQTT)
  Particle.function("control", controlCar);
  Particle.function("mode", setMode);
  Particle.function("setTarget", setTarget);
  
  Serial.println("Swift Sentinel RC Car with GPS and Separated 9-Axis Ready!");
  
  // Initial display update
  updateDisplay();
  
  // Publish startup message via MQTT
  statusFeed.publish("Swift Sentinel RC Car Online and Ready");
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // Get data from GPS unit (best if you do this continuously)
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
  
  // Read 3-Axis accelerometer data periodically
  if (accelerometerInitialized && (millis() - lastAccelerometerRead >= ACCELEROMETER_UPDATE_INTERVAL)) {
    readAccelerometerData();
    lastAccelerometerRead = millis();
  }
  
  // Read 3-Axis gyroscope data periodically
  if (gyroscopeInitialized && (millis() - lastGyroscopeRead >= GYROSCOPE_UPDATE_INTERVAL)) {
    readGyroscopeData();
    lastGyroscopeRead = millis();
  }
  
  // Read 3-Axis magnetometer data separately at different rate
  if (magnetometerInitialized && (millis() - lastMagnetometerRead >= MAGNETOMETER_UPDATE_INTERVAL)) {
    readMagnetometerData();
    lastMagnetometerRead = millis();
  }
  
  // Handle MQTT connection and communication
  MQTT_connect();
  MQTT_ping();
  
  // Check for incoming MQTT messages
  handleMQTTFeeds();
  
  // Update GPS location using your preferred timing
  if (millis() - lastGPS > UPDATE) {
    lastGPS = millis(); // reset the timer
    updateGPS();
    updateDisplay();
  }
  
  // Publish data to MQTT periodically
  if (millis() - lastMQTTPublish >= MQTT_PUBLISH_INTERVAL) {
    if (mqtt.connected()) {
      publishGPSData();
      publishStatus();
      
      // Publish accelerometer data if available (3-Axis)
      if (accelerometer.valid) {
        String accelString = String(accelerometer.accel_x) + "," + String(accelerometer.accel_y) + "," + String(accelerometer.accel_z) + "," +
                            String(accelerometer.temperature);
        accelerometerFeed.publish(accelString.c_str());
      }
      
      // Publish gyroscope data if available (3-Axis)
      if (gyroscope.valid) {
        String gyroString = String(gyroscope.gyro_x) + "," + String(gyroscope.gyro_y) + "," + String(gyroscope.gyro_z);
        gyroscopeFeed.publish(gyroString.c_str());
      }
      
      // Publish magnetometer data separately if available (3-Axis)
      if (magnetometer.valid) {
        String magString = String(magnetometer.mag_x) + "," + String(magnetometer.mag_y) + "," + String(magnetometer.mag_z) + "," +
                          String(magnetometer.heading, 1);
        magnetometerFeed.publish(magString.c_str());
      }
      
      lastMQTTPublish = millis();
    }
  }
  
  // Check if in automatic mode
  if (autoMode) {
    if (navigationMode && targetLocation.valid && currentLocation.valid) {
      // GPS Navigation mode - navigate to target coordinates
      navigateToTarget();
    } else {
      if (currentDirection != 0) {
        stopCar();
        currentDirection = 0;
      }
    }
  } else {
    // Manual mode - car controlled via WiFi commands
    delay(100);
  }
}

// Cloud function to control car movement
int controlCar(String command) {
  command.toLowerCase();
  
  if (command == "forward") {
    moveForward();
    currentDirection = 1;
    return 1;
  } else if (command == "backward") {
    moveBackward();
    currentDirection = 2;
    return 2;
  } else if (command == "left") {
    moveLeft();
    currentDirection = 3;
    return 3;
  } else if (command == "right") {
    moveRight();
    currentDirection = 4;
    return 4;
  } else if (command == "stop") {
    stopCar();
    currentDirection = 0;
    return 0;
  } else {
    return -1;
  }
}

// Cloud function to switch between auto and manual mode
int setMode(String mode) {
  mode.toLowerCase();
  
  if (mode == "auto") {
    autoMode = true;
    navigationMode = false;
    currentDirection = 0;
    lastMoveTime = millis();
    return 1;
  } else if (mode == "manual") {
    autoMode = false;
    navigationMode = false;
    stopCar();
    currentDirection = 0;
    return 0;
  } else if (mode == "navigation") {
    if (targetLocation.valid) {
      autoMode = true;
      navigationMode = true;
      Serial.println("Switched to Navigation Mode");
    } else {
      Serial.println("Cannot enter navigation mode: no valid target set.");
    }
    return 1;
  } else {
    return -1;
  }
}

void moveForward() {
  digitalWrite(D4, HIGH);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  Serial.println("Moving Forward");
}

void moveBackward() {
  digitalWrite(D4, LOW);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  Serial.println("Moving Backward");
}

void moveLeft() {
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
  digitalWrite(D7, LOW);
  Serial.println("Turning Left");
}

void moveRight() {
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, HIGH);
  Serial.println("Turning Right");
}

void stopCar() {
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  Serial.println("Car Stopped");
}

void publishStatus() {
  String status = "Mode: " + String(autoMode ? "Auto" : "Manual");
  if (navigationMode) {
    status += " (Navigation)";
  }
  status += ", GPS: " + String(currentLocation.valid ? "Valid" : "Invalid");
  
  if (!statusFeed.publish(status.c_str())) {
    Serial.println("Failed to publish status to MQTT");
  } else {
    Serial.println("Status published to MQTT: " + status);
  }
}

int setTarget(String coordinates) {
  int commaIndex = coordinates.indexOf(',');
  if (commaIndex == -1) {
    return -1;
  }
  
  double lat = coordinates.substring(0, commaIndex).toFloat();
  double lng = coordinates.substring(commaIndex + 1).toFloat();
  
  if (lat == 0.0 && lng == 0.0) {
    return -1;
  }
  
  targetLocation.latitude = lat;
  targetLocation.longitude = lng;
  targetLocation.valid = true;
  navigationMode = true;
  
  String msg = "Target set: " + String(lat, 6) + "," + String(lng, 6);
  Serial.println("Navigation target set: " + msg);
  
  return 1;
}

void updateGPS() {
  if (gpsInitialized) {
    getGPS(&gpsLat, &gpsLon, &gpsAlt, &gpsSat);
    
    if (GPS.fix && gpsLat != 0.0 && gpsLon != 0.0) {
      currentLocation.latitude = gpsLat;
      currentLocation.longitude = gpsLon;
      currentLocation.valid = true;
    } else {
      currentLocation.valid = false;
    }
  }
}

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites) {
  int theHour;
  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
  Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i\n",(int)GPS.fix,(int)GPS.fixquality);
  
  if (GPS.fix) {
    *latitude = GPS.latitudeDegrees;
    *longitude = GPS.longitudeDegrees; 
    *altitude = GPS.altitude;
    *satellites = (int)GPS.satellites;
    
    Serial.printf("GPS Data - Lat: %0.6f, Lon: %0.6f, Alt: %0.2f, Satellites: %i\n", 
                   *latitude, *longitude, *altitude, *satellites);
    Serial.printf("Speed (m/s): %0.2f\n",GPS.speed/1.944);
    Serial.printf("Angle: %0.2f\n",GPS.angle);
  } else {
    Serial.println("Waiting for GPS fix...");
    *satellites = 0;
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  
  if (currentLocation.valid) {
    display.printf("GPS: FIXED (%d sats)\n", gpsSat);
    display.printf("Lat: %0.6f\n", currentLocation.latitude);
    display.printf("Lon: %0.6f\n", currentLocation.longitude);
  } else {
    display.printf("GPS: NO FIX\n");
    display.printf("Searching...\n");
  }
  
  display.printf("Mode: %s\n", autoMode ? "AUTO" : "MANUAL");
  
  // Show accelerometer status (3-Axis)
  if (accelerometer.valid) {
    display.printf("Accel: ACTIVE\n");
  } else {
    display.printf("Accel: OFFLINE\n");
  }
  
  // Show gyroscope status (3-Axis)
  if (gyroscope.valid) {
    display.printf("Gyro: ACTIVE\n");
  } else {
    display.printf("Gyro: OFFLINE\n");
  }
  
  // Show magnetometer status and heading (3-Axis)
  if (magnetometer.valid) {
    display.printf("Mag: %0.1f deg\n", magnetometer.heading);
  } else {
    display.printf("Mag: OFFLINE\n");
  }
  
  if (navigationMode && targetLocation.valid) {
    double distance = calculateDistance(currentLocation, targetLocation);
    display.printf("Nav: %0.0fm\n", distance);
  } else {
    display.printf("Nav: OFF\n");
  }
  
  display.display();
}

void publishGPSData() {
  if (currentLocation.valid) {
    String gpsString = String(currentLocation.latitude, 6) + "," + 
                       String(currentLocation.longitude, 6) + "," +
                       String(GPS.speed * 1.852, 1);
    
    if (!gpsDataFeed.publish(gpsString.c_str())) {
      Serial.println("Failed to publish GPS data to MQTT");
    } else {
      Serial.println("GPS data published to MQTT: " + gpsString);
    }
  }
}

void navigateToTarget() {
  if (!currentLocation.valid || !targetLocation.valid) {
    stopCar();
    Serial.println("Navigation paused: Invalid GPS data.");
    return;
  }
  
  double distance = calculateDistance(currentLocation, targetLocation);
  
  if (distance <= ARRIVAL_THRESHOLD) {
    stopCar();
    navigationMode = false;
    targetLocation.valid = false;
    Serial.println("Target reached! Stopping navigation.");
    return;
  }
  
  double requiredBearing = calculateBearing(currentLocation, targetLocation);
  
  // Enhanced navigation with compass heading if available
  if (magnetometer.valid) {
    // Use magnetometer compass for precise navigation
    double headingDifference = requiredBearing - magnetometer.heading;
    
    // Normalize heading difference to -180 to +180 degrees
    if (headingDifference > 180) {
      headingDifference -= 360;
    }
    if (headingDifference < -180) {
      headingDifference += 360;
    }
    
    // Turn if heading difference is significant (>10 degrees)
    if (abs(headingDifference) > 10.0) {
      if (headingDifference > 0) {
        moveRight();
      } else {
        moveLeft();
      }
    } else {
      // Move forward when properly aligned
      moveForward();
    }
  } else {
    // Fallback: Simple forward movement without compass
    moveForward();
  }
  
  static unsigned long lastNavUpdate = 0;
  if (millis() - lastNavUpdate >= 5000) {
    if (magnetometer.valid) {
      Serial.printf("Navigating: %0.0fm, Required: %0.1f°, Current: %0.1f°\n", 
                     distance, requiredBearing, magnetometer.heading);
    } else {
      Serial.printf("Navigating: %0.0fm, Required Bearing: %0.1f degrees (no compass)\n", 
                     distance, requiredBearing);
    }
    lastNavUpdate = millis();
  }
}

// *** CORRECTED Haversine Formula ***
double calculateDistance(GPSLocation from, GPSLocation to) {
    const double R = 6371000.0; // Radius of Earth in meters
    double lat1Rad = from.latitude * M_PI / 180.0;
    double lon1Rad = from.longitude * M_PI / 180.0;
    double lat2Rad = to.latitude * M_PI / 180.0;
    double lon2Rad = to.longitude * M_PI / 180.0;
    
    double latDiff = lat2Rad - lat1Rad;
    double lonDiff = lon2Rad - lon1Rad;
    
    double a = sin(latDiff / 2.0) * sin(latDiff / 2.0) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(lonDiff / 2.0) * sin(lonDiff / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return R * c; // Distance in meters
}

double calculateBearing(GPSLocation from, GPSLocation to) {
  double lat1Rad = from.latitude * M_PI / 180.0;
  double lon1Rad = from.longitude * M_PI / 180.0;
  double lat2Rad = to.latitude * M_PI / 180.0;
  double lon2Rad = to.longitude * M_PI / 180.0;

  double y = sin(lon2Rad - lon1Rad) * cos(lat2Rad);
  double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(lon2Rad - lon1Rad);

  double bearing = atan2(y, x) * 180.0 / M_PI;
  if (bearing < 0) {
    bearing += 360.0;
  }

  return bearing;
}

void checkAdafruitCommands() { }
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      Serial.println("MQTT connection failed. Will retry next cycle.");
      return;
    }
  }
  Serial.println("MQTT Connected!");
}

void MQTT_ping() {
  static unsigned long lastPing = 0;
  if (millis() - lastPing > MQTT_KEEP_ALIVE * 1000) {
    if(mqtt.ping()) {
      Serial.println("MQTT Ping successful");
    } else {
      Serial.println("MQTT Ping failed");
    }
    lastPing = millis();
  }
}

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.on();
  WiFi.clearCredentials();
  WiFi.setCredentials(ssid, password);
  WiFi.connect();
  while(!WiFi.ready()) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void handleMQTTFeeds() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &gpsTargetFeed) {
      String gpsData = (char *)gpsTargetFeed.lastread;
      setTarget(gpsData);
    } else if (subscription == &carControlFeed) {
      String command = (char *)carControlFeed.lastread;
      controlCar(command);
    } else if (subscription == &carModeFeed) {
      String mode = (char *)carModeFeed.lastread;
      setMode(mode);
    }
  }
}

// I2C Scanner Function
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  Serial.println("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  
  for (int i = 0; i < 8; i++) {
    Serial.printf("%02x: ", i * 16);
    for (int j = 0; j < 16; j++) {
      int address = i * 16 + j;
      if (address < 8 || address > 0x77) {
        Serial.print("   ");
      } else {
        Wire.beginTransmission(address);
        int error = Wire.endTransmission();
        if (error == 0) {
          Serial.printf("%02x ", address);
        } else {
          Serial.print("-- ");
        }
      }
    }
    Serial.println();
  }
  
  Serial.println("I2C scan complete.");
  Serial.println("Expected devices:");
  Serial.println("  0x10 - GPS Module");
  Serial.println("  0x3D - OLED Display");
  Serial.println("  0x68 - MPU-9250 9-Axis Sensor");
  Serial.println("  0x0C - AK8963 Magnetometer (MPU-9250 internal)");
  Serial.println("==============================");
}

// MPU-9250 3-Axis Accelerometer Functions

bool initAccelerometer() {
  // Check WHO_AM_I register
  uint8_t whoami = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (whoami != 0x71) {
    Serial.printf("MPU-9250 WHO_AM_I failed: 0x%02X (expected 0x71)\n", whoami);
    return false;
  }
  
  // Reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100);
  
  // Clear sleep mode and select best available clock source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  delay(100);
  
  // Enable accelerometer only (disable gyroscope and magnetometer for this init)
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x38); // Disable gyro X, Y, Z
  delay(100);
  
  // Configure accelerometer sample rate (1kHz / (1 + SMPLRT_DIV))
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); // 200Hz
  
  // Configure accelerometer low-pass filter
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03); // 41Hz bandwidth
  
  // Configure accelerometer (±16g)
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x18);
  
  // Configure accelerometer low-pass filter
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03); // 41Hz bandwidth
  
  Serial.println("MPU-9250 3-axis accelerometer configured successfully");
  return true;
}

bool initGyroscope() {
  // Enable gyroscope (re-enable after accelerometer init)
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00); // Enable all sensors
  delay(100);
  
  // Configure gyroscope (±2000 dps)
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x18);
  
  // Configure interrupt pin for data ready
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
  
  Serial.println("MPU-9250 3-axis gyroscope configured successfully");
  return true;
}

bool initMagnetometer() {
  // Enable I2C master mode for magnetometer access
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20);
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x0D); // 400kHz I2C
  delay(10);
  
  // Check AK8963 WHO_AM_I
  uint8_t whoami = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
  if (whoami != 0x48) {
    Serial.printf("AK8963 WHO_AM_I failed: 0x%02X (expected 0x48)\n", whoami);
    return false;
  }
  
  // Reset AK8963
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
  delay(10);
  
  // Enter fuse ROM access mode to read sensitivity adjustment values
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
  delay(10);
  
  // Read sensitivity adjustment values
  uint8_t rawData[3];
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, rawData);
  magAdjustment[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;
  magAdjustment[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
  magAdjustment[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
  
  Serial.printf("Magnetometer sensitivity adjustments: X=%.3f, Y=%.3f, Z=%.3f\n",
                 magAdjustment[0], magAdjustment[1], magAdjustment[2]);
  
  // Set magnetometer to power down mode
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
  delay(10);
  
  // Set magnetometer to continuous measurement mode (16-bit, 100Hz)
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x16);
  delay(10);
  
  Serial.println("AK8963 3-axis magnetometer configured successfully");
  return true;
}

void readAccelerometerData() {
  uint8_t rawData[8];
  
  // Read accelerometer and temperature data (8 bytes)
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 8, rawData);
  
  // Combine high and low bytes
  accelerometer.accel_x = ((int16_t)rawData[0] << 8) | rawData[1];
  accelerometer.accel_y = ((int16_t)rawData[2] << 8) | rawData[3];
  accelerometer.accel_z = ((int16_t)rawData[4] << 8) | rawData[5];
  
  accelerometer.temperature = ((int16_t)rawData[6] << 8) | rawData[7];
  
  accelerometer.valid = true;
}

void readGyroscopeData() {
  uint8_t rawData[6];
  
  // Read gyroscope data (6 bytes)
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);
  
  // Combine high and low bytes
  gyroscope.gyro_x = ((int16_t)rawData[0] << 8) | rawData[1];
  gyroscope.gyro_y = ((int16_t)rawData[2] << 8) | rawData[3];
  gyroscope.gyro_z = ((int16_t)rawData[4] << 8) | rawData[5];
  
  gyroscope.valid = true;
}

void readMagnetometerData() {
  // Read magnetometer data
  uint8_t st1 = readByte(AK8963_ADDRESS, AK8963_ST1);
  if (st1 & 0x01) { // Data ready
    uint8_t magData[7];
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, magData);
    
    uint8_t st2 = magData[6]; // ST2 register
    if (!(st2 & 0x08)) { // Check for magnetic sensor overflow
      magnetometer.mag_x = ((int16_t)magData[1] << 8) | magData[0];
      magnetometer.mag_y = ((int16_t)magData[3] << 8) | magData[2];
      magnetometer.mag_z = ((int16_t)magData[5] << 8) | magData[4];
      
      // Apply sensitivity adjustments
      magnetometer.mag_x = (int16_t)(magnetometer.mag_x * magAdjustment[0]);
      magnetometer.mag_y = (int16_t)(magnetometer.mag_y * magAdjustment[1]);
      magnetometer.mag_z = (int16_t)(magnetometer.mag_z * magAdjustment[2]);
      
      // Calculate heading from magnetometer
      magnetometer.heading = calculateHeading(magnetometer.mag_x, magnetometer.mag_y);
      
      magnetometer.valid = true;
    }
  }
}

float calculateHeading(int16_t mx, int16_t my) {
  float heading = atan2(my, mx) * 180.0f / M_PI;
  if (heading < 0) {
    heading += 360.0f;
  }
  return heading;
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  data = Wire.read();
  return data;
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(address, count);
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }
}