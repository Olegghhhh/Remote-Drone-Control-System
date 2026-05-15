#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESP32Servo.h>

// ============================================================
// Logging macros (0=OFF, 1=ERROR, 2=INFO, 3=DEBUG)
// Set to 2 for production, 3 for development debugging
// ============================================================
#define LOG_LEVEL 2
#define LOG_ERROR(fmt, ...) do { if (LOG_LEVEL >= 1) Serial.printf("[ERR] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_INFO(fmt, ...)  do { if (LOG_LEVEL >= 2) Serial.printf("[INF] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOG_DEBUG(fmt, ...) do { if (LOG_LEVEL >= 3) Serial.printf("[DBG] " fmt "\n", ##__VA_ARGS__); } while(0)

// ============================================================
// WiFi & Network Configuration
// ============================================================
const char* ssid = "DroneController";
const char* password = "flyhigh123";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
const String tvStickMac = "00:e0:4d:02:2d:98";
const IPAddress tvStickFixedIP(192, 168, 4, 250);
String tvStickIP = "";

// ============================================================
// Server & Servo Instances
// ============================================================
WebServer server(80);
DNSServer dnsServer;
Servo servo1, servo2, servo3, servo4, servo5, servo6;

// ============================================================
// Pin Definitions (constants)
// ============================================================
const int motorPins[] = {13, 25};
const int tractionMotorEN = 12;
const int tractionMotorIN1 = 5;
const int tractionMotorIN2 = 19;
const int NAV_LIGHT_LEFT = 15;
const int NAV_LIGHT_RIGHT = 4;
const int NAV_LIGHT_REAR = 18;
const int BEACON_LIGHT_TOP = 23;
const int STROBE_LIGHT = 21;

// ============================================================
// Timing Constants
// ============================================================
const int interval = 50;
const unsigned long motorTimeout = 30000;
const unsigned long controlTimeout = 30000;
const unsigned long maxControlTime = 300000;
const int maxRequestsPerSecond = 50;

// ============================================================
// Grouped State: Servos
// ============================================================
struct ServoState {
  int pos1 = 45, pos2 = 45, pos3 = 45, pos4 = 45;
  int pos5 = 90, pos6 = 90;
  int targetPos1 = 45, targetPos2 = 45, targetPos3 = 45, targetPos4 = 45;
  int targetPos5 = 90, targetPos6 = 90;
  int baseTargetPos3 = 45, baseTargetPos4 = 45; // Pre-yaw base targets (fix 4.4)
  int step = 1;
  unsigned long prevMillis = 0;
  float yawValue = 0;
  bool flapsMode = false;
  int flapsValue = 45;
  bool cameraAutoMode = false;
  bool returningToNeutral = false;
} sv;
// Reference aliases for backward compatibility
int &pos1 = sv.pos1, &pos2 = sv.pos2, &pos3 = sv.pos3, &pos4 = sv.pos4;
int &pos5 = sv.pos5, &pos6 = sv.pos6;
int &targetPos1 = sv.targetPos1, &targetPos2 = sv.targetPos2;
int &targetPos3 = sv.targetPos3, &targetPos4 = sv.targetPos4;
int &targetPos5 = sv.targetPos5, &targetPos6 = sv.targetPos6;
int &baseTargetPos3 = sv.baseTargetPos3, &baseTargetPos4 = sv.baseTargetPos4;
int &step = sv.step;
unsigned long &prevMillis = sv.prevMillis;
float &yawValue = sv.yawValue;
bool &flapsMode = sv.flapsMode;
int &flapsValue = sv.flapsValue;
bool &cameraAutoMode = sv.cameraAutoMode;
bool &returningToNeutral = sv.returningToNeutral;

// ============================================================
// Grouped State: Motors
// ============================================================
struct MotorState {
  int tractionSpeed = 0;
  unsigned long wingStartTime = 0;
  unsigned long tractionStartTime = 0;
  bool wingRunning = false;
  bool tractionRunning = false;
} mt;
int &tractionSpeed = mt.tractionSpeed;
unsigned long &wingMotorsStartTime = mt.wingStartTime;
unsigned long &tractionMotorStartTime = mt.tractionStartTime;
bool &wingMotorsRunning = mt.wingRunning;
bool &tractionMotorRunning = mt.tractionRunning;

// ============================================================
// Grouped State: Lights
// ============================================================
struct LightState {
  unsigned long beaconPreviousMillis = 0;
  unsigned long strobePreviousMillis = 0;
  bool beaconState = false;
  bool strobeState = false;
  bool enabled = false;
} lt;
unsigned long &beaconPreviousMillis = lt.beaconPreviousMillis;
unsigned long &strobePreviousMillis = lt.strobePreviousMillis;
bool &beaconState = lt.beaconState;
bool &strobeState = lt.strobeState;
bool &lightsEnabled = lt.enabled;

// ============================================================
// Grouped State: Access Control
// ============================================================
struct AccessState {
  String activeUserIP = "";
  String authenticatedIP = "";
  unsigned long lastActivity = 0;
  unsigned long controlStartTime = 0;
  bool isAuthenticated = false;
} ac;
String &activeUserIP = ac.activeUserIP;
String &authenticatedIP = ac.authenticatedIP;
unsigned long &lastActivity = ac.lastActivity;
unsigned long &controlStartTime = ac.controlStartTime;
bool &isAuthenticated = ac.isAuthenticated;

// ============================================================
// Grouped State: Request Tracking
// ============================================================
struct RequestTracker {
  unsigned long count = 0;
  unsigned long lastSecond = 0;
  int perSecond = 0;
} rq;
unsigned long &requestCount = rq.count;
unsigned long &lastSecond = rq.lastSecond;
int &requestsPerSecond = rq.perSecond;

/**
 * @brief Initialize hardware and network settings
 */
void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  
  // Initialize servo motors with specified pins and PWM range
  if (!servo1.attach(33, 500, 2500)) Serial.println("Failed to initialize servo1 on pin 33");
  if (!servo2.attach(17, 500, 2500)) Serial.println("Failed to initialize servo2 on pin 17");
  if (!servo3.attach(32, 500, 2500)) Serial.println("Failed to initialize servo3 on pin 32");
  if (!servo4.attach(16, 500, 2500)) Serial.println("Failed to initialize servo4 on pin 16");
  if (!servo5.attach(27, 500, 2500)) Serial.println("Failed to initialize servo5 (camera pan) on pin 27");
  if (!servo6.attach(14, 500, 2500)) Serial.println("Failed to initialize servo6 (camera tilt) on pin 14");
  
  // Set initial camera servo positions
  servo5.write(90);
  servo6.write(90);

  // Log servo and motor pin information
  Serial.println("Servo pins: 33,17 (front ailerons), 32,16 (rear ailerons), 27,14 (camera) - PWM supported");
  Serial.println("Traction motor EN pin: 12 - PWM supported");

  // Initialize wing motor pins
  for (int pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // Ensure motors are off
  }

  // Initialize traction motor pins
  pinMode(tractionMotorEN, OUTPUT);
  pinMode(tractionMotorIN1, OUTPUT);
  pinMode(tractionMotorIN2, OUTPUT);
  setTractionMotor(0); // Set traction motor to off

  // Initialize navigation light pins
  pinMode(NAV_LIGHT_LEFT, OUTPUT);
  pinMode(NAV_LIGHT_RIGHT, OUTPUT);
  pinMode(NAV_LIGHT_REAR, OUTPUT);
  pinMode(BEACON_LIGHT_TOP, OUTPUT);
  pinMode(STROBE_LIGHT, OUTPUT);
  updateLights(); // Update light states

  // Configure and start WiFi access point
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(ssid, password);
  Serial.printf("Access point \"%s\" created with IP address %s\n", ssid, WiFi.softAPIP().toString().c_str());

  // Start DNS server for captive portal
  dnsServer.start(53, "*", apIP);

  // Set TV stick IP address
  tvStickIP = tvStickFixedIP.toString();
  LOG_INFO("Fixed IP for TV stick: %s", tvStickIP.c_str());

  // Set up HTTP server routes
  setupServer();
  
  // Test servo motors by setting initial positions
  Serial.println("Testing servos...");
  servo1.write(45);
  servo2.write(45);
  servo3.write(45);
  servo4.write(45);
  servo5.write(90);
  servo6.write(90);
  delay(1000); // Wait for servos to reach positions
  LOG_INFO("Ailerons set to 45 deg, camera set to 90 deg");
  
  // Log system readiness
  LOG_INFO("System ready, IP: %s", WiFi.softAPIP().toString().c_str());
}

/**
 * @brief Control traction motor speed and direction
 * @param speed Motor speed (-55 to 55)
 */
void setTractionMotor(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -55, 55);
  tractionSpeed = abs(speed);
  
  // Map speed to PWM value (200-255, or 0 for off)
  int pwmValue = (tractionSpeed == 0) ? 0 : map(tractionSpeed, 1, 55, 200, 255);
  pwmValue = (pwmValue >= 1 && pwmValue < 200) ? 200 : pwmValue;
  
  // Set motor direction and state
  if (speed > 0) {
    digitalWrite(tractionMotorIN1, HIGH);
    digitalWrite(tractionMotorIN2, LOW);
    tractionMotorRunning = true;
    tractionMotorStartTime = millis();
  } else if (speed < 0) {
    digitalWrite(tractionMotorIN1, LOW);
    digitalWrite(tractionMotorIN2, HIGH);
    tractionMotorRunning = true;
    tractionMotorStartTime = millis();
  } else {
    digitalWrite(tractionMotorIN1, LOW);
    digitalWrite(tractionMotorIN2, LOW);
    tractionMotorRunning = false;
  }
  // Apply PWM to motor
  analogWrite(tractionMotorEN, pwmValue);
  LOG_DEBUG("Traction motor: speed=%d, PWM=%d", speed, pwmValue);
}

/**
 * @brief Update navigation and beacon lights based on state
 */
void updateLights() {
  // Turn off all lights if disabled or no user is active
  if (!lightsEnabled || activeUserIP == "") {
    digitalWrite(NAV_LIGHT_LEFT, LOW);
    digitalWrite(NAV_LIGHT_RIGHT, LOW);
    digitalWrite(NAV_LIGHT_REAR, LOW);
    digitalWrite(BEACON_LIGHT_TOP, LOW);
    digitalWrite(STROBE_LIGHT, LOW);
    return;
  }

  // Turn on steady navigation lights
  digitalWrite(NAV_LIGHT_LEFT, HIGH);
  digitalWrite(NAV_LIGHT_RIGHT, HIGH);
  digitalWrite(NAV_LIGHT_REAR, HIGH);

  // Update beacon light (toggles every 1 second)
  unsigned long currentMillis = millis();
  if (currentMillis - beaconPreviousMillis >= 1000) {
    beaconPreviousMillis = currentMillis;
    beaconState = ! beaconState;
    digitalWrite(BEACON_LIGHT_TOP, beaconState ? HIGH : LOW);
  }

  // Update strobe light (double flash every 2 seconds)
  static int strobePhase = 0;
  if (currentMillis - strobePreviousMillis >= (strobePhase < 2 ? 100 : 1800)) {
    strobePreviousMillis = currentMillis;
    if (strobePhase < 2) {
      strobeState = !strobeState;
      digitalWrite(STROBE_LIGHT, strobeState ? HIGH : LOW);
      strobePhase++;
    } else {
      strobePhase = 0;
    }
  }
}

/**
 * @brief Enable or disable navigation lights
 * @param enable True to enable lights, false to disable
 */
void toggleLights(bool enable) {
  lightsEnabled = enable;
  updateLights();
  LOG_INFO("Navigation lights: %s", enable ? "enabled" : "disabled");
}

/**
 * @brief Check if request limit is exceeded
 * @return True if within limit, false otherwise
 */
bool checkRequestLimit() {
  // Track requests per second
  unsigned long currentTime = millis();
  if (currentTime / 1000 != lastSecond) {
    LOG_DEBUG("Request rate: %d/s", requestsPerSecond);
    requestsPerSecond = 0;
    lastSecond = currentTime / 1000;
  }

  // Check if request limit is exceeded
  if (requestsPerSecond >= maxRequestsPerSecond) {
    LOG_ERROR("Request limit exceeded: %d", requestsPerSecond);
    server.send(429, "text/plain", "Too Many Requests");
    return false;
  }

  // Increment request counters
  requestsPerSecond++;
  requestCount++;
  LOG_DEBUG("Request #%lu, %d/s", requestCount, requestsPerSecond);
  return true;
}

/**
 * @brief Configure HTTP server routes and handlers
 */
void setupServer() {
  // Handle 404 and captive portal logic
  server.onNotFound([]() {
    if (!checkRequestLimit()) return;
    String clientIP = server.client().remoteIP().toString();
    // Ignore favicon and static resource requests
    if (server.uri() == "/favicon.ico" || server.uri().startsWith("/static/")) {
      server.send(404, "text/plain", "");
      return;
    }
    // Serve main page if authenticated, else redirect to login
    if (isAuthenticated && clientIP == authenticatedIP) {
      handleRoot();
    } else if (server.uri() != "/login" && server.uri() != "/start") {
      server.send(200, "text/html", "<script>window.location.href='http://192.168.4.1/login';</script>");
      LOG_DEBUG("Redirecting to /login for client %s", clientIP.c_str());
    } else {
      handleRoot();
    }
  });

  // Serve login page
  server.on("/login", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    handleLogin();
  });

  // Handle start button for authentication
  server.on("/start", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    String clientIP = server.client().remoteIP().toString();
    isAuthenticated = true;
    authenticatedIP = clientIP;
    server.send(200, "text/html", "<script>window.location.href='http://192.168.4.1/';</script>");
    LOG_INFO("User %s clicked Start", clientIP.c_str());
  });

  // Serve main control page
  server.on("/", []() {
    if (!checkRequestLimit()) return;
    handleRoot();
  });

  // Handle control acquisition
  server.on("/get_control", []() {
    if (!checkRequestLimit()) return;
    handleGetControl();
  });

  // Handle control release
  server.on("/release_control", []() {
    if (!checkRequestLimit()) return;
    handleReleaseControl();
  });

  // Reset system to neutral state
  server.on("/reset", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    resetSystem();
    server.send(200, "application/json", "{\"status\":\"success\"}");
    LOG_INFO("System reset, total: %lu", requestCount);
  });

  // Start wing motors
  server.on("/start_motors", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    for (int pin : motorPins) digitalWrite(pin, HIGH);
    wingMotorsRunning = true;
    wingMotorsStartTime = millis();
    server.send(200, "application/json", "{\"status\":\"success\"}");
    LOG_INFO("Wing motors started, total: %lu", requestCount);
  });

  // Stop wing motors
  server.on("/stop_motors", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    for (int pin : motorPins) digitalWrite(pin, LOW);
    wingMotorsRunning = false;
    server.send(200, "application/json", "{\"status\":\"success\"}");
    LOG_INFO("Wing motors stopped, total: %lu", requestCount);
  });

  // Control traction motor speed
  server.on("/traction_motor", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    if (server.hasArg("speed")) {
      int speed = server.arg("speed").toInt();
      if (speed < -55 || speed > 55) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid speed\"}");
        LOG_ERROR("Invalid traction speed: %d", speed);
        return;
      }
      setTractionMotor(speed);
      server.send(200, "application/json", "{\"status\":\"success\"}");
      LOG_DEBUG("Traction speed=%d, total: %lu", speed, requestCount);
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Speed parameter missing\"}");
      LOG_ERROR("Speed parameter missing");
    }
  });

  // Handle joystick input for aileron control
  server.on("/joystick", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    
    if (server.hasArg("x") && server.hasArg("y")) {
      float x = server.arg("x").toFloat();
      float y = server.arg("y").toFloat();
      if (abs(x) > 1.0 || abs(y) > 1.0) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid x or y values\"}");
        LOG_ERROR("Invalid joystick: x=%.2f, y=%.2f", x, y);
        return;
      }
      
      // Calculate aileron positions based on joystick input
      if (abs(x) > 0.1 || abs(y) > 0.1) {
        if (!flapsMode) {
          int frontLeft = 45 - (x * 45);
          int frontRight = 45 + (x * 45);
          targetPos1 = constrain(frontLeft, 0, 90);
          targetPos2 = constrain(90 - frontRight, 0, 90);
        }
        
        int vertical = 45 + (y * 45);
        int rearLeft = vertical - (x * 45);
        int rearRight = vertical + (x * 45);
        
        // Store base targets before yaw (fix 4.4)
        baseTargetPos3 = constrain(90 - rearLeft, 0, 90);
        baseTargetPos4 = constrain(rearRight, 0, 90);
      } else {
        if (!flapsMode) {
          targetPos1 = 45;
          targetPos2 = 45;
        }
        // Store neutral base targets (fix 4.4)
        baseTargetPos3 = 45;
        baseTargetPos4 = 45;
      }
      // Apply yaw on top of base targets (fix 4.4)
      targetPos3 = constrain(baseTargetPos3 - (int)(yawValue * 45), 0, 90);
      targetPos4 = constrain(baseTargetPos4 - (int)(yawValue * 45), 0, 90);
      
      LOG_DEBUG("Joystick: x=%.2f y=%.2f yaw=%.2f tgt=%d,%d,%d,%d", x, y, yawValue, targetPos1, targetPos2, targetPos3, targetPos4);
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing x or y parameters\"}");
      LOG_ERROR("Missing x or y parameters");
    }
  });

  // Handle yaw control input
  server.on("/yaw_control", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    if (server.hasArg("value")) {
      float value = server.arg("value").toFloat();
      if (abs(value) > 1.0) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid yaw value\"}");
        LOG_ERROR("Invalid yaw value: %.2f", value);
        return;
      }
      
      yawValue = value;
      
      // Apply yaw to stored base targets (fix 4.4 — non-cumulative)
      targetPos3 = constrain(baseTargetPos3 - (int)(yawValue * 45), 0, 90);
      targetPos4 = constrain(baseTargetPos4 - (int)(yawValue * 45), 0, 90);
      
      LOG_DEBUG("Yaw: val=%.2f tgt3=%d tgt4=%d", yawValue, targetPos3, targetPos4);
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing value parameter\"}");
      LOG_ERROR("Missing yaw value parameter");
    }
  });

  // Handle flaps control input
  server.on("/flaps_control", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    if (server.hasArg("value")) {
      int value = server.arg("value").toInt();
      if (value < 0 || value > 90) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid flaps value\"}");
        LOG_ERROR("Invalid flaps value: %d", value);
        return;
      }
      
      flapsMode = true;
      flapsValue = value;
      targetPos1 = constrain(value, 0, 90);
      targetPos2 = constrain(90 - value, 0, 90);
      
      LOG_DEBUG("Flaps: val=%d tgt1=%d tgt2=%d", flapsValue, targetPos1, targetPos2);
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing value parameter\"}");
      LOG_ERROR("Missing flaps value parameter");
    }
  });

  // Reset flaps to neutral
  server.on("/reset_flaps", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    flapsMode = false;
    targetPos1 = 45;
    targetPos2 = 45;
    
    LOG_DEBUG("Flaps reset, tgt1=%d tgt2=%d", targetPos1, targetPos2);
    server.send(200, "application/json", "{\"status\":\"success\"}");
  });

  // Handle camera control input
  server.on("/camera_control", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    
    if (server.hasArg("x") && server.hasArg("y")) {
      int pan = server.arg("x").toInt();
      int tilt = server.arg("y").toInt();
      if (pan < 0 || pan > 180 || tilt < 0 || tilt > 180) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid pan or tilt values\"}");
        LOG_ERROR("Invalid camera: pan=%d, tilt=%d", pan, tilt);
        return;
      }
      
      targetPos5 = constrain(pan, 0, 180);
      targetPos6 = constrain(tilt, 0, 180);
      
      LOG_DEBUG("Camera: pan=%d tilt=%d", pan, tilt);
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing pan or tilt parameters\"}");
      LOG_ERROR("Missing camera pan/tilt parameters");
    }
  });

  // Turn on navigation lights
  server.on("/lights_on", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    toggleLights(true);
    server.send(200, "application/json", "{\"status\":\"success\"}");
    LOG_INFO("Lights on, total: %lu", requestCount);
  });

  // Turn off navigation lights
  server.on("/lights_off", HTTP_GET, []() {
    if (!checkRequestLimit()) return;
    if (!checkAccess()) return;
    toggleLights(false);
    server.send(200, "application/json", "{\"status\":\"success\"}");
    LOG_INFO("Lights off, total: %lu", requestCount);
  });

  // Start the HTTP server
  server.begin();
  LOG_INFO("HTTP server started");
}

/**
 * @brief Main loop to handle network requests and system updates
 */
void loop() {
  // Process DNS and HTTP requests
  dnsServer.processNextRequest();
  server.handleClient();
  
  unsigned long currentMillis = millis();

  // Check wing motor timeout
  if (wingMotorsRunning && currentMillis - wingMotorsStartTime >= motorTimeout) {
    for (int pin : motorPins) digitalWrite(pin, LOW);
    wingMotorsRunning = false;
    LOG_INFO("Wing motors auto-stopped after %lu ms", motorTimeout);
  }

  // Check traction motor timeout
  if (tractionMotorRunning && currentMillis - tractionMotorStartTime >= motorTimeout) {
    setTractionMotor(0);
    tractionMotorRunning = false;
    LOG_INFO("Traction motor auto-stopped after %lu ms", motorTimeout);
  }

  // Check for control inactivity timeout
  if (activeUserIP != "" && activeUserIP != tvStickIP && currentMillis - lastActivity > controlTimeout) {
    returnToNeutral();
    LOG_INFO("Control timeout, returning to neutral");
  }

  // Check for maximum control time
  if (activeUserIP != "" && activeUserIP != tvStickIP && currentMillis - controlStartTime > maxControlTime) {
    returnToNeutral();
    LOG_INFO("Max control time exceeded, returning to neutral");
  }

  // Update servos and lights
  updateServos();
  updateLights();
  
  // Periodically check WiFi connection status
  static unsigned long lastWifiCheck = 0;
  static bool clientWasConnected = false;
  if (currentMillis - lastWifiCheck > 10000) {
    lastWifiCheck = currentMillis;
    int stationCount = WiFi.softAPgetStationNum();
    if (stationCount == 0 && clientWasConnected) {
      returnToNeutral();
      LOG_INFO("Client lost, returning to neutral");
    }
    clientWasConnected = (stationCount > 0);
  }
}

/**
 * @brief Reset system to neutral state
 */
void returnToNeutral() {
  returningToNeutral = true;
  // Reset all servo target positions
  targetPos1 = 45;
  targetPos2 = 45;
  targetPos3 = 45;
  targetPos4 = 45;
  targetPos5 = 90;
  targetPos6 = 90;
  baseTargetPos3 = 45;
  baseTargetPos4 = 45;
  yawValue = 0;
  flapsMode = false;
  flapsValue = 45;
  
  // Wait for servos to reach neutral positions
  delay(500);
  
  // Detach servos to stop PWM and reduce noise
  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
  servo5.detach();
  servo6.detach();
  
  LOG_INFO("Servos detached");
  
  // Reset system state
  activeUserIP = "";
  authenticatedIP = "";
  controlStartTime = 0;
  isAuthenticated = false;
  cameraAutoMode = false;
  for (int pin : motorPins) digitalWrite(pin, LOW);
  wingMotorsRunning = false;
  setTractionMotor(0);
  tractionMotorRunning = false;
  toggleLights(false);
  LOG_INFO("Returned to neutral, lights disabled");
}

/**
 * @brief Check if client has access to control the drone
 * @return True if access granted, false otherwise
 */
bool checkAccess() {
  String clientIP = server.client().remoteIP().toString();

  // Verify authentication
  if (!isAuthenticated || clientIP != authenticatedIP) {
    server.send(200, "text/html", "<script>window.location.href='http://192.168.4.1/login';</script>");
    LOG_DEBUG("Unauthorized access, redirecting %s", clientIP.c_str());
    return false;
  }

  // Allow access if no one has control
  if (activeUserIP == "") {
    lastActivity = millis();
    return true;
  }

  // Allow access to active user
  if (clientIP == activeUserIP) {
    lastActivity = millis();
    returningToNeutral = false;
    return true;
  }

  // Deny access if another user has control
  server.send(403, "application/json", "{\"status\":\"error\",\"message\":\"Access denied\"}");
  LOG_ERROR("Unauthorized access from %s", clientIP.c_str());
  return false;
}

/**
 * @brief Handle request to gain control of the drone
 */
void handleGetControl() {
  String clientIP = server.client().remoteIP().toString();
  if (!isAuthenticated || clientIP != authenticatedIP) {
    server.send(200, "text/html", "<script>window.location.href='http://192.168.4.1/login';</script>");
    LOG_DEBUG("Unauthorized /get_control from %s", clientIP.c_str());
    return;
  }
  if (activeUserIP == "") {
    activeUserIP = clientIP;
    lastActivity = millis();
    controlStartTime = millis();
    returningToNeutral = false;
    
    // Reattach servos for new user
    servo1.attach(33, 500, 2500);
    servo2.attach(17, 500, 2500);
    servo3.attach(32, 500, 2500);
    servo4.attach(16, 500, 2500);
    servo5.attach(27, 500, 2500);
    servo6.attach(14, 500, 2500);
    
    LOG_INFO("Servos attached for %s", activeUserIP.c_str());
    
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Control granted\"}");
    LOG_INFO("User %s gained control", activeUserIP.c_str());
  } else {
    server.send(423, "application/json", "{\"status\":\"error\",\"message\":\"Control already taken\"}");
    LOG_ERROR("Control already taken, request from %s", clientIP.c_str());
  }
}

/**
 * @brief Handle request to release control of the drone
 */
void handleReleaseControl() {
  String clientIP = server.client().remoteIP().toString();
  if (!isAuthenticated || clientIP != authenticatedIP) {
    server.send(200, "text/html", "<script>window.location.href='http://192.168.4.1/login';</script>");
    LOG_DEBUG("Unauthorized /release_control from %s", clientIP.c_str());
    return;
  }
  if (clientIP == activeUserIP) {
    returnToNeutral();
    toggleLights(false);
    server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Control released\"}");
    LOG_INFO("Control released");
  } else {
    server.send(403, "application/json", "{\"status\":\"error\",\"message\":\"No permission to release control\"}");
    LOG_ERROR("Unauthorized control release from %s", clientIP.c_str());
  }
}

/**
 * @brief Update servo positions with smooth transitions
 */
void updateServos() {
  unsigned long currentMillis = millis();

  // Update servos at specified interval
  if (currentMillis - prevMillis >= interval) {
    prevMillis = currentMillis;
    
    // Smoothly return to neutral if required
    if (returningToNeutral) {
      if (pos1 == 45 && pos2 == 45 && pos3 == 45 && pos4 == 45 && pos5 == 90 && pos6 == 90) {
        returningToNeutral = false;
      } else {
        pos1 += (pos1 < 45) ? step : -step;
        pos2 += (pos2 < 45) ? step : -step;
        pos3 += (pos3 < 45) ? step : -step;
        pos4 += (pos4 < 45) ? step : -step;
        pos5 += (pos5 < 90) ? step : -step;
        pos6 += (pos6 < 90) ? step : -step;
      }
    } else {
      // Move servos towards target positions
      if (pos1 != targetPos1) pos1 += (pos1 < targetPos1) ? step : -step;
      if (pos2 != targetPos2) pos2 += (pos2 < targetPos2) ? step : -step;
      if (pos3 != targetPos3) pos3 += (pos3 < targetPos3) ? step : -step;
      if (pos4 != targetPos4) pos4 += (pos4 < targetPos4) ? step : -step;
      if (pos5 != targetPos5) pos5 += (pos5 < targetPos5) ? step : -step;
      if (pos6 != targetPos6) pos6 += (pos6 < targetPos6) ? step : -step;
    }
    
    // Constrain servo positions to valid ranges
    pos1 = constrain(pos1, 0, 90);
    pos2 = constrain(pos2, 0, 90);
    pos3 = constrain(pos3, 0, 90);
    pos4 = constrain(pos4, 0, 90);
    pos5 = constrain(pos5, 0, 180);
    pos6 = constrain(pos6, 0, 180);
    
    // Update servo positions (invert for right wing and left tail)
    if (servo1.attached()) servo1.write(pos1);
    if (servo2.attached()) servo2.write(90 - pos2);
    if (servo3.attached()) servo3.write(90 - pos3);
    if (servo4.attached()) servo4.write(pos4);
    if (servo5.attached()) servo5.write(pos5);
    if (servo6.attached()) servo6.write(pos6);
    
    // Log current servo positions
    LOG_DEBUG("Servo: %d,%d,%d,%d,%d,%d flaps=%s", pos1, pos2, pos3, pos4, pos5, pos6, flapsMode ? "ON" : "OFF");
  }
}

/**
 * @brief Reset system to initial state
 */
void resetSystem() {
  returnToNeutral();
  LOG_INFO("System reset");
}

/**
 * @brief Serve login page for captive portal
 */
void handleLogin() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Get Started</title>
    <style>
        body { font-family: Arial, sans-serif; background-color: #f0f2f5; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; }
        .container { background-color: #ffffff; padding: 30px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1); width: 100%; max-width: 400px; text-align: center; }
        h1 { color: #333; margin-bottom: 25px; }
        .button { background-color: #007bff; color: white; padding: 12px 25px; border-radius: 5px; text-decoration: none; font-size: 16px; border: none; cursor: pointer; transition: background-color 0.3s ease; }
        .button:hover { background-color: #0056b3; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Welcome!</h1>
        <a href="/start" class="button">Start</a>
    </div>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

/**
 * @brief Serve main control interface
 */
void handleRoot() {
  String clientIP = server.client().remoteIP().toString();
  if (!isAuthenticated || clientIP != authenticatedIP) {
    server.send(200, "text/html", "<script>window.location.href='http://192.168.4.1/login';</script>");
    Serial.println("Unauthorized access, redirecting to /login for client " + clientIP);
    return;
  }
  String html = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Drone Controller PRO</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }
    body {
      font-family: 'Courier New', monospace;
      background: linear-gradient(180deg, #1a1a1a, #2c2c2c);
      color: #ffffff;
      display: flex;
      justify-content: center;
      align-items: center;
      min-height: 100vh;
      padding: 20px;
      overflow-x: hidden;
    }
    .container {
      max-width: 800px;
      width: 100%;
      background: linear-gradient(135deg, #2a2a2a, #3a3a3a);
      padding: 20px;
      border-radius: 15px;
      box-shadow: 0 0 20px rgba(0, 0, 0, 0.5), inset 0 0 10px rgba(255, 255, 255, 0.1);
      border: 1px solid #444;
    }
    h1 {
      text-align: center;
      color: #00ccff;
      text-shadow: 0 0 10px #00ccff;
      margin-bottom: 20px;
      font-size: 2.2em;
    }
    #status {
      text-align: center;
      color: #00ff00;
      background: #1c1c1c;
      padding: 10px;
      border-radius: 5px;
      margin-bottom: 20px;
      font-size: 1.2em;
      box-shadow: inset 0 0 5px #00ff00;
    }
    .control-section, .yaw-control, .camera-control, .traction-control {
      margin: 15px 0;
      padding: 15px;
      background: #222;
      border-radius: 10px;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.3);
      border: 1px solid #555;
    }
    .control-section h3, .yaw-control h3, .camera-control h3, .traction-control h3 {
      color: #ffcc00;
      margin-bottom: 10px;
      text-shadow: 0 0 5px #ffcc00;
    }
    .joystick-area {
      width: 200px;
      height: 200px;
      margin: 20px auto;
      position: relative;
      background: radial-gradient(circle, #333, #1a1a1a);
      border-radius: 50%;
      box-shadow: inset 0 0 20px rgba(0, 0, 0, 0.8), 0 0 10px #00ccff;
      border: 2px solid #444;
    }
    .joystick {
      width: 50px;
      height: 50px;
      background: linear-gradient(135deg, #555, #999);
      border-radius: 50%;
      position: absolute;
      top: 75px;
      left: 75px;
      box-shadow: 0 0 15px #00ccff, inset 0 0 10px #fff;
      cursor: move;
      touch-action: none;
      transition: box-shadow 0.3s ease;
    }
    .joystick:active {
      box-shadow: 0 0 25px #00ccff, inset 0 0 15px #fff;
    }
    .btn {
      background: linear-gradient(135deg, #2ecc71, #27ae60);
      color: #fff;
      border: none;
      padding: 10px 20px;
      margin: 5px;
      font-size: 1em;
      border-radius: 5px;
      cursor: pointer;
      box-shadow: 0 0 10px #2ecc71;
      transition: transform 0.2s, box-shadow 0.2s;
      font-family: 'Courier New', monospace;
    }
    .btn:hover {
      transform: translateY(-2px);
      box-shadow: 0 0 15px #2ecc71;
    }
    .btn-danger {
      background: linear-gradient(135deg, #e74c3c, #c0392b);
      box-shadow: 0 0 10px #e74c3c;
    }
    .btn-danger:hover {
      box-shadow: 0 0 15px #e74c3c;
    }
    .btn-primary {
      background: linear-gradient(135deg, #3498db, #2980b9);
      box-shadow: 0 0 10px #3498db;
    }
    .btn-primary:hover {
      box-shadow: 0 0 15px #3498db;
    }
    .slider-container {
      display: flex;
      align-items: center;
      margin: 10px 0;
      background: #1c1c1c;
      padding: 10px;
      border-radius: 5px;
      box-shadow: inset 0 0 5px #444;
    }
    .slider-label {
      width: 100px;
      color: #00ccff;
      font-size: 1em;
      text-align: right;
      margin-right: 10px;
    }
    .slider {
      -webkit-appearance: none;
      width: 150px;
      height: 8px;
      background: linear-gradient(to right, #444, #888);
      border-radius: 5px;
      outline: none;
      box-shadow: inset 0 0 5px #000;
    }
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 20px;
      height: 20px;
      background: #00ccff;
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 0 10px #00ccff;
      transition: box-shadow 0.2s;
    }
    .slider::-webkit-slider-thumb:hover {
      box-shadow: 0 0 15px #00ccff;
    }
    .slider-value {
      width: 50px;
      text-align: center;
      color: #00ff00;
      background: #1c1c1c;
      padding: 5px;
      border-radius: 3px;
      margin-left: 10px;
    }
    .speed-slider {
      width: 200px;
    }
    .modal {
      display: none;
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background: rgba(0, 0, 0, 0.7);
      justify-content: center;
      align-items: center;
      z-index: 1000;
    }
    .modal-content {
      background: #222;
      padding: 20px;
      border-radius: 10px;
      max-width: 350px;
      width: 90%;
      box-shadow: 0 0 20px #00ccff;
      border: 1px solid #444;
      color: #fff;
    }
    .close {
      position: absolute;
      top: 10px;
      right: 15px;
      font-size: 1.5em;
      color: #e74c3c;
      cursor: pointer;
      transition: color 0.2s;
    }
    .close:hover {
      color: #ff6666;
    }
    select {
      width: 150px;
      padding: 5px;
      background: #1c1c1c;
      color: #fff;
      border: 1px solid #444;
      border-radius: 5px;
      font-family: 'Courier New', monospace;
    }
    select:focus {
      outline: none;
      box-shadow: 0 0 5px #00ccff;
    }
    @media (max-width: 600px) {
      .container {
        padding: 15px;
      }
      .joystick-area {
        width: 150px;
        height: 150px;
      }
      .joystick {
        width: 40px;
        height: 40px;
        top: 55px;
        left: 55px;
      }
      .slider {
        width: 120px;
      }
      .btn {
        padding: 8px 15px;
        font-size: 0.9em;
      }
    }
    .video-section {
      margin: 15px 0;
      padding: 15px;
      background: #222;
      border-radius: 10px;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.3);
      border: 1px solid #555;
      text-align: center;
    }
    .video-section img {
      width: 100%;
      max-width: 640px;
      border-radius: 5px;
      box-shadow: 0 0 10px #00ccff;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Drone Controller PRO</h1>
    
    <!-- Status display -->
    <div id="status">Status: Control not active</div>
    
    <!-- Control buttons for starting/stopping control -->
    <div class="control-section">
      <button class="btn btn-primary" onclick="getControl()">START CONTROL</button>
      <button class="btn btn-danger" onclick="releaseControl()">STOP CONTROL</button>
    </div>
    
    <!-- Video feed from TV stick -->
    <div class="video-section">
      <h3>Camera</h3>
      <img src="http://192.168.4.250:5000/video_feed">
    </div>
    
    <!-- Aileron control section with joystick -->
    <div class="control-section">
      <h3>Aileron Control</h3>
      <p></p>
      <button class="btn" onclick="openFlapsModal()">SET FLAPS</button>
      <div class="joystick-area" id="joystickArea">
        <div class="joystick" id="joystick"></div>
      </div>
    </div>
    
    <!-- Yaw control slider -->
    <div class="yaw-control">
      <h3>Yaw Control</h3>
      <div class="slider-container">
        <span class="slider-label">YAW:</span>
        <input type="range" min="-100" max="100" value="0" class="slider" id="yawSlider">
        <span class="slider-value" id="yawValue">0</span>
      </div>
    </div>
    
    <!-- Camera control sliders for pan and tilt -->
    <div class="camera-control">
      <h3>Camera Control</h3>
      <div class="slider-container" id="panSliderContainer">
        <span class="slider-label">PAN:</span>
        <input type="range" min="0" max="180" value="90" class="slider" id="panSlider">
        <span class="slider-value" id="panValue">90</span>
      </div>
      <div class="slider-container" id="tiltSliderContainer">
        <span class="slider-label">TILT:</span>
        <input type="range" min="0" max="180" value="90" class="slider" id="tiltSlider">
        <span class="slider-value" id="tiltValue">90</span>
      </div>
    </div>
    
    <!-- Motor and light control buttons -->
    <div class="control-section">
      <button class="btn" onclick="sendCommand('/start_motors')">START MOTORS</button>
      <button class="btn btn-danger" onclick="sendCommand('/stop_motors')">STOP MOTORS</button>
      <button class="btn" onclick="sendCommand('/reset')">DISCONNECT</button>
      <button class="btn" onclick="sendCommand('/lights_on')">TURN ON LIGHTS</button>
      <button class="btn btn-danger" onclick="sendCommand('/lights_off')">TURN OFF LIGHTS</button>
    </div>
    
    <!-- Traction motor speed control -->
    <div class="traction-control">
      <h3>Traction Motor</h3>
      <div class="slider-container">
        <span class="slider-label">SPEED:</span>
        <input type="range" min="-55" max="55" value="0" class="speed-slider" id="tractionSlider">
        <span class="slider-value" id="speedValue"></span>
      </div>
      <button class="btn btn-danger" onclick="stopTraction()">STOP</button>
    </div>
    
    <!-- Modal for setting flaps position -->
    <div class="modal" id="flapsModal">
      <div class="modal-content">
        <span class="close" onclick="closeFlapsModal()">×</span>
        <h3>Flaps Position</h3>
        <div class="slider-container">
          <span class="slider-label">FLAPS:</span>
          <input type="range" min="0" max="90" value="45" class="slider" id="flapsSlider">
          <span class="slider-value" id="flapsValue">45</span>
        </div>
        <button class="btn" onclick="resetFlaps()">DISABLE FLAPS</button>
      </div>
    </div>
  </div>

  <script>
    // DOM element references
    const joystick = document.getElementById('joystick');
    const joystickArea = document.getElementById('joystickArea');
    const panSlider = document.getElementById('panSlider');
    const tiltSlider = document.getElementById('tiltSlider');
    const panValue = document.getElementById('panValue');
    const tiltValue = document.getElementById('tiltValue');
    const statusDisplay = document.getElementById('status');
    const tractionSlider = document.getElementById('tractionSlider');
    const speedValue = document.getElementById('speedValue');
    const yawSlider = document.getElementById('yawSlider');
    const yawValue = document.getElementById('yawValue');
    const flapsModal = document.getElementById('flapsModal');
    const flapsSlider = document.getElementById('flapsSlider');
    const flapsValue = document.getElementById('flapsValue');
    const panSliderContainer = document.getElementById('panSliderContainer');
    const tiltSliderContainer = document.getElementById('tiltSliderContainer');
    
    let hasControl = false; // Flag for control status
    let isDragging = false; // Flag for joystick dragging
    let lastJoystickSend = 0; // Throttle timestamp for joystick HTTP requests
    const JOYSTICK_THROTTLE_MS = 50; // Min interval between joystick requests (ms)
    const flightCenter = { x: 100, y: 100 }; // Joystick center position
    const flightMaxMove = 75; // Maximum joystick movement distance

    // Add event listeners for joystick interaction
    joystick.addEventListener('mousedown', startFlightDrag);
    joystick.addEventListener('touchstart', startFlightDrag);
    document.addEventListener('mousemove', drag);
    document.addEventListener('touchmove', drag);
    document.addEventListener('mouseup', stopDrag);
    document.addEventListener('touchend', stopDrag);

    // Add event listeners for sliders
    panSlider.addEventListener('input', updateCameraPosition);
    tiltSlider.addEventListener('input', updateCameraPosition);
    yawSlider.addEventListener('input', updateYawPosition);
    yawSlider.addEventListener('mouseup', resetYawSlider);
    yawSlider.addEventListener('touchend', resetYawSlider);
    flapsSlider.addEventListener('input', updateFlapsPosition);
    tractionSlider.addEventListener('input', function() {
      const speed = this.value;
      speedValue.textContent = speed;
      sendTractionCommand(speed);
    });

    /**
     * @brief Update status display text
     */
    function updateStatus() {
      statusDisplay.textContent = `Status: ${hasControl ? 'Control active' : 'Control not active'}`;
    }

    /**
     * @brief Request control of the drone
     */
    function getControl() {
      fetch('/get_control')
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .then(data => {
          if (data && data.status === 'success') {
            hasControl = true;
            updateStatus();
          }
          if (data) alert(data.message);
        })
        .catch(err => console.error('Error:', err));
    }

    /**
     * @brief Release control of the drone
     */
    function releaseControl() {
      fetch('/release_control')
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .then(data => {
          if (data.status === 'success') {
            hasControl = false;
            updateStatus();
            resetFlightJoystick();
            resetCameraSliders();
            resetYawSlider();
            resetFlaps();
            stopTraction();
          }
          alert(data.message);
        })
        .catch(err => console.error('Error:', err));
    }

    /**
     * @brief Send command to server
     * @param cmd Command URL
     */
    function sendCommand(cmd) {
      if (!hasControl) {
        alert("You do not have control of the drone!");
        return;
      }
      fetch(cmd)
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          if (!response.ok) throw new Error('Server error');
          return response.json();
        })
        .then(data => {
          if (data.status !== 'success') {
            console.error('Error:', data.message);
          }
        })
        .catch(err => console.error('Error:', err));
    }

    /**
     * @brief Send traction motor speed command
     * @param speed Motor speed (-55 to 55)
     */
    function sendTractionCommand(speed) {
      if (!hasControl) {
        alert("You do not have control of the drone!");
        tractionSlider.value = 0;
        speedValue.textContent = '';
        return;
      }
      fetch(`/traction_motor?speed=${speed}`)
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error:', err));
    }
    
    /**
     * @brief Stop traction motor
     */
    function stopTraction() {
      tractionSlider.value = 0;
      speedValue.textContent = '';
      sendTractionCommand(0);
    }

    /**
     * @brief Start joystick drag interaction
     * @param e Mouse or touch event
     */
    function startFlightDrag(e) {
      if (!hasControl) return;
      e.preventDefault();
      isDragging = true;
      updateFlightPosition(e.clientX || e.touches[0].clientX, e.clientY || e.touches[0].clientY);
    }

    /**
     * @brief Handle joystick drag movement
     * @param e Mouse or touch event
     */
    function drag(e) {
      if (!hasControl || !isDragging) return;
      e.preventDefault();
      updateFlightPosition(e.clientX || e.touches[0].clientX, e.clientY || e.touches[0].clientY);
    }

    /**
     * @brief Stop joystick drag
     */
    function stopDrag() {
      if (!hasControl) return;
      if (isDragging) {
        isDragging = false;
        resetFlightJoystick();
      }
    }

    /**
     * @brief Update joystick position and send to server
     * @param clientX X coordinate
     * @param clientY Y coordinate
     */
    function updateFlightPosition(clientX, clientY) {
      const rect = joystickArea.getBoundingClientRect();
      let x = clientX - rect.left - flightCenter.x;
      let y = clientY - rect.top - flightCenter.y;
      
      const distance = Math.min(Math.sqrt(x*x + y*y), flightMaxMove);
      const angle = Math.atan2(y, x);
      
      x = distance * Math.cos(angle);
      y = distance * Math.sin(angle);
      
      // Always update visual joystick position
      joystick.style.transform = `translate(${x}px, ${y}px)`;
      
      // Throttle HTTP requests to avoid flooding ESP32 (fix 5.1)
      const now = Date.now();
      if (now - lastJoystickSend < JOYSTICK_THROTTLE_MS) return;
      lastJoystickSend = now;
      
      const normX = (x / flightMaxMove).toFixed(2);
      const normY = (y / flightMaxMove).toFixed(2);
      
      fetch(`/joystick?x=${normX}&y=${normY}`)
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error:', err));
    }

    /**
     * @brief Update camera pan and tilt positions
     */
    function updateCameraPosition() {
      if (!hasControl) {
        alert("You do not have control of the drone!");
        panSlider.value = 90;
        tiltSlider.value = 90;
        panValue.textContent = '90';
        tiltValue.textContent = '90';
        return;
      }
      const pan = panSlider.value;
      const tilt = tiltSlider.value;
      panValue.textContent = pan;
      tiltValue.textContent = tilt;
      
      fetch(`/camera_control?x=${pan}&y=${tilt}`)
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error (camera):', err));
    }

    /**
     * @brief Reset joystick to center
     */
    function resetFlightJoystick() {
      joystick.style.transform = 'translate(0, 0)';
      fetch('/joystick?x=0&y=0')
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error:', err));
    }

    /**
     * @brief Reset camera sliders to neutral
     */
    function resetCameraSliders() {
      panSlider.value = 90;
      tiltSlider.value = 90;
      panValue.textContent = '90';
      tiltValue.textContent = '90';
      fetch('/camera_control?x=90&y=90')
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error:', err));
    }

    /**
     * @brief Update yaw slider position
     */
    function updateYawPosition() {
      if (!hasControl) {
        alert("You do not have control of the drone!");
        yawSlider.value = 0;
        yawValue.textContent = '0';
        return;
      }
      const value = yawSlider.value / 100;
      yawValue.textContent = yawSlider.value;
      
      fetch(`/yaw_control?value=${value}`)
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error (yaw):', err));
    }

    /**
     * @brief Reset yaw slider to neutral
     */
    function resetYawSlider() {
      yawSlider.value = 0;
      yawValue.textContent = '0';
      fetch('/yaw_control?value=0')
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error (yaw):', err));
    }

    /**
     * @brief Update flaps position
     */
    function updateFlapsPosition() {
      if (!hasControl) {
        alert("You do not have control of the drone!");
        flapsSlider.value = 45;
        flapsValue.textContent = '45';
        return;
      }
      const value = flapsSlider.value;
      flapsValue.textContent = value;
      
      fetch(`/flaps_control?value=${value}`)
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error (flaps):', err));
    }

    /**
     * @brief Open flaps control modal
     */
    function openFlapsModal() {
      if (!hasControl) {
        alert("You do not have control of the drone!");
        return;
      }
      flapsModal.style.display = 'flex';
    }

    /**
     * @brief Close flaps control modal
     */
    function closeFlapsModal() {
      flapsModal.style.display = 'none';
    }

    /**
     * @brief Reset flaps to neutral position
     */
    function resetFlaps() {
      closeFlapsModal();
      flapsSlider.value = 45;
      flapsValue.textContent = '45';
      fetch('/reset_flaps')
        .then(response => {
          if (response.status === 429) {
            alert('Request limit exceeded!');
            return;
          }
          return response.json();
        })
        .catch(err => console.error('Error (reset flaps):', err));
    }

    // Update status display periodically
    setInterval(updateStatus, 2000);
    updateStatus();
  </script>
</body>
</html>
)=====";
  server.send(200, "text/html", html);
}
