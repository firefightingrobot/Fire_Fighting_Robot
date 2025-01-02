#include <DHT.h>
#include <TinyGPS++.h>

// Define the DHT11 pin and type
#define DHTPIN 2     // Pin where the DHT11 sensor is connected
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Define the MQ2 gas sensor pin
#define MQ2PIN A0    // Analog pin where the MQ2 sensor is connected

// Flame sensor pins (connected to digital pins)
const int flameSensorPins[5] = {3, 4, 5, 6, 7}; // Flame sensors connected to digital pins 3 to 7

// Create TinyGPS++ object
TinyGPSPlus gps;

// Ultrasonic sensor pins
#define TRIG_RIGHT 8 // Trigger pin for the right ultrasonic sensor
#define ECHO_RIGHT 9 // Echo pin for the right ultrasonic sensor
#define TRIG_LEFT 10 // Trigger pin for the left ultrasonic sensor
#define ECHO_LEFT 11 // Echo pin for the left ultrasonic sensor

// Define the buzzer pin
#define BUZZER_PIN 12

// Threshold values for triggering the buzzer
#define TEMP_THRESHOLD 30.0   // Temperature threshold in Celsius
#define HUMIDITY_THRESHOLD 70.0 // Humidity threshold in percentage
#define GAS_THRESHOLD 300     // Gas sensor threshold value

void setup() {
  // Initialize the Serial Monitor and GPS communication
  Serial.begin(9600);

  // Initialize the DHT11 sensor
  dht.begin();
  delay(2000); // Allow the DHT sensor to stabilize

  // Set flame sensor pins as input
  for (int i = 0; i < 5; i++) {
    pinMode(flameSensorPins[i], INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
  }

  // Set ultrasonic sensor pins
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  // Set buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);

  // Print a startup message
  Serial.println("Gas Sensor (MQ2), DHT11 Sensor, Flame Sensors, Ultrasonic Sensors, and GPS Reading:");
}

long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void loop() {
  // Read the gas sensor value
  int gasValue = analogRead(MQ2PIN);

  // Convert DHT11 sensor values
  float temperature = dht.readTemperature(); // Temperature in Celsius
  float humidity = dht.readHumidity();       // Humidity percentage

  // Process GPS data
  while (Serial.available() > 0) {
    char c = Serial.read();
    gps.encode(c);
  }

  // Print the gas sensor value
  Serial.print("Gas Sensor Value (MQ2): ");
  Serial.println(gasValue);

  // Print the DHT11 sensor values
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Humidity: ");
  Serial.println(humidity);

  // Check flame sensors
  bool flameDetected = false;
  for (int i = 0; i < 5; i++) {
    int sensorState = digitalRead(flameSensorPins[i]); // Read the flame sensor state (HIGH or LOW)

    if (sensorState == HIGH) { // Flame detected
      flameDetected = true;
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(": Flame detected");

    } else {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(": No flame detected");
    }
  }

  // Print GPS data if available
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Waiting for GPS signal...");
  }

  // Read ultrasonic sensor distances
  long distanceRight = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  long distanceLeft = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);

  // Print ultrasonic sensor values
  Serial.print("Right Ultrasonic Sensor: ");
  Serial.print(distanceRight);
  Serial.println(" cm");

  Serial.print("Left Ultrasonic Sensor: ");
  Serial.print(distanceLeft);
  Serial.println(" cm");

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  // Check if any value exceeds the thresholds
  if (temperature > TEMP_THRESHOLD || humidity > HUMIDITY_THRESHOLD || gasValue > GAS_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn the buzzer on
    Serial.println("Warning: Threshold exceeded! Buzzer activated.");
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn the buzzer off
  }

  delay(100);
}
