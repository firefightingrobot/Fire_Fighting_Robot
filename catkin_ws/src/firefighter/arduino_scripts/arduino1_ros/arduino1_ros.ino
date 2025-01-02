#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <DHT.h>

ros::NodeHandle nh;

// Right Ultrasonic Sensor Pins
const int right_trigPin = 8;
const int right_echoPin = 9;
long right_duration;
float right_distance;

// Left Ultrasonic Sensor Pins
const int left_trigPin = 10;
const int left_echoPin = 11;
long left_duration;
float left_distance;

// Gas Sensor MQ2 Pin
const int gas_sensorPin = A0;
float gas_value;

// DHT11 Sensor Pin and Setup
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature;
float humidity;

// Flame Sensor Pins
const int flame_pins[] = {3, 4, 5, 6, 7};
#define NUM_FLAME_SENSORS 5

// ROS Publishers
std_msgs::Float64 right_distance_msg;
ros::Publisher right_pub("right_ultrasonic", &right_distance_msg);

std_msgs::Float64 left_distance_msg;
ros::Publisher left_pub("left_ultrasonic", &left_distance_msg);

std_msgs::Float64 gas_msg;
ros::Publisher gas_pub("gas_sensor", &gas_msg);

std_msgs::Float64 temp_msg;
ros::Publisher temp_pub("temperature", &temp_msg);

std_msgs::Float64 humid_msg;
ros::Publisher humid_pub("humidity", &humid_msg);

std_msgs::String flame_msgs[NUM_FLAME_SENSORS];
ros::Publisher flame_pubs[NUM_FLAME_SENSORS] = {
  ros::Publisher("flame_sensor_0", &flame_msgs[0]),
  ros::Publisher("flame_sensor_1", &flame_msgs[1]),
  ros::Publisher("flame_sensor_2", &flame_msgs[2]),
  ros::Publisher("flame_sensor_3", &flame_msgs[3]),
  ros::Publisher("flame_sensor_4", &flame_msgs[4])
};

// Buzzer Pin (D12)
const int buzzerPin = 12;

// Thresholds for triggering the buzzer
const float TEMP_THRESHOLD = 35.0;   // Temperature threshold in Celsius
const float HUMIDITY_THRESHOLD = 100.0; // Humidity threshold in percentage
const float GAS_THRESHOLD = 350.0;    // Gas sensor threshold (this may need to be adjusted based on the sensor)

// Functions for reading sensor data
void right_readDistance() {
  digitalWrite(right_trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(right_trigPin, LOW);

  right_duration = pulseIn(right_echoPin, HIGH);
  right_distance = right_duration * 0.034 / 2; // Calculate distance in cm

  right_distance_msg.data = right_distance;
  right_pub.publish(&right_distance_msg);
}

void left_readDistance() {
  digitalWrite(left_trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(left_trigPin, LOW);

  left_duration = pulseIn(left_echoPin, HIGH);
  left_distance = left_duration * 0.034 / 2; // Calculate distance in cm

  left_distance_msg.data = left_distance;
  left_pub.publish(&left_distance_msg);
}

void readGasSensor() {
  gas_value = analogRead(gas_sensorPin);
  gas_msg.data = gas_value;
  gas_pub.publish(&gas_msg);
}

void readDHTSensor() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    return; // Skip if sensor data is not available
  }

  humid_msg.data = humidity;
  temp_msg.data = temperature;

  humid_pub.publish(&humid_msg);
  temp_pub.publish(&temp_msg);

  // Check if temperature, humidity, or gas exceed threshold and activate buzzer
  if (temperature > TEMP_THRESHOLD || humidity > HUMIDITY_THRESHOLD || gas_value > GAS_THRESHOLD) {
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
  } else {
    digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
  }
}

void readFlameSensors() {
  for (int i = 0; i < NUM_FLAME_SENSORS; i++) {
    int flame_value = digitalRead(flame_pins[i]);
    if (flame_value == LOW) { // Fire detected (assuming LOW indicates fire)
      String msg = "Fire detected at sensor " + String(i);
      flame_msgs[i].data = msg.c_str(); // Convert String to const char*
    } else {
      String msg = "No fire at sensor " + String(i);
      flame_msgs[i].data = msg.c_str(); // Convert String to const char*
    }
    flame_pubs[i].publish(&flame_msgs[i]);
  }
}

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(right_pub);
  nh.advertise(left_pub);

  for (int i = 0; i < NUM_FLAME_SENSORS; i++) {
    pinMode(flame_pins[i], INPUT);
    nh.advertise(flame_pubs[i]);
  }
  
  nh.advertise(gas_pub);
  nh.advertise(temp_pub);
  nh.advertise(humid_pub);

  pinMode(right_trigPin, OUTPUT);
  pinMode(right_echoPin, INPUT);

  pinMode(left_trigPin, OUTPUT);
  pinMode(left_echoPin, INPUT);

  pinMode(gas_sensorPin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT); // Set D13 as an output for LED
  pinMode(buzzerPin, OUTPUT);   // Set D12 as an output for buzzer
  
  dht.begin();
  
  // Blink LED once
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED
  delay(500); // Keep it on for 500ms
  digitalWrite(LED_BUILTIN, LOW);  // Turn off the LED
}

void loop() {
  nh.spinOnce(); // Spin ROS once to handle communication

  // Start capturing all sensor data
  right_readDistance();  // Read right ultrasonic sensor
  delay(20);             // Short delay to ensure right sensor is captured properly

  left_readDistance();   // Read left ultrasonic sensor
  delay(20);             // Short delay to ensure left sensor is captured properly

  readFlameSensors();    // Read flame sensors
  delay(20);             // Short delay between reading all flame sensors

  readGasSensor();       // Read gas sensor
  delay(20);             // Small delay to allow sensor to settle

  readDHTSensor();       // Read DHT sensor (temperature and humidity)
  delay(20);             // Small delay to allow sensor to settle

  // Optional: Add more delays if necessary for specific sensors or hardware
  delay(100);            // Overall loop delay (100ms) between full sensor readings
  
  digitalWrite(LED_BUILTIN, HIGH); // Keep the LED on permanently (optional)
}
