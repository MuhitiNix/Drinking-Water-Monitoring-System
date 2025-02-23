#include <OneWire.h>
#include <DallasTemperature.h>
#include "GravityTDS.h"

#define TdsSensorPin A8
#define ONE_WIRE_BUS 2
#define pHpin A7
#define TurbSensorPin A5

// Calibration parameters for the pH sensor
float calibration_slope = -2.62;      // slope from calibration
float calibration_intercept = 19.6;     // intercept from calibration
float offset = 8;

// Global variables for non-blocking pH sensor sampling
int pH_buffer[100];
int pH_buffer_index = 0;
unsigned long pH_start_time = 0;
const unsigned long pH_sampling_interval = 30;  // sample every 30ms
const unsigned long pH_duration = 5000;         // sample for 5 seconds
unsigned long pH_last_sample_time = 0;
bool pH_sampling_active = false;
float latestPH = 7.0;  // default pH value

// Other sensor calibration (unchanged)
float OffsetpH = 5.20;
float calibration_value = 21.34 + OffsetpH;  // used by other routines if needed

// Instantiate sensors
OneWire oneWire(ONE_WIRE_BUS);
GravityTDS gravityTds;
DallasTemperature tempSensor(&oneWire);

// Sampling intervals for the overall system
const unsigned long SAMPLE_INTERVAL = 500;     // sample every 500ms
const unsigned long REPORTING_INTERVAL = 15000;  // report every 15 seconds

// Sensor data tracking structures
struct SensorData {
  float sum = 0;
  int count = 0;
};

struct {
  SensorData temperature;
  SensorData tds;
  SensorData ph;
  SensorData turbidity;
} sensorReadings;

unsigned long lastSampleTime = 0;
unsigned long lastReportTime = 0;

// -------------------------
// Helper Sensor Functions
// -------------------------

// Temperature sensor reading
float readTemperature() {
  tempSensor.requestTemperatures(); 
  return tempSensor.getTempCByIndex(0);
}

// TDS sensor reading
float readTDS(float temperature) {
  gravityTds.setTemperature(temperature);  // for temperature compensation
  gravityTds.update();  // sample and calculate
  return gravityTds.getTdsValue();
}

// Turbidity sensor reading
int readTurbidity() {
  int read_ADC = analogRead(TurbSensorPin);
  // Clamp the ADC value between expected limits
  if (read_ADC > 425) read_ADC = 425;
  if (read_ADC < 290) read_ADC = 290;
  return map(read_ADC, 425, 290, 0, 300);
}

// Non-blocking update routine for the pH sensor
void updatePHSensor() {
  unsigned long current_time = millis();
  
  // Start a new pH sampling period if one isn’t active
  if (!pH_sampling_active) {
    pH_sampling_active = true;
    pH_buffer_index = 0;
    pH_start_time = current_time;
    pH_last_sample_time = current_time;
  }
  
  // Sample every pH_sampling_interval during the 5-second window
  if (pH_sampling_active && (current_time - pH_start_time < pH_duration)) {
    if (current_time - pH_last_sample_time >= pH_sampling_interval && pH_buffer_index < 100) {
      pH_buffer[pH_buffer_index++] = analogRead(pHpin);
      pH_last_sample_time = current_time;
    }
  }
  
  // Once the sampling period is over, process the data
  if (pH_sampling_active && (current_time - pH_start_time >= pH_duration) && pH_buffer_index > 0) {
    int total_readings = pH_buffer_index;
    unsigned long sum = 0;
    
    // Sort the buffer using a simple bubble sort
    for (int i = 0; i < total_readings - 1; i++) {
      for (int j = i + 1; j < total_readings; j++) {
        if (pH_buffer[i] > pH_buffer[j]) {
          int temp = pH_buffer[i];
          pH_buffer[i] = pH_buffer[j];
          pH_buffer[j] = temp;
        }
      }
    }
    
    // Average the middle 80% of readings (discard lowest and highest 10%)
    int valid_start = total_readings * 0.1;
    int valid_end = total_readings * 0.9;
    for (int i = valid_start; i < valid_end; i++) {
      sum += pH_buffer[i];
    }
    int avgReading = sum / (valid_end - valid_start);
    
    // Invert the analog reading to compute voltage if needed
    float voltage = (1023 - avgReading) * (5.0 / 1024.0);
    
    // Map the voltage to a pH value using the calibration equation
    float pHValue = calibration_slope * voltage + calibration_intercept - offset;
    pHValue = constrain(pHValue, 0.0, 14.0);
    
    latestPH = pHValue;
    // Reset flag so that a new sampling period will start next time
    pH_sampling_active = false;
  }
}

// -------------------------
// Arduino Setup and Loop
// -------------------------

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600); // For communication with ESP8266
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize TDS sensor
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();
  
  // Initialize sensor pins
  pinMode(pHpin, INPUT);
  pinMode(TurbSensorPin, INPUT);
  
  delay(2000);  // Initial stabilization delay
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update the pH sensor sampling (non-blocking)
  updatePHSensor();
  
  // Sample the sensors at regular intervals (every 500ms)
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    float temperature = readTemperature();
    sensorReadings.temperature.sum += temperature;
    sensorReadings.temperature.count++;
    
    float tds = readTDS(temperature);
    sensorReadings.tds.sum += tds;
    sensorReadings.tds.count++;
    
    // Use the latest pH value from the non-blocking routine
    sensorReadings.ph.sum += latestPH;
    sensorReadings.ph.count++;
    
    int turbidity = readTurbidity();
    sensorReadings.turbidity.sum += turbidity;
    sensorReadings.turbidity.count++;
    
    lastSampleTime = currentTime;
  }
  
  // Report averaged sensor data every 15 seconds
  if (currentTime - lastReportTime >= REPORTING_INTERVAL) {
    float avgTemperature = sensorReadings.temperature.count > 0 ? 
      sensorReadings.temperature.sum / sensorReadings.temperature.count : 0;
    float avgTDS = sensorReadings.tds.count > 0 ? 
      sensorReadings.tds.sum / sensorReadings.tds.count : 0;
    float avgPH = sensorReadings.ph.count > 0 ? 
      sensorReadings.ph.sum / sensorReadings.ph.count : 0;
    float avgTurbidity = sensorReadings.turbidity.count > 0 ? 
      sensorReadings.turbidity.sum / sensorReadings.turbidity.count : 0;
    
    // Format the data packet for transmission and debugging
    String dataPacket = "Temp:" + String(avgTemperature) +
                        "|pH:" + String(avgPH) +
                        "|Turb:" + String(avgTurbidity) +
                        "|TDS:" + String(avgTDS) + "\n";
    
    Serial3.print(dataPacket);
    Serial.println(dataPacket);
    
    // Reset accumulated sensor readings for the next reporting interval
    sensorReadings.temperature = {};
    sensorReadings.tds = {};
    sensorReadings.ph = {};
    sensorReadings.turbidity = {};
    
    lastReportTime = currentTime;
  }
}
