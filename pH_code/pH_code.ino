#define pHpin A7

// Calibration parameters (derived from sensor data)
float calibration_slope = -2.62;      // slope from calibration
float calibration_intercept = 19.6;     // intercept from calibration

// Buffer for readings (using outlier rejection: average middle 80%)
int buffer_arr[100];  
int buffer_index = 0;
unsigned long start_time = 0;
const unsigned long sampling_interval = 30;  // Sampling every 30ms
const unsigned long duration = 5000;         // 5 seconds sampling duration
unsigned long last_sample_time = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("pH Sensor Calibration Code:");
  delay(2000);  // Initial delay to allow sensor stabilization
}

void loop() {
  unsigned long current_time = millis();

  // Start a new sampling period when the buffer is empty
  if (buffer_index == 0) {
    start_time = current_time;
  }

  // Sample non-blocking every 30ms for 5 seconds
  if ((current_time - last_sample_time >= sampling_interval) && (current_time - start_time < duration)) {
    last_sample_time = current_time;
    buffer_arr[buffer_index++] = analogRead(pHpin);
  }

  // Once the sampling period is over, process the data
  if ((current_time - start_time >= duration) && (buffer_index > 0)) {
    int total_readings = buffer_index;
    unsigned long sum = 0;
    
    // Sort the buffer to discard outliers (simple bubble sort)
    for (int i = 0; i < total_readings - 1; i++) {
      for (int j = i + 1; j < total_readings; j++) {
        if (buffer_arr[i] > buffer_arr[j]) {
          int temp = buffer_arr[i];
          buffer_arr[i] = buffer_arr[j];
          buffer_arr[j] = temp;
        }
      }
    }

    // Average the middle 80% of the readings
    int valid_start = total_readings * 0.1;
    int valid_end = total_readings * 0.9;
    for (int i = valid_start; i < valid_end; i++) {
      sum += buffer_arr[i];
    }
    int avgReading = sum / (valid_end - valid_start);

    // Convert the average analog reading (0-1023) to a voltage
    float voltage = avgReading * (5.0 / 1024.0);

    // Map the voltage to a pH value using the calibration equation
    float pHValue = calibration_slope * voltage + calibration_intercept;
    pHValue = constrain(pHValue, 0.0, 14.0);

    // Print the results
    Serial.print("Average Sensor Reading: ");
    Serial.println(avgReading);
    Serial.print("Voltage: ");
    Serial.println(voltage, 3);
    Serial.print("Calculated pH: ");
    Serial.println(pHValue, 2);

    // Reset for the next sampling period
    buffer_index = 0;
  }
}
