#include "BluetoothSerial.h"  // Library for Bluetooth Serial

BluetoothSerial SerialBT;     // Object for Bluetooth Serial

// Define the analog input pins for the flex sensors
const int flexSensor1 = 34;  // GPIO34
const int flexSensor2 = 35;  // GPIO35
const int flexSensor3 = 32;  // GPIO32
const int flexSensor4 = 33;  // GPIO33
const int flexSensor5 = 36;  // GPIO36 (VP)

// Set the ADC resolution (12 bits for a range of 0-4095)
const int adcMaxValue = 4095; // 12-bit ADC max value

void setup() {
  // Initialize the serial communication at 9600 baud rate
  Serial.begin(9600);
  analogReadResolution(12); // Set ADC resolution to 12 bits

  SerialBT.begin("ESP32_BT"); // Name of the Bluetooth device
  Serial.println("Bluetooth device started, ready to pair");

  // Give a little time to ensure the Bluetooth module is properly initialized
  delay(1000);
}

void loop() {
  // Read the analog values from the flex sensors
  int sensorValue1 = analogRead(flexSensor1);
  int sensorValue2 = analogRead(flexSensor2);
  int sensorValue3 = analogRead(flexSensor3);
  int sensorValue4 = analogRead(flexSensor4);
  int sensorValue5 = analogRead(flexSensor5);

  // Invert the sensor readings to get 0 as the baseline
  sensorValue1 = adcMaxValue - sensorValue1;
  sensorValue2 = adcMaxValue - sensorValue2;
  sensorValue3 = adcMaxValue - sensorValue3;
  sensorValue4 = adcMaxValue - sensorValue4;
  sensorValue5 = adcMaxValue - sensorValue5;

  // Ensure the values are within the range 0 to adcMaxValue
  sensorValue1 = max(sensorValue1, 0);
  sensorValue2 = max(sensorValue2, 0);
  sensorValue3 = max(sensorValue3, 0);
  sensorValue4 = max(sensorValue4, 0);
  sensorValue5 = max(sensorValue5, 0);

  // Print the sensor values to the serial monitor
  Serial.print("Flex Sensor 1: ");
  Serial.print(sensorValue1);
  Serial.print("  Flex Sensor 2: ");
  Serial.print(sensorValue2);
  Serial.print("  Flex Sensor 3: ");
  Serial.print(sensorValue3);
  Serial.print("  Flex Sensor 4: ");
  Serial.print(sensorValue4);
  Serial.print("  Flex Sensor 5: ");
  Serial.println(sensorValue5);

  if(sensorValue1==4095 && sensorValue2<4095 && sensorValue3<4095 && sensorValue4<4095 && sensorValue5<4095)
  if (SerialBT.hasClient()) {
    // Send a message to the connected Bluetooth app
    SerialBT.println("One");
    
    // Wait for a while before sending the next message
    delay(2000); // 2 seconds delay
  }

  // Delay for a short period before the next reading
  delay(500);
}
