#include "I2C_MPU6886.h"
#include "SparkFunLIS3DH.h"
#include "Wire.h"

#define BUTTON_PIN 2
#define COUNTER_MAX 500
#define SIGNALS_NUMBER 9

const char* START_MESSAGE = "SOS\n";
const char* RECEIVED_MESSAGE = "R\n";
const char* FAULT_MESSAGE = "F\n";
const char* SIGNALS[] = {"MPU6886_ACCELERATION\nX\nmG\n",
                          "MPU6886_ACCELERATION\nY\nmG\n",
                          "MPU6886_ACCELERATION\nZ\nmG\n",
                          "MPU6886_GYRO\nX\ndeg\n",
                          "MPU6886_GYRO\nY\ndeg\n",
                          "MPU6886_GYRO\nZ\ndeg\n",
                          "LIS3DH_ACCELERATION\nX\nmG\n",
                          "LIS3DH_ACCELERATION\nY\nmG\n",    
                          "LIS3DH_ACCELERATION\nZ\nmG\n"};
I2C_MPU6886 IMU_MPU6886;
LIS3DH IMU_LIS3DH;

volatile int counter;
volatile bool switchedOn;
volatile bool isConnectionEstablished;
volatile bool areSignalsNegotiated;
volatile int mappedValue;
volatile float* data;
volatile bool updated_fast_buffer;
volatile int fast_buffer_bytes_to_read;
char fast_buffer[4];

void checkStartUpButton() 
{
  if(digitalRead(BUTTON_PIN) == HIGH) {
    if (counter == COUNTER_MAX)
      switchedOn = !switchedOn;
    if (counter <= COUNTER_MAX)
      counter++;  
  } else {
    counter = 0;
  }
}

bool validate_messages(const char* first_message, const char* second_message, int length_to_compare) {
  for (int i = 0; i < length_to_compare; ++i) {
    if (first_message[i] != second_message[i]) return false;
  }
  return true;
}

int find_character_position(const char* message, char char_to_find, char number_of_char_to_find = 1) {
  int counter = 0;
  int i = 0;
  while (counter < number_of_char_to_find) {
    if (message[i] == char_to_find) ++counter;
    ++i;
  }
  return i;
}

void update_measurmenets(volatile float* data) {
  data[0] = IMU_MPU6886.readFloatAccelX() * 1000;
  data[1] = IMU_MPU6886.readFloatAccelY() * 1000;
  data[2] = IMU_MPU6886.readFloatAccelZ() * 1000;
  data[3] = IMU_MPU6886.readFloatGyroX();
  data[4] = IMU_MPU6886.readFloatGyroY();
  data[5] = IMU_MPU6886.readFloatGyroZ();
  data[6] = IMU_LIS3DH.readFloatAccelX() * 1000;
  data[7] = IMU_LIS3DH.readFloatAccelY() * 1000;
  data[8] = IMU_LIS3DH.readFloatAccelZ() * 1000;
}

void listenOnSerialPort() {
  updated_fast_buffer = false;
  fast_buffer_bytes_to_read = Serial.available();
  if (fast_buffer_bytes_to_read > 0) {
    Serial.readBytes(fast_buffer, fast_buffer_bytes_to_read);
    updated_fast_buffer = true;
  }
  if (isConnectionEstablished) {
    if (areSignalsNegotiated) {
      update_measurmenets(data);
      Serial.flush();
      Serial.write(RECEIVED_MESSAGE, sizeof(RECEIVED_MESSAGE));
      char* float_to_bytes = (char*)data;
      for (int i = 0; i < SIGNALS_NUMBER; ++i) {
        Serial.write(float_to_bytes + (i * sizeof(float)), sizeof(float)/2);
        Serial.write(float_to_bytes + (2 + i * sizeof(float)), sizeof(float)/2);
      }
      while (Serial.available() == 0) {}
      char receivedMessage[2];
      Serial.readBytes(receivedMessage, 2);
    } else {
      Serial.flush();
      char signalNumberMessage[2];
      signalNumberMessage[0] = SIGNALS_NUMBER;
      signalNumberMessage[1] = '\n';
      Serial.write(signalNumberMessage, sizeof(signalNumberMessage));
      while (Serial.available() == 0) {}
      char receivedMessage[2];
      Serial.readBytes(receivedMessage, 2);
      if (validate_messages(receivedMessage, RECEIVED_MESSAGE, sizeof(RECEIVED_MESSAGE))) {
        Serial.flush();
        for (int i = 0; i < SIGNALS_NUMBER; ++i) {
          Serial.write(SIGNALS[i], find_character_position(SIGNALS[i], '\n', 3) + 1);
        }
        while (Serial.available() == 0) {}
        Serial.readBytes(receivedMessage, 2);
        if (validate_messages(receivedMessage, RECEIVED_MESSAGE, sizeof(RECEIVED_MESSAGE))) {
          Serial.write(RECEIVED_MESSAGE, sizeof(RECEIVED_MESSAGE));
          areSignalsNegotiated = true;
          data = new float[9]; 
        } else Serial.write(FAULT_MESSAGE, sizeof(FAULT_MESSAGE));
      } else Serial.write(FAULT_MESSAGE, sizeof(FAULT_MESSAGE));
    }
  }
  else {
    if (updated_fast_buffer) {
      if (validate_messages(fast_buffer, START_MESSAGE, sizeof(START_MESSAGE))) {
        isConnectionEstablished = true;
        Serial.write(RECEIVED_MESSAGE, sizeof(RECEIVED_MESSAGE));
      }
    }
  }
}

void setup()
{
  // VARIABLES
  counter = 0;
  switchedOn = false;
  isConnectionEstablished = false;
  areSignalsNegotiated = false;
  updated_fast_buffer = false;
  fast_buffer_bytes_to_read = 0;
  
  // BUILT IN LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // DIGITAL PINS
  pinMode(BUTTON_PIN, INPUT);

  // SERIAL PORT COMMUNICATION
  Serial.begin(230400);
  delay(1000);

  // I2C
  Wire.begin();

  // MPU6886
  IMU_MPU6886.begin();

  // LIS3DH
  IMU_LIS3DH.begin();
}

void loop()
{
  if (switchedOn) {
    listenOnSerialPort();
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    if (isConnectionEstablished) {
      Serial.flush();
      Serial.write(FAULT_MESSAGE, sizeof(FAULT_MESSAGE));
      isConnectionEstablished = false;
      areSignalsNegotiated = false;
    }
  }
  checkStartUpButton();
  delay(1);
}
