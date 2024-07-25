#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

const int pwmPins[6] = {25, 18, 19, 21, 22, 23};  // PWM input pins
volatile unsigned long pulseWidths[6] = {0, 0, 0, 0, 0, 0};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const byte START_BYTE = 0x7E;  // Start byte
const byte END_BYTE = 0x7F;    // End byte

void measurePulse(int pinIndex) {
  static unsigned long startTimes[6] = {0, 0, 0, 0, 0, 0};
  if (digitalRead(pwmPins[pinIndex]) == HIGH) {
    startTimes[pinIndex] = micros();
  } else {
    portENTER_CRITICAL_ISR(&mux);
    pulseWidths[pinIndex] = micros() - startTimes[pinIndex];
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR measurePulse0() { measurePulse(0); }
void IRAM_ATTR measurePulse1() { measurePulse(1); }
void IRAM_ATTR measurePulse2() { measurePulse(2); }
void IRAM_ATTR measurePulse3() { measurePulse(3); }
void IRAM_ATTR measurePulse4() { measurePulse(4); }
void IRAM_ATTR measurePulse5() { measurePulse(5); }

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) {
    pinMode(pwmPins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(pwmPins[0]), measurePulse0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmPins[1]), measurePulse1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmPins[2]), measurePulse2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmPins[3]), measurePulse3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmPins[4]), measurePulse4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmPins[5]), measurePulse5, CHANGE);

  // Start the task on the second core
  xTaskCreatePinnedToCore(
    serialTask,       // Function to implement the task
    "SerialTask",     // Name of the task
    10000,            // Stack size in words
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL,             // Task handle
    1                 // Core where the task should run
  );
}

void loop() {
  // The main loop can be empty or perform other tasks
}

void serialTask(void *pvParameters) {
  while (true) {
    portENTER_CRITICAL(&mux);
    unsigned long widths[6];
    for (int i = 0; i < 6; i++) {
      widths[i] = pulseWidths[i];
    }
    portEXIT_CRITICAL(&mux);

    byte data[26];
    data[0] = START_BYTE;
    for (int i = 0; i < 6; i++) {
      data[1 + i * 4] = (byte)(widths[i] >> 24);
      data[2 + i * 4] = (byte)(widths[i] >> 16);
      data[3 + i * 4] = (byte)(widths[i] >> 8);
      data[4 + i * 4] = (byte)widths[i];
    }
    data[25] = END_BYTE;

    Serial.write(data, 26);
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10 milliseconds
  }
}
