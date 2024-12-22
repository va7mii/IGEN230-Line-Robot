#include <Arduino.h>
#include <stdlib.h>

/**
 * Put code from src to here to compile it and flash it to the ESP-32
 * 
 * Anything from /src is not read by the compiler
 */

// You'll likely need this on vanilla FreeRTOS
//#include <semphr.h>

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Pins (change this if your Arduino board does not have LED_BUILTIN defined)
static const int led_pin = 2;

//Mutex
static SemaphoreHandle_t mutex;

//*****************************************************************************
// Tasks

// Blink LED based on rate passed by parameter
void blinkLED(void *parameters) {

  // Copy the parameter into a local variable
  int num = *(int *)parameters;
  
  //Release mutex so the creating function can finish
  xSemaphoreGive(mutex);

  // Print the parameter
  Serial.print("Received: ");
  Serial.println(num);

  // Configure the LED pin
  pinMode(led_pin, OUTPUT);

  // Blink forever and ever
  while (1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(num / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(num / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {

  long int delay_arg;

  // Configure Serial
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Mutex Challenge---");
  Serial.println("Enter a number for delay (milliseconds)");

  //Create mutex before starting tasks
  mutex = xSemaphoreCreateMutex();

  //"Steal the mutex"
  xSemaphoreTake(mutex, portMAX_DELAY);

  // Wait for input from Serial
  while (Serial.available() <= 0);


  // Read integer value
  delay_arg = Serial.parseInt();
  Serial.print("Sending: ");
  Serial.println(delay_arg);

  // Start task 1
  xTaskCreatePinnedToCore(blinkLED,
                          "Blink LED",
                          1024,
                          (void *)&delay_arg,
                          1,
                          NULL,
                          app_cpu);

  // Show that we accomplished our task of passing the stack-based argument
  Serial.println("Done!");

  //Do nothing until mutex has been returned
  xSemaphoreTake(mutex, portMAX_DELAY);
}

void loop() {
  
  // Do nothing but allow yielding to lower-priority tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
