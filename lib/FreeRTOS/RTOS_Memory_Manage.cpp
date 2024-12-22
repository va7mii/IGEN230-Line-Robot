/*Purpose: In serial monitor, user can input a blinking rate for LED*/
#include <Arduino.h>
#include <stdlib.h>

//Constrict ESP-32 to only one core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//Variables


//Settings


//Pins 
static const int redLED = 4; //red LED


// Blink an LED at led_delay intervals
void testTask(void *parameter) {
  while(1) {
    int a = 1;
    int b[100];

    for (int i = 0; i < 100; i++) {
      b[i] = a + 1;
    }
    Serial.println(b[0]);

    //Print out remaining stack memory (words)
    Serial.print("High water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    int *ptr = (int*)pvPortMalloc(1024 * sizeof(int));



    //Print out the number of free heap memory bytes before malloc
    Serial.println("Heap before malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());

    //NOTE: Regular malloc is not thread-safe unless using the ESP-IDF

    //One way to prevent heap overflow is to check malloc output
    if (ptr == NULL) {
      Serial.println("Not enough heap.");
    } else {
      //Do something with memory so its not optmized out by the compiler
      for (int i = 0; i < 1024; i++) {
        ptr[i] = 3;
      }
    }

    //Free up our allocated memory
    vPortFree(ptr);


    //Print out the number of free heap memory bytes AFTER malloc
    Serial.println("Heap after malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());

    //Wait for a while
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


void setup() {
  //Configure pin
  pinMode(redLED, OUTPUT);

  //Configure serial and wait for a second
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("FreeRTOS Memory Test");

  xTaskCreatePinnedToCore(testTask, "testTask", 1500, NULL, 1, NULL, app_cpu);

  

  //Delete "setup and loop" task
  vTaskDelete(NULL);
  

}

void loop() {
  // Don't put execution here for RTOS stuff

}
