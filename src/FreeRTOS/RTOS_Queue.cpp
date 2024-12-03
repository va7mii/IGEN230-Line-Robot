#include <Arduino.h>
#include <stdlib.h>

//Constrict ESP-32 to only one core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//Settings
static const uint8_t msg_queue_len = 5;

//Globals
static QueueHandle_t msg_queue;

//Task: wait for item on quieue and print it
void printMessages(void *parameters) {
  int item;

  while (1) {
    if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {
      Serial.println (item);
    }

    Serial.println(item);

    //Wait for program
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  //Wait for a moment to not miss Serial output
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("--FreeRTOS Queue Demo--");

  //Create queue
  msg_queue = xQueueCreate(msg_queue_len, sizeof(int));

  xTaskCreatePinnedToCore(printMessages,"Print Messages",1024,NULL, 1, NULL, app_cpu);

}

void loop() {
  static int num = 0;

  //Try to add item to queue for 10 ticks, fail if queue is full
  if (xQueueSend(msg_queue, (void*)&num, 10) != pdTRUE) {
    Serial.println("Queue full");
  }

  num++;
  //Wait before trying again
  vTaskDelay(1000 / portTICK_PERIOD_MS);


}
