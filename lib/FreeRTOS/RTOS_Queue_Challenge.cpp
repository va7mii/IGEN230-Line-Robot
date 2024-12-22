#include <Arduino.h>
#include <stdlib.h>

//Constrict ESP-32 to only one core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//Settings
static const uint8_t delay_queue_len = 5;
static const uint8_t blink_queue_len = 5;
static const int buf_len = 20; //Maximum length of message size

static const char command[] = "delay "; //Notice the spacing

//Globals
static QueueHandle_t delay_queue;
static QueueHandle_t blink_queue;


typedef struct Message {
  char body[20];
  int count;
} Message;

//Task: wait for item on quieue and print it
void readDelay(void *parameters) {

  while (1) {
    while (Serial.available() > 0) {
      //Reading serial input
      static char message[buf_len];
      static unsigned int messagePos = 0;

      char inByte = Serial.read();

      if (inByte == '\n' || messagePos > buf_len) {
        message[messagePos] = '\0';

        

        //Resets
        messagePos = 0;
        memset(message, 0, buf_len);
      } else {
        message[messagePos] = inByte;
        messagePos++;
      }
    }

    //Queue code
/*
    if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {
      Serial.println (item);
    }

    Serial.println(item);

    //Wait for program
    vTaskDelay(500 / portTICK_PERIOD_MS);*/
  }
}

//Task B: Blinks led with t dleay and sends blinked to blink_msg to queue 2 once there is 100 blinks
void blinkStatus(void *parameters) {
  int item;

  while (1) {
    if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {
      Serial.println (item);
    }

    Serial.println(item);

    //Wait for program
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  static int num = 0;

  //Try to add item to queue for 10 ticks, fail if queue is full
  // if (xQueueSend(msg_queue, (void*)&num, 10) != pdTRUE) {
  //   Serial.println("Queue full");
  // }

  num++;
  //Wait before trying again
  vTaskDelay(1000 / portTICK_PERIOD_MS);

}

void setup() {
  Serial.begin(115200);

  //Wait for a moment to not miss Serial output
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("--FreeRTOS Queue Demo--");

  //Create queue
  delay_queue = xQueueCreate(delay_queue_len, sizeof(int));
  blink_queue = xQueueCreate(blink_queue_len, sizeof(int));

  //Setup two tasks
  xTaskCreatePinnedToCore(readDelay,"Read Delay",1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(blinkStatus,"Blink Status",1024, NULL, 1, NULL, app_cpu);

  //Delete setup and loop
  //xTaskDestroy()

}

void loop() {
  //Avoiding execution here
}
