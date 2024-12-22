/*Purpose: In serial monitor, user can input a blinking rate for LED*/
#include <Arduino.h>
#include <stdlib.h>

//Constrict ESP-32 to only one core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//Settings
const int MAX_MESSAGE_LENGTH = 20;

//Global Variables
static char* msg_ptr = NULL;
static volatile uint8_t msg_flag = 0;

//Prompts user to enter a string and once entered, notifies echoSerial to start
void readSerial(void *parameter) {
  while (1) {
    while (Serial.available() > 0) {
      //Place to hold incoming messages
      static char message[MAX_MESSAGE_LENGTH];
      static unsigned int message_pos = 0;

      char inByte = Serial.read();

      //Is enter typed or message exceeds max size
      if (inByte == '\n' || message_pos > MAX_MESSAGE_LENGTH) {
        //Convert message into an integer
        message[message_pos] = '\0';

        //Debugging output (comment it)
        //Serial.println(message);

        if (msg_flag == 0) {
          
          msg_ptr = (char*) pvPortMalloc((message_pos+1) * sizeof(char));

          //Check if there is stil enough memory in heap
          configASSERT(msg_ptr);      
          
          //Transfers message to msg_ptr
          memcpy(msg_ptr, message, message_pos+1); //hmm one character = one byte ??

          //Notifies other task that message is ready
          msg_flag = 1;
        }

        //Resets message and message_pos
        message_pos = 0;
        memset(message, 0, MAX_MESSAGE_LENGTH);
        
      } else {
        //Store incoming character into message
        message[message_pos] = inByte;
        message_pos++;
      }
    }
  }
}

//Wait until readSerial notifies its ready then echoes the output, then frees up the memory
void echoSerial(void *parameter) {
  while (1) {
    if (msg_flag == 1) {
    
      //Prints the message in the heap memory
      Serial.print("Task received, the message is: ");
      Serial.println(msg_ptr);
    

      // Give amount of free heap memory (uncomment if you'd like to see it)
      Serial.print("Free heap (bytes): ");
      Serial.println(xPortGetFreeHeapSize());


      //Resets task_flag and msg_ptr
      msg_flag = 0;
      msg_ptr = NULL;

      //Free up our allocated memory
      vPortFree(msg_ptr);
  } 
  }
}


void setup() {

  //Configure serial and wait for a second
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Enter a string");

  xTaskCreatePinnedToCore(readSerial, "readSerial", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(echoSerial, "echoSerial", 1024, NULL, 1, NULL, app_cpu);

  

  //Delete "setup and loop" task
  vTaskDelete(NULL);
  

}

void loop() {
  // Don't put execution here for RTOS stuff

}
