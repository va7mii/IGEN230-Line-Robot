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
static const int led_delay = 20; //ms
static int duty_cycle = 50;

//Settings
static const int MAX_MESSAGE_LENGTH = 20;

//Pins 
static const int redLED = 4; //red LED


// Blink an LED at led_delay intervals
void toggleLED(void *parameter) {
  while(1) {
    int onTime = duty_cycle * (led_delay / portTICK_PERIOD_MS) / 100;
    int offTime = (100 - duty_cycle) * (led_delay / portTICK_PERIOD_MS) / 100;
    
    digitalWrite(redLED, HIGH);
    vTaskDelay(onTime);
    digitalWrite(redLED, LOW);
    vTaskDelay(offTime);
  }
}

//Reads user input and adjust led_delay
/**
Sources:
https://www.programmingelectronics.com/serial-read/
https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-3-task-scheduling/8fbb9e0b0eed4279a2dd698f02ce125f
*/
void readSerial(void *parameter) {
  while(1) {
    while (Serial.available() > 0) {
      //Place to hold incoming messages
      static char message[MAX_MESSAGE_LENGTH];
      static unsigned int message_pos = 0;
      

      
      char inByte = Serial.read();

      //Is enter typed or message exceeds max size
      if (inByte == '\n' || message_pos > MAX_MESSAGE_LENGTH) {
        //Convert message into an integer
        int number = atoi(message);

        //Checking if number is within allowed
        if (number >= 0 && number <= 100) {
          duty_cycle = number;
          Serial.print("Updated duty cycle to: " );
          Serial.println(duty_cycle);
        } else {
          Serial.println("Invalid value!");
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

void setup() {
  //Configure pin
  pinMode(redLED, OUTPUT);

  //Configure serial and wait for a second
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Multitask PWM Demo");
  Serial.println("Enter duty cycle in percentage");

  xTaskCreatePinnedToCore(readSerial, "readSerial", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(toggleLED, "Toggle LED", 1024, NULL, 1, NULL, app_cpu); //use xTaskCreate() in vanilia FreeRTOS
  Serial.println(led_delay);
  
  //Delete "setup and loop" task
  vTaskDelete(NULL);

}

void loop() {
  // Don't put execution here for RTOS stuff

}
