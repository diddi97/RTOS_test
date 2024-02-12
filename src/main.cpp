#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <string.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Pin Definition
#define red_led 4
#define grn_led 5

// Global variables
static TaskHandle_t red_task = NULL;
static TaskHandle_t grn_task = NULL;
static SemaphoreHandle_t semaphore;
static int buf[10];

static int led_delay = 500;

// Tasks 
void toggle_red(void *parameter) {
  Serial.println("Toggle red LED task");

  while (1)
  {
    digitalWrite(red_led, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(red_led, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

/// @brief Toggles red LED
/// @param parameter 
void toggle_grn(void *parameter) {
  Serial.println("Toggle green LED task");

  while(1) {
    digitalWrite(grn_led, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(grn_led, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/// @brief Reads value from serial port and stores in led_delay variable
/// @param parameter 
void read_serial(void *parameter) {
  int del = 0;
  while (1)
  {
    if(Serial.available() > 0) {
      del = Serial.parseInt();  
      led_delay = del;
      if(del != 0) {
        Serial.print("LED delay set to: ");
        Serial.println(led_delay);
      }  
    }
  }
}

// Function writes number i to element i in buffer each time it is runs.
void write_to_buf(void *parameter) {
  xSemaphoreGive(semaphore);
  int i = 0;
  buf[i] = i;
  i++;
}

void setup() {
  pinMode(grn_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  Serial.begin(115200);

  Serial.println("Starting FreeRTOS test program");
  for (int i = 0; i < 4; i++) {
    Serial.print(".");
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
  Serial.println("");

  // Create mutexes and semaphores before starting tasks
  semaphore = xSemaphoreCreateCounting(3,0);

  xTaskCreatePinnedToCore(toggle_red,       // function to be called
                          "Toggle red LED", // Name of task
                          1024,             // Stack size (bytes in ESP32, words in FreeRTOS)
                          NULL,             // Parameter to pass to function
                          2,                // Task priority
                          &red_task,        // Task handle
                          app_cpu);

  xTaskCreatePinnedToCore(toggle_grn,       // function to be called
                          "Toggle green LED", // Name of task
                          1024,             // Stack size (bytes in ESP32, words in FreeRTOS)
                          NULL,             // Parameter to pass to function
                          2,                // Task priority
                          &grn_task,             // Task handle
                          app_cpu);  
  
    xTaskCreatePinnedToCore(read_serial,       // function to be called
                          "Read Serial monitor", // Name of task
                          1024,             // Stack size (bytes in ESP32, words in FreeRTOS)
                          NULL,             // Parameter to pass to function
                          2,                // Task priority
                          NULL,             // Task handle
                          app_cpu);

    // xTaskCreatePinnedToCore(write_to_buf,
    //                         "Write to buffer",
    //                         1024,
    //                         NULL,
    //                         2,
    //                         NULL,
    //                         app_cpu);
    // xSemaphoreTake(semaphore, portMAX_DELAY);
}

void loop() {
// Not used
}
