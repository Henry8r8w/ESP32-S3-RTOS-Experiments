#include "freertos/FreeRTOS.h"     // FreeRTOS kernel API
#include "freertos/task.h"         // Task creation/management
#include "freertos/queue.h"        // Queue for inter-task communication

// ==============================================
// Pin Definitions
// ==============================================
#define RED_LED 2      // TODO: Connect Red LED
#define YELLOW_LED 5   // TODO: Connect Yellow LED
#define GREEN_LED 4    // TODO: Connect Green LED

// ==============================================
// Queue Handle
// ==============================================
// TODO: Define a global queue handle (QueueHandle_t)
// This queue will pass traffic light states between tasks.
QueueHandle_t trafficQueue;

// ==============================================
// Enum for Traffic Light States
// ==============================================
// TODO: Define an enum with RED, GREEN, and YELLOW
// This will make your code easier to read than using raw numbers.

typedef enum {
    RED,
    GREEN,
    YELLOW
} TrafficLightState;

// ==============================================
// Traffic Controller Task
// ==============================================
// TODO: In a loop, send the current state to the queue
// Sequence: RED → GREEN → YELLOW → RED → ...
// Use vTaskDelay(pdMS_TO_TICKS(3000)) between state changes
void Task_TrafficController(void *pvParameters) {
    // Your code here
    TrafficLightState state = RED;
    while(1){
      xQueueSend(trafficQueue, &state, portMAX_DELAY);
      // State Transition
      switch(state){
        case RED:
          state = GREEN;
          break;
        case GREEN:
          state = YELLOW;
          break;
        case YELLOW:
          state = RED;
          break;
      }
      // 3 seconds delay
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


// ==============================================
// Traffic Light Task
// ==============================================
// TODO: Wait for a state from the queue.
// Turn OFF all LEDs first, then turn ON only the LED for that state.
// Print the current state to Serial (e.g., "Traffic Light: GREEN (Go)")
void Task_TrafficLight(void *pvParameters) {
    // Your code here
    TrafficLightState state;
    while(1){
      if(xQueueReceive(trafficQueue, &state, 0) == pdTRUE){
        // Turn of the LED off
        digitalWrite(RED_LED, LOW);
        digitalWrite(YELLOW_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        // State Actions
        switch (state) {
                case RED:
                    digitalWrite(RED_LED, HIGH);
                    Serial.println("Traffic Light: RED (Stop)");
                    break;
                case GREEN:
                    digitalWrite(GREEN_LED, HIGH);
                    Serial.println("Traffic Light: GREEN (Go)");
                    break;
                case YELLOW:
                    digitalWrite(YELLOW_LED, HIGH);
                    Serial.println("Traffic Light: YELLOW (Slow Down)");
                    break;
            }
      }
    }
}


// ==============================================
// Setup
// ==============================================
void setup() {
    Serial.begin(115200);

    // TODO: Set all LED pins as OUTPUT
    pinMode(RED_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

    // TODO: Create the queue using xQueueCreate
    // Hint: Queue length can be 2, item size = sizeof(enum)
    trafficQueue = xQueueCreate(2, sizeof(TrafficLightState));

    // TODO: Create both tasks using xTaskCreate
    // Give both tasks the same priority for now
    xTaskCreate(Task_TrafficController, "TrafficController", 2048, NULL, 1, NULL);
    xTaskCreate(Task_TrafficLight, "TrafficLight", 2048, NULL, 1, NULL);
}


// ==============================================
// Loop
// ==============================================
void loop() {
    // Leave empty. FreeRTOS handles task scheduling.
}
