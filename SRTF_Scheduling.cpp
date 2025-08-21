#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>

// ============================= Global Variables =============================== //
const int LED1_PIN = 42; // ESP32-S3 built-in LED (change to 2 if your board uses pin 2)
const int SDA_PIN = 8;   // I2C SDA (as per your original code)
const int SCL_PIN = 9;   // I2C SCL (as per your original code)
const int LED_FREQ = 5000; // 5 kHz for LED PWM
const int PWM_RES = 12; // 12-bit resolution for LED PWM
const uint8_t LCD_ADDRESS = 0x27; // Typical I2C address for LCD with PCF8574
const uint8_t LCD_BACKLIGHT = 0x08; // Backlight bit for LCD
const uint8_t ENABLE = 0x04; // Enable bit for LCD
const uint8_t REGISTER_SEL = 0x01; // Register select bit for LCD data mode

// Total times for tasks (in ticks)
const TickType_t ledTaskExecutionTime = pdMS_TO_TICKS(500);      // 500 ms
const TickType_t counterTaskExecutionTime = pdMS_TO_TICKS(10000);  // 10 seconds (1-20 * 500ms)
const TickType_t alphabetTaskExecutionTime = pdMS_TO_TICKS(13000); // 13 seconds (A-Z * 500ms)

// Remaining Execution Times
volatile TickType_t remainingLedTime = 0;
volatile TickType_t remainingCounterTime = 0;
volatile TickType_t remainingAlphabetTime = 0;

// Task handles
TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t counterTaskHandle = NULL;
TaskHandle_t alphabetTaskHandle = NULL;

// Task state flags
volatile bool ledTaskRunning = false;
volatile bool counterTaskRunning = false;
volatile bool alphabetTaskRunning = false;

// LED state
volatile bool ledState = false;

//====================== LCD Function Implementations ==========================//

/**
 * Writes a 4-bit nibble to the LCD via I2C, toggling the enable pin.
 */
bool write4Bits(uint8_t val) {
    Wire.beginTransmission(LCD_ADDRESS);
    Wire.write(val | ENABLE);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    delay(100);
    
    Wire.beginTransmission(LCD_ADDRESS);
    Wire.write(val & ~ENABLE);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    delay(400);
    return true;
}

/**
 * Sends a byte to the LCD, either as data or command, by splitting into high/low nibbles.
 */
bool sendByte(uint8_t value, bool isData) {
    uint8_t mode = isData ? REGISTER_SEL : 0x00;
    uint8_t highNib = (value & 0xF0) | mode | LCD_BACKLIGHT;
    uint8_t lowNib = ((value << 4) & 0xF0) | mode | LCD_BACKLIGHT;
    
    if (!write4Bits(highNib) || !write4Bits(lowNib)) {
        return false;
    }
    
    if (!isData && (value == 0x01 || value == 0x02)) {
        vTaskDelay(pdMS_TO_TICKS(2)); // Clear command needs more time
    }
    
    return true;
}

/**
 * Initializes the LCD in 4-bit mode with standard settings.
 */
bool initLCD() {
    Serial.println("Initializing LCD...");
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for LCD power-up
    
    // 4-bit initialization sequence
    write4Bits(0x30 | LCD_BACKLIGHT); vTaskDelay(pdMS_TO_TICKS(5));
    write4Bits(0x30 | LCD_BACKLIGHT); vTaskDelay(pdMS_TO_TICKS(1));
    write4Bits(0x30 | LCD_BACKLIGHT); vTaskDelay(pdMS_TO_TICKS(1));
    write4Bits(0x20 | LCD_BACKLIGHT); // Switch to 4-bit mode
    
    // Configuration commands
    if (!sendByte(0x28, false)) return false; // 4-bit, 2 lines, 5x8 font
    if (!sendByte(0x08, false)) return false; // Display off
    if (!sendByte(0x01, false)) return false; // Clear display
    if (!sendByte(0x06, false)) return false; // Entry mode: cursor increments, no shift
    if (!sendByte(0x0C, false)) return false; // Display on, cursor off, no blinking
    
    Serial.println("LCD initialized successfully");
    return true;
}

/**
 * Clears the LCD display
 */
void clearLCD() {
    sendByte(0x01, false);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/**
 * Prints a string to LCD
 */
void printToLCD(const char* str) {
    clearLCD();
    for (int i = 0; str[i] != '\0'; i++) {
        sendByte(str[i], true);
    }
}

//====================== Task Implementations ==================================//

void ledTask(void *arg) {
    Serial.println("LED Task started");
    while (1) {
        vTaskSuspend(NULL);
        Serial.println("LED Task running");
        ledTaskRunning = true;
        for (int i = 0; i < 4; i++) {
            ledState = !ledState;
            if (ledcWrite(LED1_PIN, ledState ? 4095 : 0) == 0) {
                digitalWrite(LED1_PIN, ledState ? HIGH : LOW);
            }
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
        
        ledTaskRunning = false;
        remainingLedTime = 0; 
        Serial.println("LED Task completed");
    }
}

void counterTask(void *arg) {
    Serial.println("Counter Task started");
    while (1) {
        vTaskSuspend(NULL);  
        Serial.println("Counter Task running");
        counterTaskRunning = true;
        // Count from 1 to 20 on LCD
        for (int count = 1; count <= 20; count++) {
            char buffer[4];
            sprintf(buffer, "%d", count);
            printToLCD(buffer);
            vTaskDelay(pdMS_TO_TICKS(500)); // 500ms per count
        }
        
        counterTaskRunning = false;
        remainingCounterTime = 0; // Mark as completed
        Serial.println("Counter Task completed");
    }
}

void alphabetTask(void *arg) {
    Serial.println("Alphabet Task started");
    while (1) {
        vTaskSuspend(NULL);
        Serial.println("Alphabet Task running");
        alphabetTaskRunning = true;
        for (char letter = 'A'; letter <= 'Z'; letter++) {
            Serial.print("Alphabet: "); Serial.println(letter);
            vTaskDelay(pdMS_TO_TICKS(500)); // 500ms per letter
        }
        alphabetTaskRunning = false;
        remainingAlphabetTime = 0; // Mark as completed
        Serial.println("Alphabet Task completed");
    }
}

void scheduleTasks(void *arg) {
    Serial.println("SRTF Scheduler started");
    
    // Initial setup - set all tasks as ready
    remainingLedTime = ledTaskExecutionTime;
    remainingCounterTime = counterTaskExecutionTime;
    remainingAlphabetTime = alphabetTaskExecutionTime;
    
    while (1) {
        TaskHandle_t nextTask = NULL;
        TickType_t shortestTime = portMAX_DELAY;
        const char* taskName = "None";
        
        // SRTF Algorithm
        if (remainingLedTime > 0 && remainingLedTime < shortestTime) {
            shortestTime = remainingLedTime;
            nextTask = ledTaskHandle;
            taskName = "LED";
        }
        if (remainingCounterTime > 0 && remainingCounterTime < shortestTime) {
            shortestTime = remainingCounterTime;
            nextTask = counterTaskHandle;
            taskName = "Counter";
        }
        if (remainingAlphabetTime > 0 && remainingAlphabetTime < shortestTime) {
            shortestTime = remainingAlphabetTime;
            nextTask = alphabetTaskHandle;
            taskName = "Alphabet";
        }
        
        if (nextTask != NULL) {
            Serial.print("SRTF: Starting ");
            Serial.print(taskName);
            Serial.print(" task (remaining time): ");Serial.print(shortestTime * portTICK_PERIOD_MS); Serial.println(" ms");
            vTaskResume(nextTask); // Resume the task with shortest time
            while ((nextTask == ledTaskHandle && ledTaskRunning) ||
                   (nextTask == counterTaskHandle && counterTaskRunning) ||
                   (nextTask == alphabetTaskHandle && alphabetTaskRunning)) {
            }
            
        } else {
            Serial.println("=== All tasks completed! Starting new cycle ===");
            remainingLedTime = ledTaskExecutionTime;
            remainingCounterTime = counterTaskExecutionTime;
            remainingAlphabetTime = alphabetTaskExecutionTime;
            vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second pause between cycles
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before next scheduling decision
    }
}

void setup() {
    Serial.begin(115200);
    // Initialize I2C with custom pins 
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // 100kHz for LCD stability
    Serial.println("I2C initialized");
    Serial.println("Creating FreeRTOS tasks...");
    
    // Create tasks with appropriate stack sizes
    BaseType_t result;
    
    // Create scheduler task first (highest priority for SRTF control)
    result = xTaskCreatePinnedToCore(
        scheduleTasks,    // Task function
        "SRTF_Scheduler", // Name
        8192,            // Stack size
        NULL,            // Parameters
        3,               // Priority (highest)
        NULL,            // Task handle
        0                // Core 0
    );
    Serial.println("SRTF Scheduler task created");
    
    // Create LED task
    result = xTaskCreatePinnedToCore(
        ledTask,         // Task function
        "LED_Task",      // Name
        4096,           // Stack size
        NULL,           // Parameters
        1,              // Priority
        &ledTaskHandle, // Task handle
        0               // Core 0
    );

    Serial.println("LED task created");
    
    // Create Counter task
    result = xTaskCreatePinnedToCore(
        counterTask,        // Task function
        "Counter_Task",     // Name
        6144,              // Larger stack for LCD operations
        NULL,              // Parameters
        1,                 // Priority
        &counterTaskHandle, // Task handle
        0                  // Core 0
    );

    Serial.println("Counter task created");
    
    // Create Alphabet task
    result = xTaskCreatePinnedToCore(
        alphabetTask,       // Task function
        "Alphabet_Task",    // Name
        4096,              // Stack size
        NULL,              // Parameters
        1,                 // Priority
        &alphabetTaskHandle, // Task handle
        0                  // Core 0
    );
    Serial.println("Alphabet task created");
    
    Serial.println("SRTF scheduling will begin in 2 seconds...");
    Serial.println("Tasks: 1) LED Blinker (500ms), 2) Counter 1-20 on LCD (10s), 3) Alphabet A-Z to Serial (13s)");
    Serial.println("==========================================");
    
    delay(2000); // Give system time to stabilize
}

void loop() {
}
