include <Arduino.h>
#include <Wire.h>

/**
 * @brief Configuration constants
 * @ingroup parallel_tasks
 */
const int LED1_PIN = 42; /**< LED pin */
const int SDA_PIN = 8;   /**< I2C SDA pin */
const int SCL_PIN = 9;   /**< I2C SCL pin */
const uint8_t LCD_ADDRESS = 0x27; /**< LCD I2C address */
const uint8_t LCD_BACKLIGHT = 0x08; /**< LCD backlight bit */
const uint8_t ENABLE = 0x04; /**< LCD enable bit */
const uint8_t REGISTER_SEL = 0x01; /**< LCD data mode bit */

/**
 * @brief Shared variables
 * @ingroup parallel_tasks
 */
SemaphoreHandle_t xSemaphore; /**< Binary semaphore */
float lightLevel, sma = 0; /**< Light level and SMA */

/**
 * @defgroup lcd_control LCD Control
 * @brief I2C LCD functions
 * @{
 */

/**
 * @brief Write 4-bit nibble to LCD
 */
bool write4Bits(uint8_t val) {
    Wire.beginTransmission(LCD_ADDRESS);
    Wire.write(val | ENABLE);
    if (Wire.endTransmission() != 0) return false;
    
    delayMicroseconds(1);
    
    Wire.beginTransmission(LCD_ADDRESS);
    Wire.write(val & ~ENABLE);
    if (Wire.endTransmission() != 0) return false;
    
    delayMicroseconds(40);
    return true;
}

/**
 * @brief Send byte to LCD
 */
bool sendByte(uint8_t value, bool isData) {
    uint8_t mode = isData ? REGISTER_SEL : 0x00;
    uint8_t highNib = (value & 0xF0) | mode | LCD_BACKLIGHT;
    uint8_t lowNib = ((value << 4) & 0xF0) | mode | LCD_BACKLIGHT;
    
    if (!write4Bits(highNib) || !write4Bits(lowNib)) return false;
    
    if (!isData && (value == 0x01 || value == 0x02)) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

/**
 * @brief Initialize LCD
 */
bool initLCD() {
    Serial.println("Initializing LCD...");
    vTaskDelay(pdMS_TO_TICKS(50));
    
    write4Bits(0x30 | LCD_BACKLIGHT); vTaskDelay(pdMS_TO_TICKS(5));
    write4Bits(0x30 | LCD_BACKLIGHT); vTaskDelay(pdMS_TO_TICKS(1));
    write4Bits(0x30 | LCD_BACKLIGHT); vTaskDelay(pdMS_TO_TICKS(1));
    write4Bits(0x20 | LCD_BACKLIGHT);
    
    if (!sendByte(0x28, false)) return false;
    if (!sendByte(0x08, false)) return false;
    if (!sendByte(0x01, false)) return false;
    if (!sendByte(0x06, false)) return false;
    if (!sendByte(0x0C, false)) return false;
    
    Serial.println("LCD initialized successfully");
    return true;
}

/**
 * @brief Clear LCD
 */
void clearLCD() {
    sendByte(0x01, false);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/**
 * @brief Print string to LCD
 */
void printToLCD(const char* str, uint8_t col, uint8_t row) {
    setCursor(col, row);
    for (int i = 0; str[i] != '\0' && i < 16; i++) {
        sendByte(str[i], true);
    }
}

/**
 * @brief Set LCD cursor
 */
void setCursor(uint8_t col, uint8_t row) {
    if (row == 0) sendByte(0x80 | col, false);
    else if (row == 1) sendByte(0xC0 | col, false);
}

/** @} */

/**
 * @defgroup parallel_tasks Parallel Tasks
 * @brief FreeRTOS tasks with semaphore synchronization
 * @{
 */

/**
 * @brief Read light level and compute SMA
 */
void lightDetectorTask(void *pvParameters) {
    float lightSamples[5] = {0};
    int sampleIndex = 0;        
    float sum = 0;              
    while (1) {
        lightLevel = analogRead(2);
        sum -= lightSamples[sampleIndex];
        lightSamples[sampleIndex] = lightLevel;
        sum += lightLevel;
        sma = sum / 5.0;
        sampleIndex = (sampleIndex + 1) % 5;
        xSemaphoreGive(xSemaphore);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Display light level and SMA on LCD
 */
void lcdTask(void *pvParameters) {
    char buffer[16];
    while (1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            snprintf(buffer, sizeof(buffer), "Light: %.2f", lightLevel);
            printToLCD(buffer, 0, 0);
            snprintf(buffer, sizeof(buffer), "SMA: %.2f", sma);
            printToLCD(buffer, 0, 1);
        }
        xSemaphoreGive(xSemaphore);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Flash LED on anomalous light levels
 */
void anomalyAlarmTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            if (sma > 3800 || sma < 600) {
                digitalWrite(LED1_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(200));
                digitalWrite(LED1_PIN, LOW);
            }
        }
        xSemaphoreGive(xSemaphore);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Calculate prime numbers up to 5000
 */
void primeCalculationTask(void *pvParameters) {
    while (1) {
        for (int i = 2; i <= 5000; i++) {
            bool isPrime = true;
            for (int j = 2; j*j <= i; j++) {
                if (i % j == 0) {
                    isPrime = false;
                    break;
                }
            }
            if (isPrime) {
                Serial.println(i);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/** @} */

/**
 * @brief Initialize Serial, I2C, LED, and tasks
 */
void setup() {
    Serial.begin(9600);
    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(LED1_PIN, OUTPUT);
    if (!initLCD()) Serial.println("LCD initialization failed");
    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore != NULL) {
        xTaskCreatePinnedToCore(lightDetectorTask, "Light Detector", 2048, NULL, 1, NULL, 0);
        xTaskCreatePinnedToCore(lcdTask, "LCD", 2048, NULL, 1, NULL, 0);
        xTaskCreatePinnedToCore(anomalyAlarmTask, "Anomaly Alarm", 2048, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(primeCalculationTask, "Prime Calc", 2048, NULL, 1, NULL, 1);
    }
}

/**
 * @brief Empty loop
 */
void loop() {
}
