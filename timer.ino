const int LED1_PIN = 5;
const int LED2_PIN  = 6;
const int POT_PIN = 4;
const int intervalLED1 = 500;
const int intervalLED2 = 1000;

unsigned long prev_miliLED1 = 0;
unsigned long prev_miliLED2 = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32-S3!");
  
  // Pin Configuration and Output Signal Configuratino
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);

}

void loop() {
  // call functions on LED1 and LED2 tasks
  taskLED1();
  taskLED2();
  delay(10); 

  // Read analog input from Potentiommeter and print in serial
  int potValue = analogRead(POT_PIN);  
  Serial.print("Potentiometer value: ");
  Serial.println(potValue);
}

void taskLED1(){
  // Declare and initializaed millies function
  unsigned long curr_mili = millis();
  if (curr_mili - prev_miliLED1 >= intervalLED1){ // conditional on interval
    prev_miliLED1 = curr_mili;
    ToggleLED(LED1_PIN); // toggle led if conditional is met
    }
}

void taskLED2(){
  // Declare and initializaed millies function
  unsigned long curr_mili = millis();
  if (curr_mili - prev_miliLED2 >= intervalLED2){ // conditional on interval
    prev_miliLED2 = curr_mili;
    ToggleLED(LED2_PIN); // toggle led if conditional is met
    }
}

void ToggleLED (int pin){
  digitalWrite(pin, !digitalRead(pin)); // toggle led by assign the same pin with opposite read value

}
