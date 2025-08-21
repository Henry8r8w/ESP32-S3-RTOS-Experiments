
#include <Arduino.h>

// PIN
// a pin for Led1, led2, and poteniometer (optional)
const int LED1_PIN = 5;
const int LED2_PIN  = 6;
const int POT_PIN = 4;
const int intervalLED1 = 500;
const int intervalLED2 = 1000;

unsigned long prev_miliLED1 = 0;
unsigned long prev_miliLED2 = 0;

// Function Prototype
void taskA();
void taskB();
void ToggleLED(int pin);
typedef void (*funcptr)(); // define type; some kind of short-hand
void exec_tasks_manual(void (*funcptr)());
void exec_tasks_tcb(int index);

// Task Control Block Implementation
#define STATE_RUNNING 0
#define STATE_READY 1
#define STATE_WAITING 2
#define STATE_INACTIVE 3
#define N_MAX_TASKS 2


typedef struct TCBStruct{
  void (*fptr) (void);
  unsigned short int state;
  unsigned int delay;
} TCBStruct;



// Tasks A and B <=> taskLED1 and taskLED2
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

// Create Pointers to Tasks
funcptr taskA_ptr = taskLED1;
funcptr taskB_ptr = taskLED2;
TCBStruct TaskList[N_MAX_TASKS];

void initalizeTasks() {
  int j = 0;
  TaskList[j].fptr = taskA_ptr;
  TaskList[j].state = STATE_READY;
  TaskList[j].delay = 0;
  j++;
  TaskList[j].fptr = taskB_ptr;
  TaskList[j].state = STATE_READY;
  TaskList[j].delay = 0;
  j++;
  TaskList[j].fptr = NULL;
}

// Part 2: Task Table Scheduler with TCB

void initializeTasks() {
    TaskList[0].fptr = taskA_ptr;
    TaskList[0].state = STATE_READY;
    TaskList[0].delay = 0;
    TaskList[1].fptr = taskB_ptr;
    TaskList[1].state = STATE_READY;
    TaskList[1].delay = 0;
}

void exec_tasks_tcb(int index) {
    TCBStruct* task = &TaskList[index];
    if (task->state == STATE_READY) {
        task->fptr();
    }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32-S3!");
  initalizeTasks();
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  //exec_tasks_manual(taskA_ptr);
  //exec_tasks_manual(taskB_ptr);
  
  for (int i = 0; i < N_MAX_TASKS; i++) {

        exec_tasks_tcb(i);
    }
  delay(10);
}

void exec_tasks_manual(funcptr FuncPtr){ 
  FuncPtr();
}

void ToggleLED (int pin){
  digitalWrite(pin, !digitalRead(pin)); // toggle led by assign the same pin with opposite read value

}

/*
manual: rely not on loop function to do the looping (create exec_task())
automatic: rely on loop function
*/
