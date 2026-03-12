#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>
#include <TaskScheduler.h>
#include <myQueue.hpp>
#define STEPPERS_NUM 3

// 핀 할당
const int STEP_PIN[STEPPERS_NUM] = {2, 4, 6};
const int DIR_PIN[STEPPERS_NUM] = {3, 5, 7};
const int VACUUM_PIN = 15;
const int ENDSTOP_PIN1 = 16;
const int ENDSTOP_PIN2 = 17;
const int ENDSTOP_PIN3 = 18;
const int SERVO_PIN = 13;
const int VACUUM_INTERRUPT_PIN = 9;

Servo myServo;

AccelStepper stepper[STEPPERS_NUM] = {
    AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
    AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2])
};

//커맨드 데이터 구조체
typedef struct COMMAND_DATA {
  char command;
  long data[3];

  COMMAND_DATA() {}
  COMMAND_DATA(char c) { command = c; data[0]=0; data[1]=0; data[2]=0; }
  COMMAND_DATA(char c, long on_off) { command = c; data[0]=on_off; data[1]=0; data[2]=0; }
  COMMAND_DATA(char c, long p0, long p1, long p2) { command = c; data[0]=p0; data[1]=p1; data[2]=p2; }
} COMMAND_DATA;

my_Queue<COMMAND_DATA> command_queue;

#define MAX_SPEED 20000.0
#define MAX_ACCELERATION 10000.0

//  로봇 수행정보 확인용 
enum RobotState {
  STATE_IDLE,          // 대기 중 (큐 확인)
  STATE_HOMING_FIND,   // 엔드스톱 찾는 중
  STATE_MOVING,        // 목표 좌표로 이동 중
  STATE_DOWNMOVE_WAIT  // D명령 수신 후 다음 좌표 입력을 기다리는 중
};
volatile RobotState currentState = STATE_IDLE;

// 인터럽트, 통신용
volatile bool emergency_flag = false; // ISR 안전성을 위한 플래그
#define  MAX_BUFFER_SIZE = 20;
char rx_buffer[MAX_BUFFER_SIZE];
int rx_index = 0;

//  함수 프로토타입
void GetCommandTask();
void ProcessCommand(char* cmdStr);
void RunTask();
void BlinkTask();
void SpeedSynchronization(long position[]);
void VacuumInterrupt();

// 스케줄러 태스크
Scheduler scheduler;
Task tGetCommand(0, TASK_FOREVER, GetCommandTask, &scheduler, false); 
Task tRun(0, TASK_FOREVER, RunTask, &scheduler, false);              
Task tBlink(1000, TASK_FOREVER, BlinkTask, &scheduler, false);

void setup() {
  Serial.begin(115200);

  pinMode(VACUUM_PIN, OUTPUT);
  digitalWrite(VACUUM_PIN, LOW);
  
  pinMode(ENDSTOP_PIN1, INPUT_PULLUP);
  pinMode(ENDSTOP_PIN2, INPUT_PULLUP);
  pinMode(ENDSTOP_PIN3, INPUT_PULLUP);
    
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(VACUUM_INTERRUPT_PIN, INPUT_PULLUP);

  myServo.attach(SERVO_PIN);
  myServo.write(110);

  for (int i = 0; i < STEPPERS_NUM; i++) {
    stepper[i].setMaxSpeed(MAX_SPEED);
    stepper[i].setAcceleration(MAX_ACCELERATION);
  }

  command_queue.clear();
  command_queue.push(COMMAND_DATA('R')); // 부팅 시 초기 영점 세팅

  tGetCommand.restartDelayed(0);
  tRun.restartDelayed(0);
  tBlink.restartDelayed(0);
}

void loop() {
  scheduler.execute(); 
  // 스케줄러만 실행해 어떤 블로킹 상태도 만들지 않음.
}




// 데이터 수신-----------------------------

void GetCommandTask() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      rx_buffer[rx_index] = '\0'; 
      ProcessCommand(rx_buffer);  // \n이 들어왔을 때만 파싱 시작
      rx_index = 0;               // 버퍼 초기화
    } else if (rx_index < MAX_BUFFER_SIZE - 1) {
      if (inChar != '\r') {       // \r 제거
        rx_buffer[rx_index++] = (char)toupper(inChar); // 대문자 변환 후 저장
      }
    }
  }
}




// 데이터 처리-----------------------------
void ProcessCommand(char* cmdStr) {
  if (strlen(cmdStr) == 0) return;
  char cmdType = cmdStr[0];

  // D 명령 상태일 때는 다음 줄의 데이터를 좌표로 인식
  if (currentState == STATE_DOWNMOVE_WAIT) {
    long p[3] = {0, 0, 0};
    char* ptr = cmdStr;
    for (int i = 0; i < 3; i++) {
      p[i] = strtol(ptr, &ptr, 10);
    }
    command_queue.push(COMMAND_DATA('M', p[0], p[1], p[2]));
    currentState = STATE_IDLE; // 다시 정상 상태로 복귀
    return;
  }

  if (cmdType == 'M' || cmdType == 'S') {
    long p[3] = {0, 0, 0};
    char* ptr = cmdStr + 1;
    for (int i = 0; i < 3; i++) {
      p[i] = strtol(ptr, &ptr, 10); 
    }
    command_queue.push(COMMAND_DATA(cmdType, p[0], p[1], p[2]));
  } 
  else if (cmdType == 'G') {
    long val = strtol(cmdStr + 1, NULL, 10);
    command_queue.push(COMMAND_DATA('G', val, 0, 0));
  } 
  else if (cmdType == 'R') {
    command_queue.push(COMMAND_DATA('R'));
  } 
  else if (cmdType == 'D') {

      
    currentState = STATE_DOWNMOVE_WAIT; 
  }
}



// ISR 
void VacuumInterrupt() {
  emergency_flag = true; // 무거운 동작 없이 플래그만 세움
}


// 수행 명령 기준으로만 작동

void RunTask() {
  
  if (emergency_flag) {
    emergency_flag = false;
    Serial.println("FS");
    
    for (int i = 0; i < STEPPERS_NUM; i++) {
      stepper[i].stop(); // 현재 위치에서 급감속 정지 명령
    }
      
    command_queue.clear(); 
      
    command_queue.push_front(COMMAND_DATA('M', 
      stepper[0].currentPosition() + 1000,
      stepper[1].currentPosition() + 1000,
      stepper[2].currentPosition() + 1000));
      
    currentState = STATE_IDLE; 
  }

  
  switch (currentState) {
    case STATE_IDLE:
      if (!command_queue.is_empty()) {
        COMMAND_DATA cmd = command_queue.front();
        command_queue.pop();

        if (cmd.command == 'M') {
          SpeedSynchronization(cmd.data);
          for (int i = 0; i < STEPPERS_NUM; i++) {
            if (cmd.data[i] < -7000 || cmd.data[i] > 2500) {
              Serial.println("Invalid Position");
              return; 
            }
            stepper[i].moveTo(cmd.data[i]);
          }
          currentState = STATE_MOVING;
        } 
        else if (cmd.command == 'S') {
          for (int i = 0; i < STEPPERS_NUM; i++) {
            stepper[i].setMaxSpeed(cmd.data[i]);
            stepper[i].setAcceleration(cmd.data[i] / 2.0);
          }
        } 
        else if (cmd.command == 'R') {
          for (int i = 0; i < STEPPERS_NUM; i++) stepper[i].setSpeed(800);
          currentState = STATE_HOMING_FIND; // 블로킹 while(1) 대신 상태 전환
        } 
        else if (cmd.command == 'G') {
          if (cmd.data[0] == 1) {
            digitalWrite(VACUUM_PIN, HIGH);
            myServo.write(90);
            Serial.println("Vacuum Pump ON");
    
            attachInterrupt(digitalPinToInterrupt(VACUUM_INTERRUPT_PIN), VacuumInterrupt, FALLING);
          } else {
            detachInterrupt(digitalPinToInterrupt(VACUUM_INTERRUPT_PIN));
            digitalWrite(VACUUM_PIN, LOW);
            myServo.write(110);
            Serial.println("Vacuum Pump OFF");
          }
        }
      }
      break;

    case STATE_HOMING_FIND:
      {
        bool allHomed = true;
        if (digitalRead(ENDSTOP_PIN1)) { stepper[0].runSpeed(); allHomed = false; }
        if (digitalRead(ENDSTOP_PIN2)) { stepper[1].runSpeed(); allHomed = false; }
        if (digitalRead(ENDSTOP_PIN3)) { stepper[2].runSpeed(); allHomed = false; }

        if (allHomed) { 
          long tmp[3] = {0, 0, 0};
          for (int i = 0; i < STEPPERS_NUM; i++) {
            stepper[i].setCurrentPosition(-600);
            stepper[i].moveTo(0);
          }
          SpeedSynchronization(tmp);
          currentState = STATE_MOVING; 
        }
      }
      break;

    case STATE_MOVING:
      {
        bool isMoving = false;
        for (int i = 0; i < STEPPERS_NUM; i++) {
          if (stepper[i].distanceToGo() != 0) {
            stepper[i].run(); // 펄스 발생
            isMoving = true;
          }
        }
        if (!isMoving) {
          currentState = STATE_IDLE; 
        }
      }
      break;

    case STATE_DOWNMOVE_WAIT:
      
      break;
  }
}


void SpeedSynchronization(long position[]) {
  long max_dist = 0;
  long distance[3] = {0};
  for (int i = 0; i < 3; i++) {
    distance[i] = abs(stepper[i].currentPosition() - position[i]);
    if (max_dist < distance[i]) max_dist = distance[i];
  }

  for (int i = 0; i < 3; i++) {
    if (distance[i] > 0) {
      stepper[i].setMaxSpeed(MAX_SPEED * (float)distance[i] / (float)max_dist);
      stepper[i].setAcceleration(MAX_ACCELERATION * (float)distance[i] / (float)max_dist);
    }
  }
}

void BlinkTask() {
  static int ledState = LOW;
  digitalWrite(LED_BUILTIN, ledState);
  ledState = !ledState;
}
