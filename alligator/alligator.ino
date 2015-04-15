#include <Servo.h>

Servo hip[4];  // create servo object to control a knee and hip joints.
Servo knee[4];
Servo waist, head;

byte dir_hip[4];
byte dir_knee[4];

const byte DIR_FORWARD = 1;
const byte DIR_OUTWARD = 2;
const byte DIR_INWARD = 3;
const byte DIR_BACKWARD_BEFORE_ZERO = 4;
const byte DIR_BACKWARD_AFTER_ZERO = 5;
const byte DIR_NO_MOVEMENT = 6;

const byte DIR_CREEP_BACKWARD_1 = 7;
const byte DIR_CREEP_BACKWARD_2 = 8;
const byte DIR_CREEP_BACKWARD_3 = 9;
const byte DIR_CREEP_BACKWARD_4 = 10;
const byte DIR_CREEP_BACKWARD_5 = 11;
const byte DIR_CREEP_BACKWARD_6 = 12;

const int ANGLE_LEFT_MAX_HIP_POS = 110;
const int ANGLE_LEFT_MIN_HIP_POS = 70;
const int ANGLE_RIGHT_MAX_HIP_POS = 110;
const int ANGLE_RIGHT_MIN_HIP_POS = 70;

const int ANGLE_LEFT_MAX_KNEE_POS = 90;
const int ANGLE_LEFT_MIN_KNEE_POS = 60;
const int ANGLE_RIGHT_MAX_KNEE_POS = 125;
const int ANGLE_RIGHT_MIN_KNEE_POS = 90;

const int ANGLE_ZERO_POS = 90;

const int ANGLE_RIGHT_TURN_MAX_HIP_POS = 105;
const int ANGLE_RIGHT_TURN_MIN_HIP_POS = 75;
const int ANGLE_RIGHT_TURN_MAX_KNEE_POS = 105;
const int ANGLE_RIGHT_TURN_MIN_KNEE_POS = 90;

const int ANGLE_LEFT_TURN_MAX_HIP_POS = 75;
const int ANGLE_LEFT_TURN_MIN_HIP_POS = 105;
const int ANGLE_LEFT_TURN_MAX_KNEE_POS = 90;
const int ANGLE_LEFT_TURN_MIN_KNEE_POS = 75;

const int ANGLE_RIGHT_CREEP_BACK_1 = 110;
const int ANGLE_RIGHT_CREEP_BACK_2 = 100;
const int ANGLE_RIGHT_CREEP_BACK_3 = 90;
const int ANGLE_RIGHT_CREEP_BACK_4 = 80;
const int ANGLE_RIGHT_CREEP_BACK_5 = 70;

const int ANGLE_LEFT_CREEP_BACK_1 = 70;
const int ANGLE_LEFT_CREEP_BACK_2 = 80;
const int ANGLE_LEFT_CREEP_BACK_3 = 90;
const int ANGLE_LEFT_CREEP_BACK_4 = 100;
const int ANGLE_LEFT_CREEP_BACK_5 = 110;

const int ANGLE_RIGHT_TURN_WAIST_POS = 60;
const int ANGLE_LEFT_TURN_WAIST_POS = 120;

const int ANGLE_TURN_MAX_HEAD_POS = 110;
const int ANGLE_TURN_MIN_HEAD_POS = 70;

const int ANGLE_WAIST_CORRECTION = -4;

int NEXT_SERVO_ANGLE[9];
int CURR_SERVO_ANGLE[9];

const byte FUN_TROT_FORWARD = 0;
const byte FUN_TROT_LEFT = 1;
const byte FUN_TROT_RIGHT = 2;
const byte FUN_TROT_BACK = 3;
const byte FUN_CREEP_FORWARD = 4;
const byte FUN_STOP_MOVING = 5;

int DELAY_INIT_TIME = 1000;
int DELAY_TIME = 500;
int DELAY_TIME_1 = 25;

byte function = -1;
byte prev_function = -1;

int Obstacle = 2;

int looping = 0;

int Left_detection = 0;
int Right_detection = 0;

void setup()
{
  //legs anticlockwise
  // 1 0
  // 2 3
  hip[0].attach(11); //Pin assignment for various joints
  hip[1].attach(10);
  hip[2].attach(A2);
  hip[3].attach(A3);
  knee[0].attach(A4);
  knee[1].attach(A5);
  knee[2].attach(3);
  knee[3].attach(5);
  waist.attach(6);
  head.attach(A1);
  //attachInterrupt(2, analyseSensorData, HIGH);
  pinMode(Obstacle, INPUT);
  Serial.begin(9600);
}

void loop()
{
  switch (function) {
    case FUN_TROT_FORWARD: trotFront(); break;
    case FUN_TROT_LEFT: trotLeft(); break;
    case FUN_TROT_RIGHT: trotRight(); break;
    case FUN_TROT_BACK: trotBack(); break;
    case FUN_CREEP_FORWARD: creepFront(); break;
    case FUN_STOP_MOVING: stopMoving(); break;
    default: reset();
  }
  prev_function = function;
  if(looping % 8 == 0)
    function = analyseSensorData();
  looping++;
}

void reset()
{
  hip[0].write(ANGLE_ZERO_POS); //Setting Maximum
  hip[1].write(ANGLE_ZERO_POS);
  hip[2].write(ANGLE_ZERO_POS);
  hip[3].write(ANGLE_ZERO_POS);
  knee[0].write(ANGLE_ZERO_POS);
  knee[1].write(ANGLE_ZERO_POS);
  knee[2].write(ANGLE_ZERO_POS);
  knee[3].write(ANGLE_ZERO_POS);
  waist.write(ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION);
  head.write(ANGLE_ZERO_POS);
  int i;
  for (i = 0; i < 9; i++) {
    CURR_SERVO_ANGLE[i] = ANGLE_ZERO_POS;
  }
  delay(DELAY_INIT_TIME);
}

int analyseSensorData() {
  head.write(ANGLE_ZERO_POS);
  if (digitalRead(Obstacle) == HIGH)
  {
    head.write(ANGLE_TURN_MAX_HEAD_POS);//turn head to the left
    delay(800);
    if (digitalRead(Obstacle) == HIGH)
    {
      //obstacle present on the left
      Left_detection = 10; // 10 = obstacle detected, 0= no obstacle
    }
    head.write(ANGLE_TURN_MIN_HEAD_POS);//turn head to the right
    delay(500);
    if (digitalRead(Obstacle) == HIGH)
    {
      //obstacle present on the right
      Right_detection = 10; // 10 = obstacle detected, 0= no obstacle
    }

    if (Left_detection - Right_detection > 0)
    {
      return FUN_TROT_LEFT;
    }
    else if (Left_detection - Right_detection < 0)
    {
      return FUN_TROT_RIGHT;
    }
    else if (Left_detection == 10 && Right_detection == 10)
    {
      return FUN_TROT_BACK;
    }
  }
  return FUN_TROT_FORWARD;
}

void stopMoving() {
  delay(1000);
}

//START TROT GAIT FUNCTIONS

void resetTrot()
{
  hip[0].write(ANGLE_RIGHT_MIN_HIP_POS);
  CURR_SERVO_ANGLE[0] = ANGLE_RIGHT_MIN_HIP_POS;
  hip[1].write(ANGLE_LEFT_MIN_HIP_POS);
  CURR_SERVO_ANGLE[1] = ANGLE_LEFT_MIN_HIP_POS;
  hip[2].write(ANGLE_LEFT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[2] = ANGLE_LEFT_MAX_HIP_POS;
  hip[3].write(ANGLE_RIGHT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[3] = ANGLE_RIGHT_MAX_HIP_POS;
  knee[0].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[4] = ANGLE_ZERO_POS;
  knee[1].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[5] = ANGLE_ZERO_POS;
  knee[2].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[6] = ANGLE_ZERO_POS;
  knee[3].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[7] = ANGLE_ZERO_POS;
  waist.write(ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION);
  CURR_SERVO_ANGLE[8] = ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION;
  delay(DELAY_INIT_TIME);
}

void initialiseTrotGait() {
  resetTrot();

  dir_hip[0] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[1] = DIR_FORWARD;
  dir_hip[2] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[3] = DIR_FORWARD;

  dir_knee[0] = DIR_NO_MOVEMENT;
  dir_knee[1] = DIR_INWARD;
  dir_knee[2] = DIR_NO_MOVEMENT;
  dir_knee[3] = DIR_INWARD;
}

void trotFront()
{
  if (prev_function != function)
  {
    initialiseTrotGait();
  }
  int i;
  for (i = 0; i < 4; i++) {
    if (i == 0 || i == 3) {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_MAX_HIP_POS;  //hip[i].write(ANGLE_RIGHT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MIN_KNEE_POS; //knee[i].write(ANGLE_RIGHT_MIN_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_MIN_HIP_POS;  //hip[i].write(ANGLE_RIGHT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MAX_KNEE_POS; //knee[i].write(ANGLE_RIGHT_MAX_KNEE_POS);
      }
    } else {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_MIN_HIP_POS;  //hip[i].write(ANGLE_LEFT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MAX_KNEE_POS; //knee[i].write(ANGLE_LEFT_MAX_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_MAX_HIP_POS;  //hip[i].write(ANGLE_LEFT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MIN_KNEE_POS; //knee[i].write(ANGLE_LEFT_MIN_KNEE_POS);
      }
    }
  }
  execute(); //delay(DELAY_TIME);
}

//END TROT GAIT FUNCTIONS

//START CREEP GAIT FUNCTIONS

void resetCreep()
{
  hip[0].write(ANGLE_RIGHT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[0] = ANGLE_RIGHT_MAX_HIP_POS;
  hip[1].write(ANGLE_LEFT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[1] = ANGLE_LEFT_MAX_HIP_POS;
  hip[2].write(ANGLE_LEFT_CREEP_BACK_4);
  CURR_SERVO_ANGLE[2] = ANGLE_LEFT_CREEP_BACK_4;
  hip[3].write(ANGLE_RIGHT_CREEP_BACK_2);
  CURR_SERVO_ANGLE[3] = ANGLE_RIGHT_CREEP_BACK_2;
  knee[0].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[4] = ANGLE_ZERO_POS;
  knee[1].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[5] = ANGLE_ZERO_POS;
  knee[2].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[6] = ANGLE_ZERO_POS;
  knee[3].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[7] = ANGLE_ZERO_POS;
  waist.write(ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION);
  CURR_SERVO_ANGLE[8] = ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION;
  delay(DELAY_INIT_TIME);
}

void initialiseCreepGait() {
  resetCreep();

  dir_hip[0] = DIR_FORWARD;
  dir_hip[1] = DIR_CREEP_BACKWARD_6;
  dir_hip[2] = DIR_CREEP_BACKWARD_4;
  dir_hip[3] = DIR_CREEP_BACKWARD_2;

  dir_knee[0] = DIR_INWARD;
  dir_knee[1] = DIR_NO_MOVEMENT;
  dir_knee[2] = DIR_NO_MOVEMENT;
  dir_knee[3] = DIR_NO_MOVEMENT;
}

void creepFront()
{
  if (prev_function != function)
  {
    initialiseCreepGait();
  }
  int i;
  for (i = 0; i < 4; i++) {
    if (i == 0 || i == 3) {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_MAX_HIP_POS;  //hip[i].write(ANGLE_RIGHT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MIN_KNEE_POS;  //knee[i].write(ANGLE_RIGHT_MIN_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_hip[i] = DIR_CREEP_BACKWARD_1;
        dir_knee[i] = DIR_NO_MOVEMENT;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_CREEP_BACK_1;  //hip[i].write(ANGLE_RIGHT_CREEP_BACK_1);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_1) {
        dir_hip[i] = DIR_CREEP_BACKWARD_2;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_CREEP_BACK_2;  //hip[i].write(ANGLE_RIGHT_CREEP_BACK_2);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_2) {
        dir_hip[i] = DIR_CREEP_BACKWARD_3;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_CREEP_BACK_3;  //hip[i].write(ANGLE_RIGHT_CREEP_BACK_3);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_3) {
        dir_hip[i] = DIR_CREEP_BACKWARD_4;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_CREEP_BACK_4;  //hip[i].write(ANGLE_RIGHT_CREEP_BACK_4);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_4) {
        dir_hip[i] = DIR_CREEP_BACKWARD_5;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_CREEP_BACK_5;  //hip[i].write(ANGLE_RIGHT_CREEP_BACK_5);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_5) {
        dir_hip[i] = DIR_CREEP_BACKWARD_6;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_MIN_HIP_POS;  //hip[i].write(ANGLE_RIGHT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_6) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MAX_KNEE_POS;  //knee[i].write(ANGLE_RIGHT_MAX_KNEE_POS);
      }
    } else {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_MIN_HIP_POS;  //hip[i].write(ANGLE_LEFT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MAX_KNEE_POS;  //knee[i].write(ANGLE_LEFT_MAX_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_hip[i] = DIR_CREEP_BACKWARD_1;
        dir_knee[i] = DIR_NO_MOVEMENT;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_CREEP_BACK_1;  //hip[i].write(ANGLE_LEFT_CREEP_BACK_1);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_1) {
        dir_hip[i] = DIR_CREEP_BACKWARD_2;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_CREEP_BACK_2;  //hip[i].write(ANGLE_LEFT_CREEP_BACK_2);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_2) {
        dir_hip[i] = DIR_CREEP_BACKWARD_3;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_CREEP_BACK_3;  //hip[i].write(ANGLE_LEFT_CREEP_BACK_3);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_3) {
        dir_hip[i] = DIR_CREEP_BACKWARD_4;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_CREEP_BACK_4;  //hip[i].write(ANGLE_LEFT_CREEP_BACK_4);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_4) {
        dir_hip[i] = DIR_CREEP_BACKWARD_5;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_CREEP_BACK_5;  //hip[i].write(ANGLE_LEFT_CREEP_BACK_5);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_5) {
        dir_hip[i] = DIR_CREEP_BACKWARD_6;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_MAX_HIP_POS;  //hip[i].write(ANGLE_LEFT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_CREEP_BACKWARD_6) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MIN_KNEE_POS;  //knee[i].write(ANGLE_LEFT_MIN_KNEE_POS);
      }
    }
  }
  execute(); //delay(DELAY_TIME);
}

//END CREEP GAIT FUNCTIONS

//START LEFT TROT GAIT FUNCTIONS

void resetTrotLeft()
{
  hip[0].write(ANGLE_RIGHT_MIN_HIP_POS);
  CURR_SERVO_ANGLE[0] = ANGLE_RIGHT_MIN_HIP_POS;
  hip[1].write(ANGLE_LEFT_TURN_MIN_HIP_POS);
  CURR_SERVO_ANGLE[1] = ANGLE_LEFT_TURN_MIN_HIP_POS;
  hip[2].write(ANGLE_LEFT_TURN_MAX_HIP_POS);
  CURR_SERVO_ANGLE[2] = ANGLE_LEFT_TURN_MAX_HIP_POS;
  hip[3].write(ANGLE_RIGHT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[3] = ANGLE_RIGHT_MAX_HIP_POS;
  knee[0].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[4] = ANGLE_ZERO_POS;
  knee[1].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[5] = ANGLE_ZERO_POS;
  knee[2].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[6] = ANGLE_ZERO_POS;
  knee[3].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[7] = ANGLE_ZERO_POS;
  waist.write(ANGLE_LEFT_TURN_WAIST_POS);
  CURR_SERVO_ANGLE[8] = ANGLE_LEFT_TURN_WAIST_POS;
  delay(DELAY_INIT_TIME);
}

void initialiseTrotLeftGait() {
  resetTrotLeft();

  dir_hip[0] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[1] = DIR_FORWARD;
  dir_hip[2] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[3] = DIR_FORWARD;

  dir_knee[0] = DIR_NO_MOVEMENT;
  dir_knee[1] = DIR_INWARD;
  dir_knee[2] = DIR_NO_MOVEMENT;
  dir_knee[3] = DIR_INWARD;
}

void trotLeft()
{
  if (prev_function != function)
  {
    initialiseTrotLeftGait();
  }
  int i;
  for (i = 0; i < 4; i++) {
    if (i == 1 || i == 2) {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_TURN_MIN_HIP_POS;  //hip[i].write(ANGLE_LEFT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_TURN_MAX_KNEE_POS;  //knee[i].write(ANGLE_LEFT_MAX_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_TURN_MAX_HIP_POS;  //hip[i].write(ANGLE_LEFT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_TURN_MIN_KNEE_POS;  //knee[i].write(ANGLE_LEFT_MIN_KNEE_POS);
      }
    } else {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_MAX_HIP_POS;  //hip[i].write(ANGLE_RIGHT_TURN_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MIN_KNEE_POS;  //knee[i].write(ANGLE_RIGHT_TURN_MIN_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_MIN_HIP_POS;  //hip[i].write(ANGLE_RIGHT_TURN_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MAX_KNEE_POS;  //knee[i].write(ANGLE_RIGHT_TURN_MAX_KNEE_POS);
      }
    }
  }
  execute(); //delay(DELAY_TIME);
}

//END LEFT TROT GAIT FUNCTIONS

//START RIGHT TROT GAIT FUNCTIONS

void resetTrotRight()
{
  hip[0].write(ANGLE_RIGHT_TURN_MIN_HIP_POS);
  CURR_SERVO_ANGLE[0] = ANGLE_RIGHT_TURN_MIN_HIP_POS;
  hip[1].write(ANGLE_LEFT_MIN_HIP_POS);
  CURR_SERVO_ANGLE[1] = ANGLE_LEFT_MIN_HIP_POS;
  hip[2].write(ANGLE_LEFT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[2] = ANGLE_LEFT_MAX_HIP_POS;
  hip[3].write(ANGLE_RIGHT_TURN_MAX_HIP_POS);
  CURR_SERVO_ANGLE[3] = ANGLE_RIGHT_TURN_MAX_HIP_POS;
  knee[0].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[4] = ANGLE_ZERO_POS;
  knee[1].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[5] = ANGLE_ZERO_POS;
  knee[2].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[6] = ANGLE_ZERO_POS;
  knee[3].write(ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[7] = ANGLE_ZERO_POS;
  waist.write(ANGLE_RIGHT_TURN_WAIST_POS);
  CURR_SERVO_ANGLE[8] = ANGLE_RIGHT_TURN_WAIST_POS;
  delay(DELAY_INIT_TIME);
}

void initialiseTrotRightGait() {
  resetTrotRight();

  dir_hip[0] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[1] = DIR_FORWARD;
  dir_hip[2] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[3] = DIR_FORWARD;

  dir_knee[0] = DIR_NO_MOVEMENT;
  dir_knee[1] = DIR_INWARD;
  dir_knee[2] = DIR_NO_MOVEMENT;
  dir_knee[3] = DIR_INWARD;
}

void trotRight()
{
  if (prev_function != function)
  {
    initialiseTrotRightGait();
  }
  int i;
  for (i = 0; i < 4; i++) {
    if (i == 1 || i == 2) {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_MIN_HIP_POS;  //hip[i].write(ANGLE_LEFT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MAX_KNEE_POS;  //knee[i].write(ANGLE_LEFT_MAX_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_LEFT_MAX_HIP_POS;  //hip[i].write(ANGLE_LEFT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MIN_KNEE_POS;  //knee[i].write(ANGLE_LEFT_MIN_KNEE_POS);
      }
    } else {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_TURN_MAX_HIP_POS;  //hip[i].write(ANGLE_RIGHT_TURN_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_TURN_MIN_KNEE_POS;  //knee[i].write(ANGLE_RIGHT_TURN_MIN_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = ANGLE_RIGHT_TURN_MIN_HIP_POS;  //hip[i].write(ANGLE_RIGHT_TURN_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_TURN_MAX_KNEE_POS;  //knee[i].write(ANGLE_RIGHT_TURN_MAX_KNEE_POS);
      }
    }
  }
  execute(); //delay(DELAY_TIME);
}

//END RIGHT TROT GAIT FUNCTIONS

//START TROT BACK GAIT FUNCTIONS

void resetTrotBack()
{
  hip[0].write(180 - ANGLE_RIGHT_MIN_HIP_POS);
  CURR_SERVO_ANGLE[0] = 180 - ANGLE_RIGHT_MIN_HIP_POS;
  hip[1].write(180 - ANGLE_LEFT_MIN_HIP_POS);
  CURR_SERVO_ANGLE[1] = 180 - ANGLE_LEFT_MIN_HIP_POS;
  hip[2].write(180 - ANGLE_LEFT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[2] = 180 - ANGLE_LEFT_MAX_HIP_POS;
  hip[3].write(180 - ANGLE_RIGHT_MAX_HIP_POS);
  CURR_SERVO_ANGLE[3] = 180 - ANGLE_RIGHT_MAX_HIP_POS;
  knee[0].write(180 - ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[4] = 180 - ANGLE_ZERO_POS;
  knee[1].write(180 - ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[5] = 180 - ANGLE_ZERO_POS;
  knee[2].write(180 - ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[6] = 180 - ANGLE_ZERO_POS;
  knee[3].write(180 - ANGLE_ZERO_POS);
  CURR_SERVO_ANGLE[7] = 180 - ANGLE_ZERO_POS;
  waist.write(ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION);
  CURR_SERVO_ANGLE[8] = ANGLE_ZERO_POS + ANGLE_WAIST_CORRECTION;
  delay(DELAY_INIT_TIME);
}

void initialiseTrotBackGait() {
  resetTrotBack();

  dir_hip[0] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[1] = DIR_FORWARD;
  dir_hip[2] = DIR_BACKWARD_AFTER_ZERO;
  dir_hip[3] = DIR_FORWARD;

  dir_knee[0] = DIR_NO_MOVEMENT;
  dir_knee[1] = DIR_INWARD;
  dir_knee[2] = DIR_NO_MOVEMENT;
  dir_knee[3] = DIR_INWARD;
}

void trotBack()
{
  if (prev_function != function)
  {
    initialiseTrotBackGait();
  }
  int i;
  for (i = 0; i < 4; i++) {
    if (i == 0 || i == 3) {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_RIGHT_MAX_HIP_POS;  //hip[i].write(ANGLE_RIGHT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MIN_KNEE_POS; //knee[i].write(ANGLE_RIGHT_MIN_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_RIGHT_MIN_HIP_POS;  //hip[i].write(ANGLE_RIGHT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_RIGHT_MAX_KNEE_POS; //knee[i].write(ANGLE_RIGHT_MAX_KNEE_POS);
      }
    } else {
      if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_OUTWARD) {
        dir_knee[i] = DIR_INWARD;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_LEFT_MIN_HIP_POS;  //hip[i].write(ANGLE_LEFT_MIN_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MAX_KNEE_POS; //knee[i].write(ANGLE_LEFT_MAX_KNEE_POS);
      } else if (dir_hip[i] == DIR_FORWARD && dir_knee[i] == DIR_INWARD) {
        dir_knee[i] = DIR_NO_MOVEMENT;
        dir_hip[i] = DIR_BACKWARD_BEFORE_ZERO;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_BEFORE_ZERO) {
        dir_hip[i] = DIR_BACKWARD_AFTER_ZERO;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_LEFT_MAX_HIP_POS;  //hip[i].write(ANGLE_LEFT_MAX_HIP_POS);
        NEXT_SERVO_ANGLE[i + 4] = CURR_SERVO_ANGLE[i + 4];
      } else if (dir_hip[i] == DIR_BACKWARD_AFTER_ZERO) {
        dir_hip[i] = DIR_FORWARD;
        dir_knee[i] = DIR_OUTWARD;
        NEXT_SERVO_ANGLE[i] = 180 - ANGLE_ZERO_POS;  //hip[i].write(ANGLE_ZERO_POS);
        NEXT_SERVO_ANGLE[i + 4] = ANGLE_LEFT_MIN_KNEE_POS; //knee[i].write(ANGLE_LEFT_MIN_KNEE_POS);
      }
    }
  }
  execute(); //delay(DELAY_TIME);
}

//END TROT BACK GAIT FUNCTIONS

//EXECUTE
void execute() {
  int i;
  int ANGLE_INCREMENT[8];
  int DIV_ANGLE_INCREMENT = 10;
  if (function == FUN_TROT_LEFT || function == FUN_TROT_RIGHT)
    DIV_ANGLE_INCREMENT = 5;
  for (i = 0; i < 8; i++) {
    ANGLE_INCREMENT[i] = (NEXT_SERVO_ANGLE[i] - CURR_SERVO_ANGLE[i]) / DIV_ANGLE_INCREMENT;
  }

  while (DIV_ANGLE_INCREMENT-- > 0) {
    for (i = 0; i < 4; i++) {
      CURR_SERVO_ANGLE[i] += ANGLE_INCREMENT[i];
      hip[i].write(CURR_SERVO_ANGLE[i]);
      //Serial.println(i + ": " + CURR_SERVO_ANGLE[i]);
    }
    for (i = 0; i < 4; i++) {
      CURR_SERVO_ANGLE[i + 4] += ANGLE_INCREMENT[i + 4];
      knee[i].write(CURR_SERVO_ANGLE[i + 4]);
      //Serial.println((i + 4) + ": " + CURR_SERVO_ANGLE[i + 4]);
    }
    delay(DELAY_TIME_1);
  }
}
