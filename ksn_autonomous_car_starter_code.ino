/*
 * Starter code for making an autonomous Arduino car with the Science Buddies Bluebot kit
 * For introductory video see https://youtu.be/qUo6hXSV1b8
 * For circuit diagram and instructions see: https://www.sciencebuddies.org/science-fair-projects/project-ideas/Robotics_p042/robotics/arduino-self-driving-car
 * 
 * 
 * *Important*: this code assumes you use the Arduino pins from the circuit diagram on the Science Buddies website. 
 * If you wire your circuit differently, you will need to adjust your code accordingly. 
 * 
 * This code is intended to help you get started building a self-driving car with the Science Buddies Bluebot kit.
 * It assumes you are using the Bluebot chassis along with the two included infrared (IR) sensors, as well as 
 * an Arduino UNO and a PING ultrasonic distance sensor (purchased separately).
 * 
 * The code will read the sensors and is set up to control the speed and direction of the robot's two motors
 * using an H-bridge integrated circuit (the L293D, included in your Bluebot kit). However, *you* must write
 * an algorithm that determines what the robot will do based on the sensor readings. 
 */

// define pins for H bridge
// to make a motor spin forward, set the forward pin high and the backward pin low
// to make a motor spin backward, set the backward pin high and the forward pin low
// to stop a motor, set both pins low OR set both pins high, OR set the speed to zero (regardless of direction)
// to control motor speed, use the analogWrite() function for a speed control pin
const int Lmotor_forward_pin = 5;    // left motor forward pin is Arduino pin 5
const int Lmotor_backward_pin = 4;   // left motor backward pin is Arduino pin 4
const int Rmotor_forward_pin = 3;    // right motor forward pin is Arduino pin 3
const int Rmotor_backward_pin = 2;   // right motor backward pin is Arduino pin 2
const int Lmotor_speed_pin = 9;      // left motor speed control pin is Arduino pin 9
const int Rmotor_speed_pin = 10;     // right motor speed control pin is Arduino pin 10


// define pin for the ultrasonic sensor
const int trigPin = 7;  
const int echoPin = 8;

// define pins for LEDs
const int leftLEDpin = 6;
const int rightLEDpin = 11;

// define constants for comparing sensor readings
// you will need to calibrate these values for your setup - the sensors can be affected by ambient light
// the Arduino's analog input will read a value between 0-1023. Higher values mean *less* IR light is reflected (dark surface)
const int leftIRthreshold = 200;    // threshold for left IR sensor to detect line
const int rightIRthreshold = 200;   // threshold for right IR sensor to detect line
const int brakingDistance = 8;      // desired automatic stopping distance in centimeters
const int delayTime = 150 ;          //delay time for motor

// define other non-constant variables used in program
// all speeds must be valus between 0 (stopped) to 255 (full speed)
// tweak the speed values to change how fast your robot drives and turns straight (decrease them if it is overshooting turns)
int speedL = 0;            // left motor speed
int speedR = 0;            // right motor speed
int turnspeed1 = 200;      // speed of outer wheel when turning
int turnspeed2 = 25;       // speed of inner wheel when turning
int straightspeed = 130;   // speed of both motors when driving straight
int leftIR = 0;            // left IR sensor reading
int rightIR = 0;           // right IR sensor reading
long distance = 0;         // distance measured by the ultrasonic sensor in centimeters

void setup() { // setup code that only runs once

  // set pin modes
  pinMode(Lmotor_forward_pin, OUTPUT); 
  pinMode(Lmotor_backward_pin, OUTPUT); 
  pinMode(Rmotor_forward_pin, OUTPUT); 
  pinMode(Rmotor_backward_pin, OUTPUT); 
  pinMode(Lmotor_speed_pin, OUTPUT);
  pinMode(Rmotor_speed_pin, OUTPUT); 
  pinMode(leftLEDpin, OUTPUT);
  pinMode(rightLEDpin, OUTPUT);

  // set H bridge pins for robot to drive forward
  digitalWrite(Lmotor_forward_pin, HIGH);
  digitalWrite(Lmotor_backward_pin, LOW);
  digitalWrite(Rmotor_forward_pin, HIGH);
  digitalWrite(Rmotor_backward_pin, LOW);

  // initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() { // code that loops forever

  // get all three sensor readings
  leftIR = analogRead(A1);            // read left IR sensor
  rightIR = analogRead(A0);           // read right IR sensor
  distance = readUltrasonic(); // read ultrasonic sensor and return distance in centimeters

  /*
   * **********   YOUR ALGORITHM HERE!   **********
   * 
   * Based on the sensor readings, write code to control the speed of the left and right motors (the speedL and speedR variables).
   * Your robot should drive straight when both motors spin the same speed. It will turn left when the right motor spins faster than
   * the left motor, and vice versa. It will stop when the speed of both motors is zero. For example, how can you make your robot:
   * 
   * Turn in the correct direction when only one sensor detects a line?
   */
   if (distance > brakingDistance) 
   {
   if ((leftIR >=  leftIRthreshold) && (rightIR >= rightIRthreshold))
   {
     goForward();
     //delay(1.5*delayTime);     
   }  
   else if ((leftIR >= leftIRthreshold) &&  (rightIR <= rightIRthreshold))
   {
     goLeft();
     delay(delayTime);
//     goRight();
//     delay(1.5*delayTime);
     goForward();
   }
   else if ((leftIR <= leftIRthreshold) &&  (rightIR >= rightIRthreshold))
   {
     goRight();
     delay(delayTime);
 //    goLeft();
 //    delay(1.5*delayTime);
     goForward();
   }

   /* Stop if both sensors detect a line (e.g. a crosswalk)?*/
  else if ((leftIR <= leftIRthreshold) &&  (rightIR <= rightIRthreshold))
    {
      stopCar();
      delay(50*delayTime);
      goForward();
    }
    else goForward();
   }
   /* Stop if there is an obstacle too close to the front of the robot?   */
else stopCar();


  // set motor speeds  
  //analogWrite(Lmotor_speed_pin, speedL);
  //analogWrite(Rmotor_speed_pin, speedR);

  // print variables for debugging purposes - useful for calibrating the IR sensors
  Serial.print("Left IR sensor: ");
  Serial.print(leftIR);
  Serial.print("  Right IR sensor: ");
  Serial.print(rightIR);
  Serial.print("  Ultrasonic sensor (cm): ");
  Serial.print(distance);
  Serial.print("  Left motor ");
  Serial.print(speedL);
  Serial.print("  Right motor: ");
  Serial.println(speedR);
  
}

long readUltrasonic(){
  // this code is based on the PING example code. File -> Examples --> 06.Sensors --> Ping
  long duration;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
 
  duration = pulseIn(echoPin, HIGH);
  //Serial.print(duration);

  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return duration / 29 / 2;
}


void stopCar () {
  digitalWrite(Lmotor_forward_pin, LOW);
  digitalWrite(Lmotor_backward_pin, LOW);
  digitalWrite(Rmotor_forward_pin, LOW);
  digitalWrite(Rmotor_backward_pin, LOW);
  digitalWrite(leftLEDpin, LOW);
  digitalWrite(rightLEDpin, LOW);
  analogWrite(Lmotor_speed_pin, 0);
  analogWrite(Rmotor_speed_pin, 0);
}

void goForward () {
  digitalWrite(Lmotor_forward_pin, HIGH);
  digitalWrite(Lmotor_backward_pin, LOW);
  digitalWrite(Rmotor_forward_pin, HIGH);
  digitalWrite(Rmotor_backward_pin, LOW);
  digitalWrite(leftLEDpin, HIGH);
  digitalWrite(rightLEDpin, HIGH);
  analogWrite(Lmotor_speed_pin, straightspeed);
  analogWrite(Rmotor_speed_pin, straightspeed);
}

void goBackward () {
  digitalWrite(Lmotor_forward_pin, LOW);
  digitalWrite(Lmotor_backward_pin, HIGH);
  digitalWrite(Rmotor_forward_pin, LOW);
  digitalWrite(Rmotor_backward_pin, HIGH);
  analogWrite(Lmotor_speed_pin, straightspeed);
  analogWrite(Rmotor_speed_pin, straightspeed);
}

void goLeft () {
  digitalWrite(Lmotor_forward_pin, HIGH);
  digitalWrite(Lmotor_backward_pin, LOW);
  digitalWrite(Rmotor_forward_pin, HIGH);
  digitalWrite(Rmotor_backward_pin, LOW);
  digitalWrite(leftLEDpin, HIGH);
  digitalWrite(rightLEDpin, LOW);
  analogWrite(Lmotor_speed_pin, turnspeed2);
  analogWrite(Rmotor_speed_pin, turnspeed1);
}

void goRight () {
  digitalWrite(Lmotor_forward_pin, HIGH);
  digitalWrite(Lmotor_backward_pin, LOW);
  digitalWrite(Rmotor_forward_pin, HIGH);
  digitalWrite(Rmotor_backward_pin, LOW);
  digitalWrite(leftLEDpin, LOW);
  digitalWrite(rightLEDpin, HIGH);
  analogWrite(Lmotor_speed_pin, turnspeed1);
  analogWrite(Rmotor_speed_pin, turnspeed2);
}

