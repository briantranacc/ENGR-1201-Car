// Used the uploaded code on blackboard, but there are small modifications
// Libraries used
// Include additional software library to control the motor shield:
#include <Adafruit_MotorShield.h>  // Libary for motor shield
#include <NewPing.h>               // Library for controlling the ultrasonic sensor
#include <Servo.h>                 // Library for controlling servo motor


// Specify the Arduino pins numbers that will connect to the sensor:
#define TRIG_PIN_MID   A0  // Define the trigger pin as analog pin #0  
#define ECHO_PIN_MID   A1  // Define the echo pin as analog pin #1
#define TRIG_PIN_RIGHT   A2  // Define the trigger pin as analog pin #2  
#define ECHO_PIN_RIGHT   A3  // Define the echo pin as analog pin #3
#define TRIG_PIN_LEFT   12  // Define the trigger pin as analog pin #4  
#define ECHO_PIN_LEFT   13  // Define the echo pin as analog pin #5
#define TRIG_PIN_REAR   11  // Define the trigger pin as analog pin #4  
#define ECHO_PIN_REAR   8  // Define the echo pin as analog pin #5

#define BLACK_SERVO_PIN  9  // Defines the pin to use for the servo to pin #9, which is connected to
#define BLUE_SERVO_PIN  10  // Defines the pin to use for the servo to pin #9, which is connected to

Servo       blackServo; 
Servo       blueServo; 

NewPing     midSonar(TRIG_PIN_MID, ECHO_PIN_MID);  
NewPing     rightSonar(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);  
NewPing     leftSonar(TRIG_PIN_LEFT, ECHO_PIN_LEFT);  
NewPing     rearSonar(TRIG_PIN_REAR, ECHO_PIN_REAR);  
Adafruit_MotorShield myMotorShield = Adafruit_MotorShield();


// Assign Adafruit_DCMotor objects to the 4 'ports' (connections) on the motor shield
Adafruit_DCMotor *backRight = myMotorShield.getMotor(1); // Assign 'backRight' to 'port' M1 on motor shield
Adafruit_DCMotor *frontRight = myMotorShield.getMotor(2); // Assign 'frontRight' to 'port' M2 on motor shield
Adafruit_DCMotor *frontLeft = myMotorShield.getMotor(3); // Assign 'frontLeft' to 'port' M3 on motor shield
Adafruit_DCMotor *backLeft = myMotorShield.getMotor(4); // Assign 'backLeft' to 'port' M4 on motor shield

// Variables
int straightDistance;  // Distance to obstacle straight ahead (in cm)
int rightDistance;  // Distance to obstacle ~90 degrees to right
int leftDistance;  // Distance to obstacle ~90 degrees to left
int fixedLeftDist;  // Distance to obstacle ~45 degrees to right
int fixedRightDist;  // Distance to obstacle ~45 degrees to left
int rearStraightDist;
int rearLeftDist;
int rearRightDist;


void setup() 
{
  Serial.begin(9600);  // Starts the serial communication at the specified baud rate
  delay(1500);    // Delays the program for 1.5 seconds before starting

  blackServo.attach(BLACK_SERVO_PIN); // Attaches the servo pin to the servo object (SERVO_2 on the Motor Drive Shield)
  blueServo.attach(BLUE_SERVO_PIN); // Attaches the servo pin to the servo object (SERVO_2 on the Motor Drive Shield)
  blackServo.write(90);         // Straightens the motor and sensor
  delay(500);     // Stabilizes motor and sensor for 500 ms
  blueServo.write(90);         // Straightens the motor and sensor
  delay(500);     // Stabilizes motor and sensor for 500 ms

  straightDistance = midSensor();
  fixedLeftDist = leftSensor();
  fixedRightDist = rightSensor();

  myMotorShield.begin();  // Start the motor_shield with the default PWM frequency (1.6 kHz)
  stop();   // Initially, stop all the DC motors
}

void loop() 
{
  static float currentSpeed = 0;
  static float leftSpeed = 0;
  static float rightSpeed = 0;

  //check_left_and_right();

  // Read distances from sensors
  straightDistance = midSensor();
  fixedLeftDist = leftSensor();
  fixedRightDist = rightSensor();
  rearStraightDist = rearSensor();  // Continuously check the rear sensor
  

  // Main driving logic
  // 28 cm because of the ~3 cm difference from the front of car to the sensor
  if ((straightDistance > 28) && (fixedLeftDist > 25) && (fixedRightDist > 25)) 
  {
    if (currentSpeed < 180)
    {
      currentSpeed += 0.55; // Gradual acceleration
      leftSpeed = currentSpeed; // Keep left and right wheels synchronized
      rightSpeed = currentSpeed;
      forward(currentSpeed);
    }
  } 
  else if ((fixedRightDist - fixedLeftDist) <= 3 && (fixedLeftDist - fixedRightDist) <= 3))
  {
    forward(currentSpeed);
    Serial.println("close distances, going forward");
  }
  else if ((straightDistance < 85)) 
  {
    if (currentSpeed > 60)
    {
      currentSpeed -= 8.25; // Gradual deceleration
      forward(currentSpeed);
    }

    if (straightDistance < 27)
    {
      Serial.println("obstacle in front, stop");
      stop();
      delay(250);    // 1/4 delay
      check_rear();

      if ((rearStraightDist > 10) && (rearRightDist > 10) && (rearLeftDist > 10))
      {
        
        unsigned long startTime = millis();  // Start a timer
        while (millis() - startTime < 1500)  // Reverse for 1.25 seconds
        {
          backward(75);  // Reverse the car
          delay(50);
          Serial.println("REVERSING");

          rearStraightDist = rearSensor();  // Continuously check the rear sensor

          if (rearStraightDist < 20)  // If an obstacle is detected at the rear
          {
            stop();
            Serial.println("Obstacle detected while reversing!");
            break;  // Exit the loop immediately
          }
        }

        stop();  // Stop the car after reversing
        delay(250);

        //stop();  // Stop after reversing
        //delay(250);
        fixedLeftDist = leftSensor();
        fixedRightDist = rightSensor();
        check_left_and_right();

        if (rightDistance > leftDistance)  // Compare distances
        {
          turnRight(125);  // Execute right turn
          delay(550);
          Serial.println("compare then turn right");
        }
        else if (leftDistance > rightDistance)
        {
          turnLeft(125);  // Execute left turn
          delay(550);
          Serial.println("compare then turn left");
        }
      }
    }

    // Adjust left and right speeds based on obstacles
    else if (fixedLeftDist < 65) 
    {
      leftSpeed = constrain(leftSpeed + 12, 0, 180); // Increase left speed for correction
      rightSpeed = currentSpeed; // Keep right wheels steady
      quickRight(rightSpeed, leftSpeed); // Turn right

      if (fixedLeftDist < 33)
      {
        Serial.println("hard right");

        turnRight(160);
        delay(400);
      }
    } 

    else if (fixedRightDist < 65) 
    {
      rightSpeed = constrain(rightSpeed + 12, 0, 180); // Increase right speed for correction
      leftSpeed = currentSpeed; // Keep left wheels steady
      quickLeft(leftSpeed, rightSpeed); // Turn left

      if (fixedRightDist < 33)
      {
        Serial.println("hard left");

        turnLeft(160);
        delay(400);
      }
    }
  }
  // Persist speed adjustments and avoid abrupt resets
  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);
}


// Sets speed of the car
void set_speed(int speed)
{
  backRight->setSpeed(speed);
  frontRight->setSpeed(speed);
  frontLeft->setSpeed(speed);
  backLeft->setSpeed(speed);
} 

// Moves car forward
void forward(int speed)
{
  backRight->setSpeed(speed);
  frontRight->setSpeed(speed);
  frontLeft->setSpeed(speed); 
  backLeft->setSpeed(speed);

  backRight->run(FORWARD);
  frontRight->run(FORWARD);
  frontLeft->run(FORWARD);
  backLeft->run(FORWARD);
}

// Reverses car
void backward(int speed)
{
  set_speed(speed);
  backRight->run(BACKWARD);
  frontRight->run(BACKWARD);
  frontLeft->run(BACKWARD);
  backLeft->run(BACKWARD);
}  

// Turns car right
void turnRight(int speed)
{
  set_speed(speed);
  backRight->run(BACKWARD);
  frontRight->run(BACKWARD);
  frontLeft->run(FORWARD);
  backLeft->run(FORWARD);
}  


// Turns car left
void turnLeft(int speed)
{
  set_speed(speed);
  backRight->run(FORWARD);
  frontRight->run(FORWARD);
  frontLeft->run(BACKWARD);
  backLeft->run(BACKWARD);
}

void quickRight(int right, int left)
{
  backRight->setSpeed(right);
  frontRight->setSpeed(right);
  frontLeft->setSpeed(left);
  backLeft->setSpeed(left);

  backRight->run(FORWARD);
  frontRight->run(FORWARD);
  frontLeft->run(FORWARD);
  backLeft->run(FORWARD);

  Serial.println("quick right");
}  

void quickLeft(int left, int right)
{
  backRight->setSpeed(right);
  frontRight->setSpeed(right);
  frontLeft->setSpeed(left);
  backLeft->setSpeed(left);

  backRight->run(FORWARD);
  frontRight->run(FORWARD);
  frontLeft->run(FORWARD);
  backLeft->run(FORWARD);

  Serial.println("quick left");
}  

// Stops car
void stop() 
{
  backRight->run(RELEASE);
  frontRight->run(RELEASE);
  frontLeft->run(RELEASE);
  backLeft->run(RELEASE);
}

// Function that gives distance of the ultrasonic sensor
int midSensor()
{
  long duration;
  long distance;
  digitalWrite(TRIG_PIN_MID, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_MID, HIGH);
  delayMicroseconds(10);
  //digitalWrite(TRIG_PIN_MID, LOW);
  duration = pulseIn(ECHO_PIN_MID, HIGH);
  distance = (duration * 0.034) / 2;
  //Serial.println(distance);
  return distance;
}

int leftSensor()
{
  long duration;
  long distance;
  digitalWrite(TRIG_PIN_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_LEFT, HIGH);
  delayMicroseconds(10);
  //digitalWrite(TRIG_PIN_MID, LOW);
  duration = pulseIn(ECHO_PIN_LEFT, HIGH);
  distance = (duration * 0.034) / 2;
  Serial.println(distance);
  return distance;
}

int rightSensor()
{
  long duration;
  long distance;
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  //digitalWrite(TRIG_PIN_RIGHT, LOW);
  duration = pulseIn(ECHO_PIN_RIGHT, HIGH);
  distance = (duration * 0.034) / 2;
  //Serial.println(distance);
  return distance;
}

int rearSensor()
{
  long duration;
  long distance;
  digitalWrite(TRIG_PIN_REAR, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_REAR, HIGH);
  delayMicroseconds(10);
  //digitalWrite(TRIG_PIN_REAR, LOW);
  duration = pulseIn(ECHO_PIN_REAR, HIGH);
  distance = (duration * 0.034) / 2;
  //Serial.println(distance);
  return distance;
}

// Function that checks 90 degrees left and right
void check_left_and_right()
{
  // delay() after myservo.write() stabilizes the motors 
  delay(200);
  blackServo.write(0); // Turn ultrasonic sensor 90 degrees to right
  delay(200);      
  rightDistance = midSensor();

  delay(200);
  blackServo.write(90);
  straightDistance = midSensor();              
  delay(200);    

  delay(200);
  blackServo.write(180); // Turn ultrasonic sensor 90 degrees to left
  delay(200); 
  leftDistance = midSensor();

  delay(200);
  blackServo.write(90);   // Turn ultrasonic sensor straight ahead again            
  delay(200);
  straightDistance = midSensor();              
}

void check_rear()
{
  // delay() after myservo.write() stabilizes the motors 
  // Get distance to obstacles straight ahead:
  blueServo.write(90);  // Set servo motor to point straight ahead 
  delay(325);
  rearStraightDist = rearSensor();

  // Get distance to obstacles 45 degrees to right:
  blueServo.write(35);  // Set servo motor to point ~45 degrees to right (increased range)
  delay(325);
  rearRightDist = rearSensor();

  // Swing servo back straight ahead and again get distance to obstacles:
  blueServo.write(90);  // Set servo motor to point straight ahead
  delay(325); 
  rearStraightDist = rearSensor();

  // Get distance to obstacles 45 degrees to left:
  blueServo.write(145);  // Set servo motor to point ~45 degrees to left (increased range)
  delay(325); 
  rearLeftDist = rearSensor();
  
  // Again, get distance to obstacles straight ahead:
  blueServo.write(90);  // Set servo motor to point straight ahead
  delay(325);
  rearStraightDist = rearSensor();
}
