// Used the uploaded code on blackboard, but there are small modifications
// Libraries used
#include <Adafruit_MotorShield.h>  // Libary for motor shield
#include <Servo.h>                 // Library for controlling servo motor
#include <NewPing.h>               // Library for controlling the ultrasonic sensor

// Define pins for ultrasonic sensor:
#define TRIG_PIN   A0  // Define the trigger pin as analog pin #0  
#define ECHO_PIN   A1  // Define the echo pin as analog pin #1

// Define pin for servo motor:
#define SERVO_PIN  9  // Defines the pin to use for the servo to pin #9, which is connected to
                      // pins labelled 'SERVO_2' on the Motor Drive Shield

// Objects for servo motor, motorshield, and ultrasonic sensor, respectively.
Servo       myservo; 
Adafruit_MotorShield myMotorShield = Adafruit_MotorShield();
NewPing     sonar(TRIG_PIN, ECHO_PIN);  

// Assign Adafruit_DCMotor objects to the 4 'ports' (connections) on the motor shield
Adafruit_DCMotor *backRight = myMotorShield.getMotor(1); // Assign 'backRight' to 'port' M1 on motor shield
Adafruit_DCMotor *frontRight = myMotorShield.getMotor(2); // Assign 'frontRight' to 'port' M2 on motor shield
Adafruit_DCMotor *frontLeft = myMotorShield.getMotor(3); // Assign 'frontLeft' to 'port' M3 on motor shield
Adafruit_DCMotor *backLeft = myMotorShield.getMotor(4); // Assign 'backLeft' to 'port' M4 on motor shield

// Variables
float straightDistance;  // Distance to obstacle straight ahead (in cm)
float rightDistance;  // Distance to obstacle ~90 degrees to right
float leftDistance;  // Distance to obstacle ~90 degrees to left
float rightCenterDistance;  // Distance to obstacle ~45 degrees to right
float leftCenterDistance ;  // Distance to obstacle ~45 degrees to left
int carSpeed = 90;
int turningSpeed = 125;

void setup() 
{
  delay(1500);    // Delays the program for 1.5 seconds before starting
  Serial.begin(9600);  // Starts the serial communication at the specified baud rate
  myservo.attach(SERVO_PIN); // Attaches the servo pin to the servo object (SERVO_2 on the Motor Drive Shield)
  myservo.write(96.5);         // Straightens the motor and sensor
  delay(500);     // Stabilizes motor and sensor for 500 ms
  myMotorShield.begin();  // Start the motor_shield with the default PWM frequency (1.6 kHz)
  stop();   // Initially, stop all the DC motors
}

void loop() 
{
  check_for_obstacles(); // Constantly checks for obstacles
  if ((straightDistance > 30.5) && (rightCenterDistance > 20.5) && (leftCenterDistance > 20.5)) 
  {
    // Obstacles in all 3 directions are far away, so move forward:
    forward(carSpeed);
  }
  else
  { 
    // When obstacles are too close, ensure that motors are stopped, then back up
    // slightly, and then determine which direction to turn the car:
    stop();
    backward(carSpeed);
    delay(300);   // Go backwards for 300 ms
    stop();       // Stops after reversing

    // Check for obstacles ~45 degrees to left and right and check ~90 degrees to left and right
    // Turn car towards direction that has more open space:
    check_for_obstacles();
    check_left_and_right();
    turn();
  }   
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
  set_speed(speed);
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
// Stops car
void stop() 
{
  backRight->run(RELEASE);
  frontRight->run(RELEASE);
  frontLeft->run(RELEASE);
  backLeft->run(RELEASE);
}

// Function that gives distance of the ultrasonic sensor
float get_distance()
{
  // Create a "long" integer variable to accommodate a large number of microseconds:
  unsigned long duration = 0; 

  // Create a floating-point variable for the distance:
  float float_distance = 0.0;

  // Read the ultrasonic sensor and return the sound wave travel time in microseconds.
  // The 'ping_median()' method will quickly send 5 pings and return the median echo time, 
  // thereby discarding values that deviate from the average/median. 
  duration = sonar.ping_median(5);
  
  // Check if the duration is zero. Sometimes the sensor reports zero microseconds, which is 
  // physically impossible. In these (rare) cases, substitute 1824 microseconds for the echo
  // time, corresponding to an obstacle distance of ~31 cm:
  if (duration == 0)  {
    duration = 1824;
  }
    
  float_distance = duration *0.034/2;    // Calculate distance in centimeters
  return float_distance;
}

// Stops the car if the distances straight ahead, 45 degrees left and right are within 30.5 cm
void check_and_stop(float distance) 
{
  if (distance <= 30.5) {
    stop();
  }
}

void check_for_obstacles()
{
  // delay() after myservo.write() stabilizes the motors 
  // Get distance to obstacles straight ahead:
  myservo.write(96.5);  // Set servo motor to point straight ahead 
  delay(175);
  straightDistance = get_distance();
  check_and_stop(straightDistance);

  // Get distance to obstacles 45 degrees to right:
  myservo.write(35);  // Set servo motor to point ~45 degrees to right (increased range)
  delay(175);
  rightCenterDistance = get_distance();
  check_and_stop(rightCenterDistance);

  // Swing servo back straight ahead and again get distance to obstacles:
  myservo.write(96.5);  // Set servo motor to point straight ahead
  delay(175); 
  straightDistance = get_distance();
  check_and_stop(straightDistance);

  // Get distance to obstacles 45 degrees to left:
  myservo.write(145);  // Set servo motor to point ~45 degrees to left (increased range)
  delay(175); 
  leftCenterDistance = get_distance();
  check_and_stop(leftCenterDistance);
  
  // Again, get distance to obstacles straight ahead:
  myservo.write(96.5);  // Set servo motor to point straight ahead
  delay(175);
  straightDistance = get_distance();
  check_and_stop(straightDistance); 
}

// Function that checks 90 degrees left and right
void check_left_and_right()
{
  // delay() after myservo.write() stabilizes the motors 
  myservo.write(0); // Turn ultrasonic sensor 90 degrees to right
  delay(200);      
  rightDistance = get_distance();

  delay(200);
  myservo.write(96.5);
  straightDistance = get_distance();              
  delay(250);    

  myservo.write(180); // Turn ultrasonic sensor 90 degrees to left
  delay(200); 
  leftDistance = get_distance();

  delay(200);
  myservo.write(96.5);   // Turn ultrasonic sensor straight ahead again            
  delay(250);
  straightDistance = get_distance();              
}

// Function that turns the car
void turn()
{ 
  if ((rightDistance > rightCenterDistance + 5) && (rightDistance > leftDistance))
  {
    stop();
    backward(carSpeed);
    delay(400);
    turnRight(turningSpeed);
    delay(625); 
    stop();
  }
  else if ((leftDistance > leftCenterDistance + 5) && (leftDistance > rightDistance))
  {
    stop();
    backward(carSpeed);
    delay(400);
    turnLeft(turningSpeed);
    delay(625);
    stop();
  }        // The two conditions above are supposed to do a wider turn, if there is more room 90 degrees to the left and right compared to the 45 degree angles
           // FOR FUTURE REFERENCE: For the final car, need to recheck and retest these conditions as I couldn't get these conditions to work how I wanted
  else if (rightCenterDistance > leftCenterDistance)
  {
    stop();
    delay(200);
    turnRight(turningSpeed);
    delay(450);
    stop();
  }
  else if (leftCenterDistance > rightCenterDistance)
  {
    stop();
    delay(200);
    turnLeft(turningSpeed);
    delay(450);
    stop();
  }        // The conditions above allow the car to do smaller turns after comparing the 45 degree angles
}
