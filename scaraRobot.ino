#include <AccelStepper.h>
#include <Servo.h>


#define L1 0.106 //length of axis
#define L2 0.093
#define acceleration 150

// (Type:driver, STEP, DIR)
AccelStepper stepper1(1, 4, 7);// theta 1 stepper motor.
AccelStepper stepper2(1, 2, 5); //theta 2 stepper motor
AccelStepper stepper3(1, 3, 6); //Zaxis stepper motor


Servo gripperServo;

//test variables...
//maximum axis length is 320mm = 32cm
float Y1 = 0 ;
float X1 = -0.1 ;
float desY = 0.1;
float desX = 0 ;


int referenceZaxisPosition = 0;


int angles[3];

void setup() {
  Serial.begin(9600);

  //  pinMode(limitSwitch1, INPUT_PULLUP);
  //  pinMode(limitSwitch2, INPUT_PULLUP);
  //  pinMode(limitSwitch3, INPUT_PULLUP);
  //  pinMode(limitSwitch4, INPUT_PULLUP);

  // Setting macimum speed and accelration for the stepper motors
  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(acceleration);
  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(acceleration);
  stepper3.setMaxSpeed(500);
  stepper3.setAcceleration(acceleration);

  gripperServo.attach(A0, 600, 2500);
//  homing();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int SteppsforMotor1 = 0;
  int SteppsforMotor2 = 0;
  int SetupTiming = 0;
//
//  if(Serial.available() > 0){
//    int command = Serial.parseInt();
//    if (command == 1){
      //getting the object position to retrieve the object..
      inverseKinematics(X1, Y1);
      Serial.println(angles[1]);
      Serial.println(angles[2]);
      SteppsforMotor1 = angles[1]/1.8 ;
      Serial.println(SteppsforMotor1);
      SteppsforMotor2 = angles[2]/1.8;
      Serial.println(SteppsforMotor2);
      
      if(referenceZaxisPosition != 0){
        stepper3.setCurrentPosition(-referenceZaxisPosition);
        referenceZaxisPosition = 0;
      }
            
      for(int pos = 0; pos <=100; pos +=1){
        gripperServo.write(pos);
        delay(15);
      }
      
      delay(500);
      stepper1.moveTo(SteppsforMotor1);
      delay(500);
      stepper2.moveTo(SteppsforMotor2);
      delay(1000);

      while (stepper1.currentPosition() != SteppsforMotor1 || stepper2.currentPosition() != SteppsforMotor1){
        stepper1.run();
        stepper2.run();
      }
      
      for(int pos = 100; pos >= 0; pos -=1){
        gripperServo.write(pos);
        delay(15);
//      SetupTiming += 0.015;
      }
      delay(1000);
      
//      homingStepper2(SteppsforMotor2);
//      homingStepper1(SteppsforMotor1);
      //destination setup code...
      inverseKinematics(desX, desY);
      SteppsforMotor1 = 10* angles[1]/1.8;
      SteppsforMotor2 = 10 * angles[2]/1.8;

      stepper3.moveTo(10000);
      referenceZaxisPosition = 10000;

      delay(500);
  
      stepper1.moveTo(SteppsforMotor1);
      delay(500);
      stepper2.moveTo(SteppsforMotor2);
      delay(500);

      while (stepper1.currentPosition() != SteppsforMotor1 || stepper2.currentPosition() != SteppsforMotor1 || stepper3.currentPosition() != referenceZaxisPosition){
        stepper1.run();
        stepper2.run();
        stepper3.run();
      }

      for(int pos = 0; pos <=100; pos +=1){
        gripperServo.write(pos);
        delay(15);
      }
      stepper1.moveTo(-35);
      stepper2.moveTo(-10);
      
      delay(1000);
      for(int pos = 100; pos <= 40; pos -= 1){
        gripperServo.write(pos);
        delay(15);   
      }
      
//    }
//  }
}



//Return to initial position.
void homingStepper3(int referenceZaxisPosition){
  //Homing Stepper3
  stepper3.setSpeed(1500);
  stepper3.runSpeed();
  stepper3.moveTo(-referenceZaxisPosition);
  stepper3.run();
}
void homingStepper2(int SteppsforMotor2){
  stepper2.setSpeed(-1300);
  stepper2.runSpeed();
  stepper2.moveTo(-SteppsforMotor2);
  stepper2.run();
}
void homingStepper1(int SteppsforMotor1){
  stepper1.setSpeed(-1200);
  stepper1.runSpeed();
  stepper1.moveTo(-SteppsforMotor1);
  stepper1.run();
}

//void homing() {
//  // Homing Stepper3
//    stepper3.setSpeed(1500);
//    stepper3.runSpeed();
//    stepper3.setCurrentPosition(17000);
//    // When limit switch pressed set position to 0 steps
//  delay(20);
//  stepper3.moveTo(10000);
//  while (stepper3.currentPosition() != 10000) {
//    stepper3.run();
//  }
//  
//  // Homing Stepper2
//  
//    stepper2.setSpeed(-1300);
//    stepper2.runSpeed();
//    stepper2.setCurrentPosition(-5420); // When limit switch pressed set position to -5440 steps
//
//  delay(20);
//
//  stepper2.moveTo(0);
//  while (stepper2.currentPosition() != 0) {
//    stepper2.run();
//  }
//
//  // Homing Stepper1
// 
//    stepper1.setSpeed(-1200);
//    stepper1.runSpeed();
//    stepper1.setCurrentPosition(-3955); // When limit switch pressed set position to 0 steps
//
//  delay(20);
//  stepper1.moveTo(0);
//  while (stepper1.currentPosition() != 0) {
//    stepper1.run();
//  }
//}
//

// INVERSE KINEMATICS
void inverseKinematics(float x, float y) {
  double phi = 0.00;
  double theta2 = acos((sq(x) + sq(y) - sq(L1) - sq(L2)) / (2 * L1 * L2));
  if (x < 0 & y < 0) {
    theta2 = (-1) * theta2;
  }
  
  double theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));
  
  theta2 = (-1) * theta2 * 180 / PI;
  theta1 = theta1 * 180 / PI;

 // Angles adjustment depending in which quadrant the final tool coordinate x,y is
  if (x >= 0 & y >= 0) {       // 1st quadrant
    theta1 = 90 - theta1;
  }
  if (x < 0 & y > 0) {       // 2nd quadrant
    theta1 = 90 - theta1;
  }
  if (x < 0 & y < 0) {       // 3d quadrant
    theta1 = 270 - theta1;
    phi = 270 - theta1 - theta2;
    phi = (-1) * phi;
  }
  if (x > 0 & y < 0) {       // 4th quadrant
    theta1 = -90 - theta1;
  }
  if (x < 0 & y == 0) {
    theta1 = 270 + theta1;
  }
  
  // Calculate "phi" angle so gripper is parallel to the X axis
  phi = 90 + theta1 + theta2;
  phi = (-1) * phi;

  // Angle adjustment depending in which quadrant the final tool coordinate x,y is
  if (x < 0 & y < 0) {       // 3d quadrant
    phi = 270 - theta1 - theta2;
  }
  if (abs(phi) > 165) {
    phi = 180 + phi;
  }

  angles[0] =round(phi);
  angles[1] = round(theta1);
  angles[2] = round(theta2);
}
