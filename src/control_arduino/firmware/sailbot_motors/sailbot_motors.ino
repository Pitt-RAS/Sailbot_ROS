/* 
 * rosserial subscriber to rudder heading
 * moves rudder to match heading
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
#include <Stepper.h>

class HackyStepper : public Stepper {
  boolean running = false;
  public:
  HackyStepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4)
  : Stepper(number_of_steps, motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4) {
  }
    void openLoop(bool run, bool forward) {
      this->running = run;
      this->direction = forward ? 0 : 1;
    }

    void update() {
      if ( running ) {
        unsigned long now = micros();
        if (now - this->last_step_time >= this->step_delay)
    {
      // get the timeStamp of when you stepped:
      this->last_step_time = now;
      // increment or decrement the step number,
      // depending on direction:
      if (this->direction == 1)
      {
        this->step_number++;
        if (this->step_number == this->number_of_steps) {
          this->step_number = 0;
        }
      }
      else
      {
        if (this->step_number == 0) {
          this->step_number = this->number_of_steps;
        }
        this->step_number--;
      }
      // step the motor to step number 0, 1, ..., {3 or 10}
      if (this->pin_count == 5)
        stepMotor(this->step_number % 10);
      else
        stepMotor(this->step_number % 4);
    }
      }
    }
};


const int SPR= 200;

Servo myservo;
HackyStepper mystepper(SPR, 8, 9, 10, 11);
int stepperGoal= 0;


ros::NodeHandle  nh;

void rudderCb(const std_msgs::Int32 &rudderHeading){
  myservo.write(rudderHeading.data); //changes rudder heading
}

void sailCb(const std_msgs::Int32 &sailAngle){
  stepperGoal = map(sailAngle.data, 0, 90, 0, 180);
}

ros::Subscriber<std_msgs::Int32> rudderSub("cmd_rudder_angle", &rudderCb );
ros::Subscriber<std_msgs::Int32> sailSub("cmd_sail_angle", &sailCb );

int sailPos = 0;


void encISR() {
  if ( digitalRead(3) )
    sailPos++;
  else
    sailPos--;
}



void setup()
{ 
  mystepper.setSpeed(60);
  myservo.attach(6);
  nh.initNode();
  nh.subscribe(rudderSub);
  nh.subscribe(sailSub);
  attachInterrupt(digitalPinToInterrupt(2), encISR, RISING);
  pinMode(13, OUTPUT);
}

void loop()
{  
  digitalWrite(13, HIGH);
  int error = sailPos - stepperGoal;
  
  if ( error > 3 ){
    mystepper.openLoop(true, false);
  }
  else if ( error < -3 ) {
    mystepper.openLoop(true, true);
  }
  else {
    mystepper.openLoop(false, false); 
  }
  
  mystepper.update();
  digitalWrite(13, LOW);
  
  nh.spinOnce();
}


