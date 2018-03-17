/*
 * Code for the lower Teensy
 * Controls the rudder, sail stepper, and IMU
 */

#include <ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <PWMServo.h>
#include <Stepper.h>
#include <SBUS.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

SBUS r9(Serial1);
uint16_t channels[16];

int rudder_rc;
int sail_rc;
bool manualMode = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

class HackyStepper : public Stepper {
  bool running = false;
  int counter = 0;
  int goal = 0;
  public:
  HackyStepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4)
  : Stepper(number_of_steps, motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4) {
  }

    void setGoal(int goal) {
      this->goal = goal;
    }

    int getPosition() {
      return counter;
    }

    int getGoal() {
      return goal;
    }


    void update() {
      running = counter != goal;
      direction = counter > goal;
      
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
            counter--;
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
            counter++;
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

PWMServo myservo;
HackyStepper mystepper(SPR, 14, 15, 16, 17);

ros::NodeHandle  nh;

void rudderCb(const std_msgs::Int32 &rudderHeading){
  myservo.write(rudderHeading.data); //changes rudder heading
}

void sailCb(const std_msgs::Int32 &sailAngle){
  int stepGoal = sailAngle.data;
  
  if(stepGoal>70)
    stepGoal = 70;
   
  mystepper.setGoal(map(stepGoal, 0, 70, 0, 1500));
}

ros::Subscriber<std_msgs::Int32> rudderSub("cmd_rudder_angle", &rudderCb );
ros::Subscriber<std_msgs::Int32> sailSub("cmd_sail_angle", &sailCb );
sensor_msgs::Imu imu_msg;
ros::Publisher imuPub("/imu", &imu_msg);

int pause = 0;

void setup()
{ 
  mystepper.setSpeed(60);
  myservo.attach(6);
  nh.initNode();
  nh.subscribe(rudderSub);
  nh.subscribe(sailSub);
  nh.advertise(imuPub);
  
  pinMode(13, OUTPUT);

  //I2C connection is created
  Wire.begin();
  
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  
  imu_msg.header.frame_id = "base_link";
}

void loop()
{ 
  digitalWrite(13, HIGH);

  if(r9.read(&channels[0], NULL, NULL))
  {
    int manual_value = channels[4];
    if(manual_value>1500){
      manualMode = true;
    } 
    else{
      manualMode = false;
    }
    rudder_rc = map(channels[1],172,1808,0,180);
    sail_rc = map(channels[0],172,1811,0,70);
  }
  
  mystepper.update();
  
  if(!manualMode){
    if(pause==100)
    {
      imu::Quaternion imuQuat = bno.getQuat();
      imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu::Vector<3> angularVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
      imu_msg.header.stamp = nh.now();
    
      imu_msg.orientation.x = imuQuat.x();
      imu_msg.orientation.y = imuQuat.y();
      imu_msg.orientation.z = imuQuat.z();
      imu_msg.orientation.w = imuQuat.w();
    
      imu_msg.angular_velocity.x = angularVel.x();
      imu_msg.angular_velocity.y = angularVel.y();
      imu_msg.angular_velocity.z = angularVel.z();
    
      imu_msg.linear_acceleration.x = linearAccel.x();
      imu_msg.linear_acceleration.y = linearAccel.y();
      imu_msg.linear_acceleration.z = linearAccel.z();
    
      imuPub.publish(&imu_msg);


      //char debug[128];
      //sprintf(debug, "at %d, target is %d", mystepper.getPosition(), mystepper.getGoal());
      //nh.logwarn(debug);

      pause= 0;
    }
    pause++;
  }
  else{ //manual mode
    myservo.write(rudder_rc); //changes rudder heading
    mystepper.setGoal(map(sail_rc, 0, 70, 0, 1500));
  }
  
  digitalWrite(13, LOW);
  nh.spinOnce();

  delay(1);
}

