/*
 * Code for the lower Teensy
 * Controls the rudder, sail stepper, and IMU
 */

#include <ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <PWMServo.h>
#include <Stepper.h>
#include <MPU9250.h>

MPU9250 mpuIMU;

class HackyStepper : public Stepper {
  bool running = false;
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

PWMServo myservo;
HackyStepper mystepper(SPR, 14, 15, 16, 17);
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
sensor_msgs::Imu imuMsg;
sensor_msgs::MagneticField magMsg;
ros::Publisher imuMsgPub("imu/data_raw", &imuMsg);
ros::Publisher magMsgPub("imu/mag", &magMsg);

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
  nh.advertise(imuMsgPub);
  nh.advertise(magMsgPub);
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(2, encISR, RISING);
  pinMode(13, OUTPUT);

  //I2C connection is created
  Wire.begin();
  
  //Has to be 0x73 because this is an MPU9255.
  if(mpuIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) == 0x73)
  {
    //Sets up the device
    mpuIMU.calibrateMPU9250(mpuIMU.gyroBias, mpuIMU.accelBias);
    mpuIMU.initMPU9250();
    mpuIMU.initAK8963(mpuIMU.magCalibration);
  }

  imuMsg.header.frame_id = "base_link";
  magMsg.header.frame_id = "base_link";
}

void loop()
{  
  digitalWrite(13, HIGH);
  int error = sailPos - stepperGoal;
  
  if ( error > 1 ){
    mystepper.openLoop(true, false);
  }
  else if ( error < -1 ) {
    mystepper.openLoop(true, true);
  }
  else {
    mystepper.openLoop(false, false); 
  }
  
  mystepper.update();

  //Every time the data is ready to be read the code gets exacuted
  if (mpuIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    //Calculates the accelerometer data
    mpuIMU.readAccelData(mpuIMU.accelCount);
    mpuIMU.getAres();
    mpuIMU.ax = (float)mpuIMU.accelCount[0]*mpuIMU.aRes - mpuIMU.accelBias[0];
    mpuIMU.ay = (float)mpuIMU.accelCount[1]*mpuIMU.aRes - mpuIMU.accelBias[1];
    mpuIMU.az = (float)mpuIMU.accelCount[2]*mpuIMU.aRes - mpuIMU.accelBias[2];
    
    imuMsg.linear_acceleration.x = mpuIMU.ax * 9.8;
    imuMsg.linear_acceleration.y = mpuIMU.ay * 9.8;
    imuMsg.linear_acceleration.z = mpuIMU.az * 9.8;

    mpuIMU.readGyroData(mpuIMU.gyroCount);
    mpuIMU.getGres();
    mpuIMU.gx = (float)mpuIMU.gyroCount[0]*mpuIMU.gRes - mpuIMU.gyroBias[0];
    mpuIMU.gy = (float)mpuIMU.gyroCount[1]*mpuIMU.gRes - mpuIMU.gyroBias[0];
    mpuIMU.gz = (float)mpuIMU.gyroCount[2]*mpuIMU.gRes - mpuIMU.gyroBias[0];

    imuMsg.angular_velocity.x = mpuIMU.gx * (M_PI/180.0);
    imuMsg.angular_velocity.y = mpuIMU.gy * (M_PI/180.0);
    imuMsg.angular_velocity.z = mpuIMU.gz * (M_PI/180.0);

    mpuIMU.readMagData(mpuIMU.magCount);
    mpuIMU.getMres();
    magMsg.magnetic_field.x = (float)mpuIMU.magCount[0]*mpuIMU.mRes*mpuIMU.magCalibration[0];
    magMsg.magnetic_field.y = (float)mpuIMU.magCount[1]*mpuIMU.mRes*mpuIMU.magCalibration[1];
    magMsg.magnetic_field.z = (float)mpuIMU.magCount[2]*mpuIMU.mRes*mpuIMU.magCalibration[2];

    imuMsg.header.stamp = nh.now();
    magMsg.header.stamp = nh.now();
    imuMsgPub.publish(&imuMsg);
    magMsgPub.publish(&magMsg);
  }

  digitalWrite(13, LOW);
  nh.spinOnce();
}



