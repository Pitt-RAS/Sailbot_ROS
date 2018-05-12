#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno;

void setup() {
    Serial.begin(9600);
    bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
}
    
void loop() {
    uint8_t system = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println(system);
    Serial.println(gyro);
    Serial.println(accel);
    Serial.println(mag);
    Serial.print("\n");

    if (system == 3 && gyro == 3 && accel == 3 && mag == 3)
    {
        Serial.println("Storing calibration data");
        gotCalibration();
        while(1);
    }
}

void gotCalibration() {
    adafruit_bno055_offsets_t sensor_offsets;
    bno.getSensorOffsets(sensor_offsets);
    Serial.print("Accel: ");
    Serial.print(sensor_offsets.accel_offset_x);
    Serial.print(" ");
    Serial.print(sensor_offsets.accel_offset_y);
    Serial.print(" ");
    Serial.print(sensor_offsets.accel_offset_z);
    Serial.print("\n");

    Serial.print("Mag: ");
    Serial.print(sensor_offsets.mag_offset_x);
    Serial.print(" ");
    Serial.print(sensor_offsets.mag_offset_y);
    Serial.print(" ");
    Serial.print(sensor_offsets.mag_offset_z);
    Serial.print("\n");

    Serial.print("Gyro: ");
    Serial.print(sensor_offsets.gyro_offset_x);
    Serial.print(" ");
    Serial.print(sensor_offsets.gyro_offset_y);
    Serial.print(" ");
    Serial.print(sensor_offsets.gyro_offset_z);
    Serial.print("\n");

    Serial.print("Accel radius:");
    Serial.print(sensor_offsets.accel_radius);
    Serial.print("\nMag radius:");
    Serial.print(sensor_offsets.mag_radius);
}

