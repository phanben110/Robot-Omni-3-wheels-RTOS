void mpu6050_begin()  {
  Wire.begin();
  Serial.print("MPU6050: Starting calibration; leave device flat and still ... ");
  int error = mpu6050.begin();
  Serial.println(mpu6050.error_str(error));
}
//doc gia tri truc dz
float mpu6050_yaw() {
  MPU6050_t data = mpu6050.get();
  while ( data.dir.error != 0 ) {
    // I suffer from a lot of I2C problems
    Serial.println(mpu6050.error_str(data.dir.error));
    // Reset I2C
    TWCR = 0; Wire.begin();
    // Reread
    data = mpu6050.get();
  }
  return data.dir.yaw;
}
