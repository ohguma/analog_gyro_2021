/**
  M5Atom Matrix Analog出力ジャイロ

  Grove G26 DAC でジャイロセンサによる方位をアナログ出力
  
  参考
  Arduinoで信号処理をする上で便利なフィルタのライブラリ
  https://ehbtj.com/electronics/filter-library-useful-for-arduino-signal-processing/#Madgwick

*/

#include "M5Atom.h"
#include <MadgwickAHRS.h>   // 角度と角速度にMadgwickフィルタをかけるライブラリ
Madgwick MadgwickFilter;

#include "EEPROM.h"
//キャリブレーション時のオフセット値を保存するEEPROMアドレス
#define POS_ACCEL_X 0x00
#define POS_ACCEL_Y 0x04
#define POS_ACCEL_Z 0x08
#define POS_GYRO_X  0x0C
#define POS_GYRO_Y  0x10
#define POS_GYRO_Z  0x14

//LEDの方向表示
// 0  1  2  3  4
// 5  6  7  8  9
// 10 11 12 13 14
// 15 16 17 18 19
// 20 21 22 23 24 ↓コネクタ側
#define NUM_CLOCK 12
uint8_t clockwise[NUM_CLOCK] = {2, 3, 9, 14, 19, 23, 22, 21, 15, 10, 5, 1};
uint8_t pos_time = 0;
uint8_t pos_time_before = 13;

#define PIN_DAC 26

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0 , gyroY = 0 , gyroZ = 0;
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;
float gyroX_offset = 0 , gyroY_offset = 0 , gyroZ_offset = 0;
float yaw_offset  = 0.0F;
float yaw_current = 0.0F;

void setup() {
  //M5Stackシリーズ M5.begin()の互換性
  //https://qiita.com/penguinprogrammer/items/5abf3e4f9583e41e9cfa
  //M5Atom : SerialEnable, I2CEnable, DisplayEnable
  M5.begin(true, true, true);
  M5.IMU.Init();
  M5.IMU.SetGyroFsr(M5.IMU.GFS_250DPS); //250,,500,1000,2000
  M5.IMU.SetAccelFsr(M5.IMU.AFS_2G); //2,4,8,16

  if (!EEPROM.begin(24)) {  //②
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  accX_offset = EEPROM.readFloat(POS_ACCEL_X);
  accY_offset = EEPROM.readFloat(POS_ACCEL_Y);
  accZ_offset = EEPROM.readFloat(POS_ACCEL_Z);
  gyroX_offset = EEPROM.readFloat(POS_GYRO_X);
  gyroY_offset = EEPROM.readFloat(POS_GYRO_Y);
  gyroZ_offset = EEPROM.readFloat(POS_GYRO_Z);

  MadgwickFilter.begin(100);  // Madgwickフィルタの周波数を100Hzに設定。

  attach_offset();
}

void loop() {
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  MadgwickFilter.updateIMU(gyroX - gyroX_offset, gyroY - gyroY_offset, gyroZ - gyroZ_offset, accX - accX_offset, accY - accY_offset, accZ - accZ_offset);
  pitch  = MadgwickFilter.getPitch();
  roll  = MadgwickFilter.getRoll();
  yaw  = MadgwickFilter.getYaw();
  //  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  if (M5.Btn.wasPressed()) {
    dacWrite(PIN_DAC, 128);
    M5.dis.drawpix(12, 0x0000ff); //BLUE
    delay(1000);
    M5.dis.drawpix(12, 0x000000);
    if (M5.Btn.read()) { //キーが押されたままだったら長押しと判定
      action_calibration();
    }
    attach_offset();
  }
  //データ出力（0 <= yaw <  360　小←　→大)
  //過去のAnalog gyroに合わせて、大←　→小とする。
  yaw_current = -(yaw - yaw_offset);  // -360 < x < 360
  if (yaw_current < -180.0F) {
    yaw_current += 360.0F;
  } else if (yaw_current > 180.0F) {
    yaw_current -= 360.0F;
  }

  int dacout = map((int)yaw_current, -180, 180, 0, 255);
  dacout = constrain(dacout, 0, 255);
  dacWrite(PIN_DAC, dacout);

  //LED表示
  float oneclock = 360.0 / NUM_CLOCK;
  pos_time = (int)((yaw_current + 360.0 + oneclock / 2.0) / oneclock) % NUM_CLOCK;
  if (pos_time_before != pos_time) {
    M5.dis.drawpix(clockwise[pos_time_before], 0x000000);
    M5.dis.drawpix(clockwise[pos_time], 0x400000); //RED
    pos_time_before = pos_time;
  }
  if (yaw_current < -1.0 && yaw_current >= -5.0) {
    M5.dis.drawpix(6, 0x004000); //GREEN
  } else {
    M5.dis.drawpix(6, 0x000000);
  }
  if (abs(yaw_current) <= 1.0) {
    M5.dis.drawpix(7, 0x004000); //GREEN
  } else {
    M5.dis.drawpix(7, 0x000000);
  }
  if (yaw_current > 1.0 && yaw_current <= 5.0) {
    M5.dis.drawpix(8, 0x004000); //GREEN
  } else {
    M5.dis.drawpix(8, 0x000000);
  }

  Serial.printf("%d %5.2f %5.2f ", pos_time, yaw_current, yaw);
  Serial.printf("%5.2f %5.2f %5.2f ", gyroX, gyroY, gyroZ );
  Serial.printf("%5.2f %5.2f %5.2f\n", accX, accY, accZ );
  delay(10);
  M5.update();
}

void attach_offset()
{
  Serial.printf("\n[attach_offset]");
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  MadgwickFilter.updateIMU(gyroX - gyroX_offset, gyroY - gyroY_offset, gyroZ - gyroZ_offset, accX - accX_offset, accY - accY_offset, accZ - accZ_offset);
  yaw_offset  = MadgwickFilter.getYaw();

  if (isnan(yaw_offset)) {
    action_calibration();
  }
}


float mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
float ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int buffersize = 1000;   //平均化時の読み取り回数。大きくすると制度が上がるが、速度低下する。
float acel_deadzone = 0.02;   //加速度センサのエラー許容範囲。下げすぎると収束しないかも。
float giro_deadzone = 0.01;   //ジャイロセンサのエラー許容範囲。下げすぎると収束しないかも。

void action_calibration()
{
  int state = 0;
  Serial.printf("\n[action_calibration]");
  M5.dis.drawpix(12, 0x808000); //YELLOW
  delay(1000);
  //reset offset
  accX_offset = 0.0;
  accY_offset = 0.0;
  accZ_offset = 0.0;
  gyroX_offset = 0.0;
  gyroY_offset = 0.0;
  gyroZ_offset = 0.0;
  while (1) {
    if (state == 0) {
      Serial.printf("\nReading sensors for first time...");
      meansensors();
      state = 1;
      delay(1000);
    }
    if (state == 1) {
      Serial.printf("\nCalculating offsets...");
      calibration();
      state = 2;
      delay(1000);
    }
    if (state == 2) {
      break;
    }
  }
  Serial.printf("\nFINISHED!");
  Serial.printf("\noffsets : ");
  Serial.printf("AX:%5.2f ", ax_offset);
  Serial.printf("AY:%5.2f ", ay_offset);
  Serial.printf("AZ:%5.2f ", az_offset);
  Serial.printf("GX:%5.2f ", gx_offset);
  Serial.printf("GY:%5.2f ", gy_offset);
  Serial.printf("GZ:%5.2f ", gz_offset);
  accX_offset = ax_offset;
  accY_offset = ay_offset;
  accZ_offset = az_offset;
  gyroX_offset = gx_offset;
  gyroY_offset = gy_offset;
  gyroZ_offset = gz_offset;
  EEPROM.writeFloat(POS_ACCEL_X, accX_offset);
  EEPROM.writeFloat(POS_ACCEL_Y, accY_offset);
  EEPROM.writeFloat(POS_ACCEL_Z, accZ_offset);
  EEPROM.writeFloat(POS_GYRO_X, gyroX_offset);
  EEPROM.writeFloat(POS_GYRO_Y, gyroY_offset);
  EEPROM.writeFloat(POS_GYRO_Z, gyroZ_offset);
  EEPROM.commit();
  delay(3000);
  M5.dis.drawpix(12, 0x000000);
}


void meansensors()
{
  Serial.printf("\n[meansensors]");
  long i = 0;
  float buff_ax = 0, buff_ay = 0, buff_az = 0;
  float buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ);
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + accX - accX_offset;
      buff_ay = buff_ay + accY - accY_offset;
      buff_az = buff_az + accZ - accZ_offset ;
      buff_gx = buff_gx + gyroX - gyroX_offset;
      buff_gy = buff_gy + gyroY - gyroY_offset;
      buff_gz = buff_gz + gyroZ - gyroZ_offset;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
    M5.update();
  }
  Serial.printf("\nmeans : ");
  Serial.printf("AX:%5.2f ", mean_ax);
  Serial.printf("AY:%5.2f ", mean_ay);
  Serial.printf("AZ:%5.2f ", mean_az);
  Serial.printf("GX:%5.2f ", mean_gx);
  Serial.printf("GY:%5.2f ", mean_gy);
  Serial.printf("GZ:%5.2f ", mean_gz);
}


void calibration()
{
  Serial.printf("\n[calibration]");
  ax_offset = mean_ax / 8.0;
  ay_offset = mean_ay / 8.0;
  az_offset = (mean_az - 1.0) / 8.0;
  gx_offset = mean_gx / 4.0;
  gy_offset = mean_gy / 4.0;
  gz_offset = mean_gz / 4.0;
  uint8_t pos = 13;
  uint8_t pos_before = 12;
  while (1) {
    int ready = 0;
    accX_offset = ax_offset;
    accY_offset = ay_offset;
    accZ_offset = az_offset;
    gyroX_offset = gx_offset;
    gyroY_offset = gy_offset;
    gyroZ_offset = gz_offset;
    meansensors();
    Serial.printf("\n %d AX", ready);
    if (abs(mean_ax) <= acel_deadzone) {
      ready++;
    } else {
      ax_offset = ax_offset + mean_ax / 8.0;
    }
    Serial.printf("\n %d AY", ready);
    if (abs(mean_ay) <= acel_deadzone) {
      ready++;
    } else {
      ay_offset = ay_offset + mean_ay / 8.0;
    }
    Serial.printf("\n %d AZ", ready);
    if (abs(mean_az - 1.0) <= acel_deadzone) {
      ready++;
    } else {
      az_offset = az_offset + (mean_az - 1.0) / 8.0;
    }
    Serial.printf("\n %d GX", ready);
    if (abs(mean_gx) <= giro_deadzone) {
      ready++;
    } else {
      gx_offset = gx_offset + mean_gx / 4.0;
    }
    Serial.printf("\n %d GY", ready);
    if (abs(mean_gy) <= giro_deadzone) {
      ready++;
    } else {
      gy_offset = gy_offset + mean_gy / 4.0;
    }
    Serial.printf("\n %d GZ", ready);
    if (abs(mean_gz) <= giro_deadzone) {
      ready++;
    } else {
      gz_offset = gz_offset + mean_gz / 4.0;
    }
    if (ready == 6) {
      break;
    }

    //LED表示
    if (pos != pos_before) {
      M5.dis.drawpix(pos_before, 0x000000);
      M5.dis.drawpix(pos, 0x808000); //YELLOW
      pos_before = pos;
    }
    pos++;
    if (pos == 14) {
      pos = 11;
    }
  }
  M5.dis.drawpix(11, 0x000000);
  M5.dis.drawpix(12, 0x000000);
  M5.dis.drawpix(13, 0x000000);
}
