/**
  Analog出力ジャイロ
  2021-12-27 by ohguma

  ▼パーツ
  DAC          MCP4726 秋月
  ジャイロセンサ MPU6050 Amazonなど

  ▼ピン設定
  入力
  　基準向きリセットSW   A3
  出力
  　向き表示LED1,2,3    D11,D12,D13

  ▼参考
  センサーの使い方(ジャイロ編)
  http://blog.livedoor.jp/revolution_include/archives/2815979.html
*/

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

#include <EEPROM.h>
#include <Wire.h>
#include <I2Cdev.h>
//MP6050の重複をさける
#define I2CDEVLIB_MPU6050_TYPEDEF
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>

//DACアドレス
#define MCP4726_ADDR 0x60

//PIN設定
#define LED_RIGHT 13
#define LED_CENTER 12
#define LED_LEFT 11
#define PIN_SW A3

//方向リセットを始めるスイッチ長押し時間[ms]
#define MS_RESET 50
//キャリブレーションを始めるスイッチ長押し時間[ms]
#define MS_CALIB 3000

//キャリブレーション時のオフセット値を保存するEEPROMアドレス
#define POS_GYRO_X  0x00
#define POS_GYRO_Y  0x02
#define POS_GYRO_Z  0x04
#define POS_ACCEL_Z 0x06

//長押し用タイマー変数。押された瞬間のmillis()を保存する
unsigned long tm_reset = 0;
unsigned long tm_calib = 0;

//DAC出力するローカル角度
int16_t mydeg = 0;
//ジャイロセンサ角度とローカル角度のオフセット
int16_t myinitdeg = 0;

//ジャイロセンサ角度計算用
MPU6050_6Axis_MotionApps20 mpu;
int16_t gyro = 0;             //ジャイロセンサ角度
static uint8_t mpuIntStatus;
static bool dmpReady = false; // set true if DMP init was successful
static uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;
uint8_t fifoBuffer[64]; // FIFO storage buffer                 // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup()
{
  Serial.begin(9600);
  DEBUG_PRINTLN("Analog Gyro 2021-12");
  DEBUG_PRINTLN("Initialize...");
  //ピン設定
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_CENTER, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(PIN_SW, INPUT_PULLUP);
  //出力初期化
  digitalWrite(LED_RIGHT, LOW);
  digitalWrite(LED_CENTER, LOW);
  digitalWrite(LED_LEFT, LOW);
  //ジャイロセンサ初期化
  init_gyro();
  attach_offset();
  DEBUG_PRINTLN("Start...");
}

void loop()
{
  if (digitalRead(PIN_SW) == HIGH) {
    //キーが押されていないデフォルト状態
    tm_reset = 0;
    tm_calib = 0;
    //ジャイロ値更新
    set_gyro_value();
    //ローカル角度に変換（-180< x <= 180）
    mydeg = gyro - myinitdeg;
    if (mydeg < -180) {
      mydeg += 360;
    } else if (mydeg > 180) {
      mydeg -= 360;
    }
    DEBUG_PRINTLN(mydeg);
    //DAC出力 -180～180のデータを0～4095(2^12-1)のデータに換算する
    int val = constrain(map(mydeg, -180, 180, 4095, 0), 0, 4095);
    Wire.beginTransmission(MCP4726_ADDR);
    Wire.write((uint8_t) ((val >> 8) & 0x0F));   // MSB: (D11, D10, D9, D8)
    Wire.write((uint8_t) (val));  // LSB: (D7, D6, D5, D4, D3, D2, D1, D0)
    Wire.endTransmission();
    //状態LED出力
    if (mydeg >= -5 && mydeg < -1 ) {
      digitalWrite(LED_LEFT, HIGH);
    } else {
      digitalWrite(LED_LEFT, LOW);
    }
    if (mydeg >= -1 && mydeg <= 1 ) {
      digitalWrite(LED_CENTER, HIGH);
    } else {
      digitalWrite(LED_CENTER, LOW);
    }
    if (mydeg > 1 && mydeg <= 5 ) {
      digitalWrite(LED_RIGHT, HIGH);
    } else {
      digitalWrite(LED_RIGHT, LOW);
    }
  } else if (tm_reset == 0 && tm_calib == 0) {
    //スイッチ押し時間計測開始
    tm_reset = millis();
    tm_calib = millis();
  } else if (tm_reset > 0 && millis() - tm_reset > MS_RESET) {
    //ローカル角度のリセット（現時点の向きを基準向きとする）
    tm_reset = 0;
    attach_offset();

  } else if (tm_calib > 0 && millis() - tm_calib > MS_CALIB) {
    //キャリブレーション開始
    tm_calib = 0;
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_CENTER, HIGH);
    digitalWrite(LED_LEFT, HIGH);
    action_calibration();
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_CENTER, LOW);
    digitalWrite(LED_LEFT, LOW);
    init_gyro();
    attach_offset();
  }
}

//ローカル角度のリセット用のオフセット更新
void attach_offset()
{
  myinitdeg = gyro;
}

//ジャイロセンサ値の更新
void set_gyro_value()
{
  mpuIntStatus = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    gyro = degrees(ypr[0]) + 180;
    if (gyro < 0) {
      gyro += 360;
    }
    if (gyro > 359) {
      gyro -= 360;
    }
  }
}

//ジャイロセンサ初期化
void init_gyro()
{
  int val;
  mpu.initialize();
  while (!mpu.testConnection()) {
  }
  if (mpu.testConnection() != true) {
    DEBUG_PRINTLN("MPU disconection");
    while (true) {}
  }
  if (mpu.dmpInitialize() != 0) {
    DEBUG_PRINTLN("MPU break");
    while (true) {}
  }
  EEPROM.get(POS_GYRO_X, val);
  mpu.setXGyroOffset(val);
  EEPROM.get(POS_GYRO_Y, val);
  mpu.setYGyroOffset(val);
  EEPROM.get(POS_GYRO_Z, val);
  mpu.setZGyroOffset(val);
  EEPROM.get(POS_ACCEL_Z, val);
  mpu.setZAccelOffset(val);
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  //少し待つ
  for (int i = 0; i < 100; i++) {
    set_gyro_value();
  }
}


// 以下は長押し時のキャリブレーション用
MPU6050_Base accelgyro;
int state = 0;
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

//キャリブレーション処理
void action_calibration()
{
  DEBUG_PRINTLN("\nMPU6050 Calibration");
  //delay(2000);
  DEBUG_PRINTLN("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  //delay(3000);
  // verify connection
  DEBUG_PRINTLN(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.
  accelgyro.initialize();
  while (1) {
    if (state == 0) {
      DEBUG_PRINTLN("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(1000);
    }
    if (state == 1) {
      DEBUG_PRINTLN("\nCalculating offsets...");
      calibration();
      state++;
      delay(1000);
    }
    if (state == 2) {
      meansensors();
      DEBUG_PRINTLN("\nFINISHED!");
      DEBUG_PRINT("\nSensor readings with offsets:\t");
      DEBUG_PRINT(mean_ax);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_ay);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_az);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_gx);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_gy);
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(mean_gz);
      DEBUG_PRINT("Your offsets:\t");
      DEBUG_PRINT(ax_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(ay_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(az_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(gx_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(gy_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(gz_offset);
      DEBUG_PRINTLN("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      EEPROM.put(POS_GYRO_X, gx_offset);
      EEPROM.put(POS_GYRO_Y, gy_offset);
      EEPROM.put(POS_GYRO_Z, gz_offset);
      EEPROM.put(POS_ACCEL_Z, az_offset);
      return;
    }
  }
}

void meansensors()
{
  long i = 0;
  long buff_ax = 0, buff_ay = 0, buff_az = 0;
  long buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
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
  }
}

void calibration()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
    meansensors();
    DEBUG_PRINTLN("...");
    if (abs(mean_ax) <= acel_deadzone) {
      ready++;
    } else {
      ax_offset = ax_offset - mean_ax / acel_deadzone;
    }
    if (abs(mean_ay) <= acel_deadzone) {
      ready++;
    } else {
      ay_offset = ay_offset - mean_ay / acel_deadzone;
    }
    if (abs(16384 - mean_az) <= acel_deadzone) {
      ready++;
    } else {
      az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    }
    if (abs(mean_gx) <= giro_deadzone) {
      ready++;
    } else {
      gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
    }
    if (abs(mean_gy) <= giro_deadzone) {
      ready++;
    } else {
      gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
    }
    if (abs(mean_gz) <= giro_deadzone) {
      ready++;
    } else {
      gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    }
    if (ready == 6) break;
  }
}
