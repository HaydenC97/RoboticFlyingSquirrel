//Setup ultrasonic
int trigPIN = 4;
int echoPIN = 5;
float duration, distance;

//Setup servo
int servoPIN = 9;
#include <Servo.h>
Servo myservo;

//Setup LED
const int led = LED_BUILTIN;

//Setup tail
int pos_down = 0;
int pos_up = 90 ;
int pos = 0;
int tail_pos = 0; //0 down, 1 up

int thresh = 50; //within __cm triggers tail movement

//setup IMU
#include <MPU9250.h>
MPU9250 IMU(Wire, 0x68);
int status;
float AcX, AcY, AcZ, gy;
float pitch, roll;

//setup PID
float i; //integral
int vortex = 40; //optimum vortex angle
int glide = 10; //optimum glide angle
int target = glide;     //glide or vortex

//setup time
float tic, toc, dt;

void getAngle(float Vx, float Vy, float Vz) {
  float x = Vx;
  float y = Vy;
  float z = Vz;
  pitch = atan(x / sqrt((y * y) + (z * z)));
  //pitch = pitch*2 +45;
  roll = atan(y / sqrt((x * x) + (z * z)));
  //convert radians into degrees
  pitch = pitch * (180.0 / 3.14159);
  pitch = pitch*2 + 90;
  roll = roll * (180.0 / 3.14159) ;
}

void calibrateIMU(){
  status = IMU.begin();
  Serial.println("Calibrating...");
  status = IMU.calibrateAccel();
  float axb = IMU.getAccelBiasX_mss();
  float axs = IMU.getAccelScaleFactorX();
  float ayb = IMU.getAccelBiasY_mss();
  float ays = IMU.getAccelScaleFactorY();
  float azb = IMU.getAccelBiasZ_mss();
  float azs = IMU.getAccelScaleFactorZ();
  IMU.setAccelCalX(axb,axs);
  IMU.setAccelCalY(ayb,ays);
  IMU.setAccelCalZ(azb,azs);
  Serial.println("Calibration complete.");
}

float PIDcalc(float pitch, float pitch_dot, float dt)
{
  float p = target - pitch;
  float d = pitch_dot;
  i = i + p * dt;

  float kp = 2;
  float kd = 1;
  float ki = 0.01;

  pos = kp * p + kd * d + ki * i; //add pitch so it's relative to the ground instead of the glider?
  if (pos > (pos_up)) {
    pos = pos_up;
  }
  else if (pos < (pos_down)) {
    pos = pos_down;
  }
  return pos;
}

void setup() {
  Wire.begin();
  Serial.begin (9600);
  while (!Serial) {}
  
  pinMode(trigPIN, OUTPUT);
  pinMode(echoPIN, INPUT);

  pinMode(led, OUTPUT);

  myservo.attach(servoPIN);
  pinMode(led, OUTPUT);

  myservo.write(glide);

  //calibrate accelerometer
  calibrateIMU();
//  status = IMU.begin();
//  status = IMU.calibrateAccel();
//  float axb = IMU.getAccelBiasX_mss();
//  float axs = IMU.getAccelScaleFactorX();
//  float ayb = IMU.getAccelBiasY_mss();
//  float ays = IMU.getAccelScaleFactorY();
//  float azb = IMU.getAccelBiasZ_mss();
//  float azs = IMU.getAccelScaleFactorZ();
//  IMU.setAccelCalX(axb,axs);
//  IMU.setAccelCalY(ayb,ays);
//  IMU.setAccelCalZ(azb,azs);

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {
      digitalWrite(led, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      delay(100);
    }
  }

  Serial.print("Tail_Angle");
  Serial.print("\t");
  Serial.print("Pitch");
  Serial.print("\t");
  Serial.print("Target_Angle");
  Serial.println("");
}

void loop() {
  tic = millis();

  // Send pulse
  digitalWrite(trigPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPIN, LOW);

  // Receive pulse
  duration = pulseIn(echoPIN, HIGH, 24000L);
//  distance = ((duration/2) / 29.1)*cos(pitch/57.298);
  distance = 40;

  // Measure distance and move tail
  if ((distance > 5) && (distance < thresh)) {
    digitalWrite(led, HIGH);
    target = vortex;
  }
  else if ((distance < 5) || (distance > thresh)) {
    digitalWrite(led, LOW);
    target = glide;
    i = 0; //reset integral
  }
  IMU.readSensor();
  gy = IMU.getGyroY_rads() * 180/3.14159;
  AcX = IMU.getAccelX_mss();
  AcY = IMU.getAccelY_mss();
  AcZ = IMU.getAccelZ_mss();
  //get pitch/roll
  getAngle(AcX, AcY, AcZ);
  pos = PIDcalc(pitch, gy, millis() - tic);
  tic = millis();
  
  myservo.write(pos);

  Serial.print(pos);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(target);
  Serial.println("");

//  Serial.print(AcX);
//  Serial.print("\t");
//  Serial.print(AcY);
//  Serial.print("\t");
//  Serial.print(AcZ);
//  Serial.println("");

  delay(100);
}
