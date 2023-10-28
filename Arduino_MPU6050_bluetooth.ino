#include <Wire.h>
#include <MPU6050.h>
#define button_reset 52
#define SENT_DATA_SIZE 22
#define RECIVE_DATA_SIZE 22
#define MAX_RPM_RAD 890
typedef struct{
  float t ;
  float omga ;
  float INN;
  float desired;
  float ANGLE ;
  int pwmVal ;
 
}A ;

union{
  A x;
  byte y[SENT_DATA_SIZE];
 
 
  }dataSent;


// flags des gains
typedef struct{
  byte flag ;
  byte control_flag ;
  float des_omega;
  float Angle_dg ;
  float kp ;
  float ki ;
  float kd ;  
   
 
}B;
union {
  B n;
  byte M[RECIVE_DATA_SIZE];
  }dataRead;
 
MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;


  //
  float kp=2;
  float ki=0.0 ;
  float kd=0 ;  
  float dd0=0 ;
  float di=0 ;
  float dt=0.05;
  float des_omega=0;  
  float Angle_dg = 90 ;
  float angle_des=Angle_dg*3.14/(float)180;
  float current_angle=0;
  float error_angle =0;
  float dif_error =0;
  float inn =0;
  float int_error=0;
  float innn ;
  float execTime=0;
  int velocity ;
  //
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
 
 
void setup()
{
  Serial.begin(115200);
  Serial.println("Start...");
  Serial3.begin(115200) ;
 
 while(!Serial3.available()){
  delay(10000);
  //stopCode =1;
 
  //Serial.println("reset");
  //pinMode(button_reset,HIGH);  
  };
 
  Serial.print("Connection Established..");delay(2000);
  // Initialize MPU6050
 while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
 
  // Calibrate gyroscope. The   calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);

  //////////////
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
}
byte flag =0 ;
byte control_flag =0 ;
float time_old = 0;
int stopCode = 0 ;


// gyro_function
void Gyro_an()
{
   timer = millis();

  //if ((timer-time_old)>10000){
    //Serial.println("Reset...");
    //stopCode = 1;
  //}
 
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep*1;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
 
dataSent.x.t = timer/((float)1000);
dataSent.x.omga = norm.YAxis;
dataSent.x.ANGLE= pitch;
Serial.println(" ");
Serial.println("");
Serial.println("");
  wrap (&roll) ;
  wrap (&pitch) ;
  wrap (&yaw) ;
  // Output raw
 /* Serial.print(" Pitch = ");
  Serial.print(norm.YAxis);
  Serial.print(" Roll = ");
  Serial.print(norm.XAxis);  
  Serial.print(" Yaw = ");
  Serial.println(norm.ZAxis);
*/
//Serial.println(flag) ;

  // Wait to full timeStep period
  //delay((timeStep*1000) - (millis() - timer));

   return pitch;
    }
  void move (){

  innn = abs(inn);
velocity = map(innn, 0, MAX_RPM_RAD, 0, 1023);
//velocity = 1023;
 if (inn> 0)
{
 analogWrite(RPWM_Output, velocity);
 analogWrite(LPWM_Output, 0);
 //dataSent.x.motorDirection = 1;
}
 else
 { analogWrite(RPWM_Output, 0);
  analogWrite(LPWM_Output, velocity);
  //dataSent.x.motorDirection = -1;
 
  }
 dataSent.x.pwmVal = velocity ;
/*
   for (int idx =0 ;idx<SENT_DATA_SIZE ;idx++){
    Serial.print(dataSent.y[idx],HEX);Serial.print(" ");
  }
  Serial.println() ;
Serial.print(dataSent.x.pwmVal);Serial.print(" ");Serial.println(dataSent.x.motorDirection);
  */
}
void wrap (float *angle)
{
  if (*angle>180)
  {
    *angle = *angle - 360 ;
  }
  else if(*angle<-180){
    *angle = *angle + 360 ;
  }
}

  void(* resetFunc) (void) = 0; //declare reset function @ address 0
  float sign(float inputValue){
  return inputValue/abs(inputValue) ;
}

void Controll_angle()
{
    if (stopCode==1){
    Serial.println("Exiting Code....");
    inn = 0;
    move() ;
    delay(5000) ;
    resetFunc();  //call reset
    }
  Gyro_an();
 current_angle=(pitch*3.14)/(float)180; //return the value from gyro function and assgin it to Calculated_Angle
 error_angle = angle_des - current_angle ;
 wrap(&error_angle) ;
 
// Serial.println(error_angle*180/(float)3.14);
 dif_error = (error_angle - dd0)/dt ;
 di = di+(((error_angle+dd0)*dt)/2);
 inn = (kp*error_angle) + (ki*int_error) + (kd*dif_error);
  if (abs(inn) > MAX_RPM_RAD) {
    inn = MAX_RPM_RAD * sign(inn);
  }
  else{
    inn=inn;
  }
  dataSent.x.INN=inn;
  dataSent.x.desired=Angle_dg;
 
  move();
if (flag){
//Serial.println(dataSent.x.pwmVal);Serial.println(dataSent.x.motorDirection);
//  for (int idx =0 ;idx<SENT_DATA_SIZE ;idx++){
//    Serial.print(dataSent.y[idx],HEX);Serial.print(" ");
//  }
//  Serial.println() ;
//Serial.print(dataSent.x.pwmVal);Serial.print(" ");Serial.println(dataSent.x.motorDirection);
//  
 //Serial.println("Sending");
 Serial3.write(dataSent.y,SENT_DATA_SIZE) ;
 flag = 0;
}
  dd0 = error_angle ;
  execTime = millis() - timer ;
  delay (30); float
  dt = execTime/(float)1000 ;
  timeStep = dt;
  if (dt*1000>execTime)
    delay(dt*1000-execTime);  
  }

void Controll_angular_velocity()
{
 

  /// need to controoooooooooooooooooooooooooooooool

 Serial.print("control angle ");
  }

void loop()
{
    if(Serial3.available())
  {
    delay(50) ;
    if (!flag){
     
      for (int j = 0 ;j<RECIVE_DATA_SIZE;j++){
        dataRead.M[j] = Serial3.read() ;
       // Serial.println("Well Received");
       // Serial.print("Flag:  ");
        //Serial.println(dataRead.n.flag);
      }

      flag=dataRead.n.flag;
      control_flag=dataRead.n.control_flag;
      des_omega=dataRead.n.des_omega ;
      Angle_dg = dataRead.n.Angle_dg  ;
      kp = dataRead.n.kp ;
      ki = dataRead.n.ki ;
      kd = dataRead.n.kd  ;
      time_old = timer ;
      //  Serial.print(velocity);
      /*
      Serial.println(flag);
      Serial.println(control_flag);
      Serial.println(Angle_dg);
      Serial.println(kp);
      Serial.println(ki);
      Serial.println(kd);
    */
    }
  }

  if (!control_flag) {
    Controll_angle();
    //Serial.println(velocity);
    }
   else
   {
    Controll_angular_velocity();
    //Serial.println("    ");
    }
}
