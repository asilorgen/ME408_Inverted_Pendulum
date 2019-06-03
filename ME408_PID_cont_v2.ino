#include <PID_v1.h>

#define negPWM 3

int potPin = A1;
double potVal = 0.0;
double angle = 0.0;
double EMA_a = 0.5; 	   //initalization of EMA alpha  // 0.5, 200Hz ->  23Hz cutoff point!
double filt_angle = 0.0;   //initialization of EMA output (low pass)

int left_switch = 7;  //end swtich left
int right_switch = 8; //end swtich right

#define in1 5 //PWM output to L298n
#define in2 6 //PWM output to L298n
#define enA 9 //Pin to enable/disable H bridge

double Kp = 70.0;
double Ki = 250.0;
double Kd = 0.25;

double a_in, a_out, a_setp, pwmin; //PID parameters
PID angPID(&a_in, &a_out, &a_setp, Kp, Ki, Kd, DIRECT); //PID loop for controlling the cart


void setup() {
  pinMode(negPWM, OUTPUT);  //pwm input for charge pump (power supply for op-amps)

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  angPID.SetSampleTime(5);  //200Hz sample time
  angPID.SetMode(AUTOMATIC);
  angPID.SetOutputLimits(-255, 255);

  pinMode(left_switch, INPUT);
  pinMode(right_switch, INPUT);

  Serial.begin(115200);
}
//Rough PWM function for negative voltage generation with charge pump
void negVoltPWM() {
  digitalWrite(negPWM, HIGH);
  delayMicroseconds(500);  //Approx 50% duty cycle @ 1kHz
  digitalWrite(negPWM, LOW);
  delayMicroseconds(1000 - 500);
}
//Low pass filter design, Exponential Moving Average (EMA)
void lowPassFilt() {
  filt_angle = (EMA_a * angle) + ((1 - EMA_a) * filt_angle);
}

void DEBUG_potVal() {
  //Serial.println(potVal);
  Serial.println(filt_angle);
}

String readStr;
int serialCount = 0;
double tuneParam = 0.0;

//Function for tuning PID parameters during the execution with serial communication
void setPIDParams() {
  Serial.println("Enter PID parameters (Kp, Ki, Kd): " );
  while (Serial.available()){
    char c = Serial.read();
    if (c == ','){
        tuneParam = readStr.toDouble();
        readStr ="";
        if (serialCount % 3 == 0){
          Kp = tuneParam;
        }
        else if (serialCount % 3 == 1){
          Ki = tuneParam;
        }
        else if (serialCount % 3 == 2){
          Kd = tuneParam;
        }
        serialCount = serialCount + 1;
    }
    else{
      readStr = readStr + c;
    }
  }
  Serial.print("Received parameters: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.println(Kd);
  angPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  negVoltPWM();
  
  if (digitalRead(left_switch) == true && digitalRead(right_switch) == true){ //if the cart touches to the end-switches, stop the motor!
    potVal = analogRead(potPin);
    angle = potVal * 50 / (1023.0 - 16.0) - 25.79;  //pot value to angle conversion
    lowPassFilt();
    //DEBUG_potVal();

    a_setp = 0.0;
    a_in = filt_angle;  //filt_angle
    setPIDParams();
    angPID.Compute();

    if (a_out < 0) //setting moving direction 
    {
      pwmin = (-1 * a_out) + 70;  //+70 due to static friction
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      pwmin = min(pwmin, 255);    //max pwm value could be 255
      analogWrite(enA, pwmin);	  //PWM for controlling the motor
    }
    else if (a_out >= 0)
    {
      pwmin = (a_out) + 70;	  //+70 due to static friction
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      pwmin = min(pwmin, 255);    //max pwm value could be 255
      analogWrite(enA, pwmin);    //PWM for controlling the motor
    }
    
  } else {
      analogWrite(enA, 0); //Stop motor!
  }
}
