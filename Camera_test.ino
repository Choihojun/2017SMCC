//#include <MsTimer2.h>
#include <Servo.h>

Servo myservo;

//1m/s
//센서 및 GAIN 설정
#define LINE_REF 400 //라인카메라 기준치 설정
#define CAMERA_DELAY 0.025 //라인카메라 딜레이(fix Value)
#define ANGLE_P_GAIN 0.9 //임시 비례상수 : 1.0
#define DC_P_GAIN 3.0 //임시 비례상수 : 1.0
#define RPM_REF 500 //원하는 분당 회전수

//물리적 상수 (측정값)
#define LINE_LENGTH 57//라인카메라 측정 거리(수평) cm
#define WHEEL_R 93.67//자동차 바퀴 지름 mm
#define SPR 1000//1회전 당 클럭수

//핀 설정
#define CLK_LINE 4//라인카메라 CLK 핀
#define SIG_LINE 5//라인카메라 SIG 핀
//#define OUT_LINE A0 //라인카메라 OUT 핀

#define DC_PWM 6//모터드라이버 PWM핀

#define IN1 12//모터드라이버 INA
#define IN2 13//모터드라이버 INB
//11 : Short 00 : Not

#define SRV_SIG 9//서보모터 SIG 핀[PWM]

#define ENC_X 4//엔코더 X 핀(2번핀 예약)
#define ENC_Y 8//엔코더 Y 핀 

//타이머 주기 설정
#define mT 10 //타이머함수 주기(ms)

int i_line = 0;
int cnt = 0;

int lineData[128]; //라인카메라 센서 값
int edgeData[127]; //라인카메라 엣지 검출
int positionQueue[3]; //포지션 큐 Data : 0 ~ 127
int positionCount = 0; //포지션 카운트 : BOT

float vectorData[2]; //실시간 속도 벡터 순서쌍 (x, y)
int lengthData; //적외선 카메라 거리 값
int timerCount; //타이머 카운트
int servoTimerCount; //서보모터 카운트

unsigned long currentRPM = 0; //실시간 RPM
int currentOrient = 90; //서보모터 각도 값(중앙 90')
int encoderCount = 0; //엔코더 카운트

int currentDcPwm = 0; //DC 모터드라이버 PWM 값(PWM)
int currentDcDir = 1; //DC 모터드라이버 DIR 값(CW/CCW)


void readLine(); //라인카메라 값 측정 함수
void readENC(); //엔코더 카운트 업 <ISR>
void readRPM(); //엔코더 각속도
void readLength(); //적외선센서 값 측정 함수

void calRisingEdge(); //엣지 검출
void calPosition(); //위치 계산
void calVector(); //벡터 계산
void calOrient(); //적정방향 계산

void setRPM(); //RPM 설정
void setOrient(int angle); //서보모터 각도 설정 90' : 기준

void setup() {
   Serial.begin(9600);
   

   pinMode(CLK_LINE, 1); //라인카메라 CLK 핀
   pinMode(SIG_LINE, 1); //라인카메라 SIG 핀
   pinMode(DC_PWM, 1); //모터드라이버 PWM핀
   pinMode(IN1, 1); //모터드라이버 DIR핀(1)
   pinMode(IN2, 1); //모터드라이버 DIR핀(2)

   pinMode(ENC_Y, 0); //엔코더 Y 핀

   myservo.attach(SRV_SIG); //서보모터 SIG 핀

                      //라인카메라 OUT 핀 : A0
                      //적외선카메라 OUT 핀 : A1
   currentDcPwm = 40;
   
   
//   MsTimer2::set(mT, readRPM); //호출 주기 mT, 서비스 루틴 readRPM
//   MsTimer2::start();
}

void loop() {

   //속도 유지


   readLine();
   calRisingEdge(); //엣지 검출
                
                for (int i = 0; i < 128; i++) {
                Serial.print( lineData[i]);
                Serial.print(" ");
                }
                Serial.print("\n");
  

   delay(100);


}

//끝
void readLine() { //라인카메라 값
   i_line = 0;

   delayMicroseconds(CAMERA_DELAY);
   digitalWrite(SIG_LINE, 1);

   delayMicroseconds(CAMERA_DELAY);
   digitalWrite(CLK_LINE, 1);
   delayMicroseconds(CAMERA_DELAY);

   digitalWrite(SIG_LINE, 0);
   delayMicroseconds(CAMERA_DELAY);

   lineData[i_line] = analogRead(A0); //카메라 값 수신 : ADC[0]
   digitalWrite(CLK_LINE, 0);
   for (i_line = 1; i_line < 128; i_line++)
   {
      delayMicroseconds(CAMERA_DELAY);
      digitalWrite(CLK_LINE, 1);
      delayMicroseconds(CAMERA_DELAY);
      lineData[i_line] = analogRead(A0); //카메라 값 수신 : ADC[i]
      digitalWrite(CLK_LINE, 0);
   }
   //종료 클럭(탈출 클럭 : 129CLK)
   delayMicroseconds(CAMERA_DELAY);
   digitalWrite(CLK_LINE, 1);
   delayMicroseconds(CAMERA_DELAY);
   digitalWrite(CLK_LINE, 0);
   delayMicroseconds(CAMERA_DELAY);
}



//끝
void calRisingEdge() { //엣지 검출
                  //이진화

   for (int i = 0; i < 128; i++)
   {
      if (lineData[i] > LINE_REF)
      {
         lineData[i] = 1;
      }
      else
      {
         lineData[i] = 0;
      }
   }

   //엣지 검출
   for (int i = 0; i < 127; i++)
   {
      if (lineData[i + 1] - lineData[i] > 0)
      {
         edgeData[i] = 1;
      }
      else
      {
         edgeData[i] = 0;
      }
   }
}



