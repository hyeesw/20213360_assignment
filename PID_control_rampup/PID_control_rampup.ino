#include <math.h>
#include <Servo.h>

//6점짜리

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9            // [20200000] LED핀 설정
#define PIN_SERVO 10         //[20191979] Servo 핀 설정
#define PIN_IR A0            // [20191979] 적외선 핀(아날로그핀)

// Framework setting
#define _DIST_TARGET 255    // [20213085] 정지하려는 위치 목표값
#define _DIST_MIN 100         //[20191979] 최소거리
#define _DIST_MAX 410         //[20191979] 최대거리

// Distance sensor
#define _DIST_ALPHA 0.5 //[20203118] DIST_ALPHA 값 설정

// Servo range
#define _DUTY_MIN 1400    // [20213083]서보 각도의 최솟값 설정
#define _DUTY_NEU 1480    // [20213090]서보 수평 각도 펄스 값
#define _DUTY_MAX 1650        // [20213081] 서보 각도의 최댓값

// Servo speed control
#define _SERVO_ANGLE 45 //45 [20203118] 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 1000 //[20203118] 서보 속도 설정

// Event periods
#define _INTERVAL_DIST 20   // [20213099] 거리 센서 주기 설정
#define _INTERVAL_SERVO 20  // [20213099] 서보 주기 설정
#define _INTERVAL_SERIAL 100  // [20213099] 시리얼 표시 주기 설정

// PID parameters
#define _KP 1.7        //[20191979] 비례제어 값
#define _KD 10        // 미분제어 값 17
#define _KI 1         // 적분제어 값 1

// ramp_up parameters
#define _RAMPUP_TIME 360 // servo speed rampup (0 to max) time (unit: ms)
#define INTERVAL 20  // servo update interval
#define START _DUTY_MIN + 100 //-200
#define END _DUTY_MAX - 100

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  // [20213090] 서보 인스턴스 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; //[20203118] 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial, last_sampling_time; //[20203118] 마지막으로 측정한 거리, 마지막으로 측정한 서보 각도(각 이벤트별로 업데이트를 위해 측정하는 시간)
//각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; //[20203118] 이벤트 별로 이번 루프에서 업데이트 여부
//각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval; //[20213086]interval 당 최대 duty 변화량
int duty_target, duty_curr; //[20213086] 목표 duty와 현재 duty 값

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
//[20213086] error_curr: 현재 주기 오차값 / error_prev : 이전 주기 오차 값 / control : PID 제어량 / pterm, dterm, iterm : 비례,적분,미분 이득값

//ramp_up variables
int duty_chg_max; // maximum speed, i.e., duty difference per interval (unit: us/interval)
int duty_chg_adjust; // duty accelration per interval during ramp up/down period (unit: us/interval^2)
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec

float _ITERM_MAX = 50; //50 줄이니까 더 활발해짐

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  
// move servo to neutral position
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = START; // duty_NEU -> START
  myservo.writeMicroseconds(duty_curr + 1000);

// initialize global variables
duty_curr=_DUTY_MIN + 300; // ??? [20203118] duty_curr 값 초기화 
dist_raw = dist_ema = ir_distance_filtered(); // 초기화
error_curr = error_prev = dist_target - dist_ema;  // 초기화
dist_target=_DIST_TARGET; //[20203118] dist_target 값 초기화
pterm = 0;
dterm = 0;
iterm = 0;

last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
event_dist = event_servo = event_serial = false;

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);//[20203118] duty_chg_per_interval 값 설정   
//[20213103] 오류 수정

// servo related variables
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  duty_chg_adjust = (float) duty_chg_max * INTERVAL / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.
// initialize variables for duty_target update.
  pause_time = 0.5;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  //toggle_interval = 1; // to demonstrate overshoot
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;

}
  


void loop() {
/////////////////////
// Event generator //
/////////////////////
// [20213090]
if(millis() < last_sampling_time + INTERVAL) return;

unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
}
if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
}
if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
}


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = false;
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered();
      dist_ema = (1 - _DIST_ALPHA)*dist_ema + _DIST_ALPHA*dist_raw; // [20213083]

  // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr; //[20213083]
    dterm = _KD * (error_curr - error_prev);    //[20191979]
    iterm += _KI * error_curr;   // 적분 control
    
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX * 0.00001;
    if(iterm < -_ITERM_MAX) iterm = -_ITERM_MAX * 0.00001;
    
    //if(abs(iterm) > _ITERM_MAX) iterm = 0; //시연 1
    control = pterm + dterm + iterm; //[20203118]  

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; //[20213086]

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}
if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}
//[20213086]

error_prev = error_curr;
  }
  
  if(event_servo) {
event_servo=false; //[20203118] 

    // adjust duty_curr toward duty_target by duty_chg_per_interval
if(duty_target > duty_curr) { //목표 지점이 현재 지점보다 멀리있다면 -> 멀리 빨리 반응해서 오버슈트를 없애야함
    if(duty_target - 100 < duty_curr) { //40
      iterm = iterm * 0.00001;
    }
    if(duty_chg_per_interval < duty_chg_max) {
      duty_chg_per_interval += duty_chg_adjust;
      if(duty_chg_per_interval > duty_chg_max) 
        duty_chg_per_interval = duty_chg_max;
    }
    duty_curr += duty_chg_per_interval; 
    if(duty_curr > duty_target)//오버슈팅 된 경우(더했는데 너무 더해서 오바난 경우)
    duty_curr = duty_target + 10; //목적지로 오게함.
}
else if(duty_target < duty_curr) {  //목표 지점이 현재 지점보다 가까이 있다면 -> 가까이
    if(duty_target + 40 > duty_curr) { 
      iterm = iterm * 0.00001;
    }
    if(duty_chg_per_interval > -duty_chg_max) {
      duty_chg_per_interval -= duty_chg_adjust;
      if(duty_chg_per_interval < -duty_chg_max) 
        duty_chg_per_interval = -duty_chg_max;
    }
    duty_curr += duty_chg_per_interval;
    if(duty_curr < duty_target) 
      duty_curr = duty_target; //(뺐는데 너무 빼서 오바난 경우)
      iterm = iterm * 0.1;
  }
  else {
    duty_chg_per_interval = 0;
    iterm = iterm * 0.0001;
  }
    // update servo position
myservo.writeMicroseconds(duty_curr); //[20213086]
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_ema+ 52);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
  // toggle duty_target between _DUTY_MIN and _DUTY_MAX.
  if(toggle_interval_cnt >= toggle_interval) { //갱신할 때
    toggle_interval_cnt = 0;
    if(duty_curr < START + 200) //현위치가 start + 200 보다 작으면
      duty_target = END; // end로 결정 duty_max - 100
    else if(duty_curr > END - 200)  // 현위치가 end - 200 보다 크면 
      duty_target = START; // start로 결정 duty_min + 100
  }
  else {
    toggle_interval_cnt++;
  }
  // update last sampling time
  last_sampling_time += INTERVAL;
}

float ir_distance(void){ // return value unit: mm
  float val;
  int a = 100;
  int b = 400;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (_DIST_MAX - _DIST_MIN) *(val - _DIST_MIN);//[20213092] 오류수정
  val = 100 + 300.0 / (b-a) * (val - a);
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}
