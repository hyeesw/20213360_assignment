#define PIN_LED 7

void setup() {
  Serial.begin(115200); 
  pinMode(PIN_LED, OUTPUT);
}
// 0:켜짐 , 1:꺼짐
void loop() {
        digitalWrite(PIN_LED, 0); //13번에 high 보내기
        delay(1000); // 1초동안 high 계속 나가고 있음
        int i;
        for (i=0; i<5; i++){
          digitalWrite(PIN_LED,1); // LED 끄기
          delay(100); // 100ms 끄기 지속
          digitalWrite(PIN_LED,0); // 켜기
          delay(100); // 100ms 켜기 지속
        }
        while(1){
          digitalWrite(PIN_LED,1);        
        }
        
}
