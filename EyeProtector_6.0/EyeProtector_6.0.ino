#include <IRremote.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//#include "PinDefinitionsAndMore.h"
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define DECODE_NEC 1  // 디코딩 방식
#define timeSeconds 0.1 // 초
#define TIME_BUFFER_SIZE (30)
#define RAW_BUFFER_SIZE  (11)
#define SAVE_DATA_SIZE   (10)
#define DATA_BUFFER_SIZE (50)
#define BTN     13
#define LED_PIN 27
#define ECHO    12
#define TRIG    14
#define IR_RECEIVE_PIN 23

enum{
  RIGHT,
  LEFT,
  UP,
  DOWN
};

//const int IR_RECEIVE_PIN = 23;  // 데이터 핀
volatile boolean gLedState = LOW;
const int RGB[3] = {16, 17, 18};
int colorState[3] = {0,};
volatile int btnCount; // default : 0
float duration, distance;
int sonarDistance = 0;
int btnFlag = 0;
int menuFlag = 0;
int menuCount = 0;
int flag = 0;
int state = LOW;
int tempData;
int dataFlag = 0;
int detectCount = 0;
int detectFlag = 0;

unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

char strTime[TIME_BUFFER_SIZE];
char hexbuf[RAW_BUFFER_SIZE];
uint16_t* storage[SAVE_DATA_SIZE];
int dataArray[SAVE_DATA_SIZE];
char databuffer[DATA_BUFFER_SIZE];

uint16_t sAddress = 0x0000;
uint8_t sCommand  = 0x00;
uint8_t sRepeats  = 0;


// Storage for the recorded code IR저장구조체
struct storedIRDataStruct {
    IRData receivedIRData;
    // extensions for sendRaw
    uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw
    uint8_t rawCodeLength; // The length of the code
} sStoredIRData;

// 외부 인터럽트 구현, 내부는 계산만 하는게 좋음
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;  // 낮은 HZ일때 도움됨
void IRAM_ATTR buttonPressed(){
  portENTER_CRITICAL(&synch);
  btnCount++;
  portEXIT_CRITICAL(&synch);
}


void setup() {
    Serial.begin(115200);
    EEPROM.begin(256);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
    
    // 데이터 핀, 수신 확인용 LED<- 잘모르겠음
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver, enable feedback LED, take LED feedback pin from the internal boards definition
    IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK); // Specify send pin and enable feedback LED at default feedback LED pin
    Serial.print(F("Ready to receive IR signals at pin "));
    Serial.print(F("Ready to send IR signals at pin "));
    Serial.println(IR_SEND_PIN);
    pinMode(LED_PIN, OUTPUT);
    for(int i = 0; i<3; i++){
      ledcSetup(i, 500, 8);
      ledcAttachPin(RGB[i], i);
      ledcWrite(i, 0);
    }
    pinMode(BTN, INPUT_PULLUP);
    pinMode(ECHO, INPUT);
    pinMode(TRIG, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    // 버튼 누를 때  HIGH->LOW : FALLING, LOW->HIGH : RISING, 아무 때나 CHANGE
    attachInterrupt(digitalPinToInterrupt(BTN), buttonPressed, FALLING); 
    
    u8g2.begin();
    u8g2.enableUTF8Print();
    readData(dataArray, SAVE_DATA_SIZE);
    delay(1000);

}


void loop() {
    buttonClicked();
    detectDistance();

    Serial.print("detectCount : ");
    Serial.print(detectCount);
    Serial.print(" detectFlag : ");
    Serial.println(detectFlag);
    if((detectCount >= 0) && (detectCount <= 1000)){
      if(sonarDistance <= 10){
        detectCount++;
      }else{
        detectCount--;
      }
    }

    if (detectCount >= 1000){
      detectCount = 1000;
      detectFlag = 1;
    }else if (detectCount <= 0){
      detectCount = 0;
      detectFlag = 0;
    }
    
    
    if ((detectFlag == 1)&&(detectCount >= 1000)){      // 밝기 감소
      changeBright();
      Serial.println("밝기 감소");
      
      
    }else if((detectFlag == 1) && (detectCount <500)){  // 밝기 증가
      changeBright();
      Serial.println("밝기 증가");
      detectFlag = 0;
    }

    timeInterval();   // 카운트하는 시간 때문에 늘어남
}

void changeBright(){
  startTimer = true;
    lastTrigger = millis();   // 신호가 들어온 시간
    Serial.print("작동 시간 : ");
    Serial.println(lastTrigger);
    if(flag == 0){
      flag = 1;           //flag = 1 일 때는 추가 신호 안받음
      Serial.flush();
      Serial.println("move backWard");
      IrSender.sendNEC(sAddress, EEPROM.readInt(0), sRepeats);
      delay(1000);
      IrSender.sendNEC(sAddress, EEPROM.readInt(28), sRepeats);
      delay(1000);
      IrSender.sendNEC(sAddress, EEPROM.readInt(32), sRepeats);
      delay(1000);
      IrSender.sendNEC(sAddress, EEPROM.readInt(36), sRepeats);
      delay(1000);
      IrSender.sendNEC(sAddress, EEPROM.readInt(20), sRepeats);
  }
}


void buttonClicked(){
  if (btnCount > 0){
      portENTER_CRITICAL(&synch);
      btnCount--;
      portEXIT_CRITICAL(&synch);
      
      if(digitalRead(BTN)==LOW){
        if(btnFlag==0){
          //gLedState = !gLedState;
          //clearData(dataArray);
          IrReceiver.resume();
          initialArray(dataArray, SAVE_DATA_SIZE);
          //printArray(dataArray, SAVE_DATA_SIZE);
          Serial.print("버튼 눌러짐");
          //Serial.println(gLedState);
          u8g2.setFont(u8g2_font_unifont_t_korean2);
          setMenuButton();
          dataReceive();
          btnFlag = 1;
          
          Serial.println("저장 완료");
          delay(1000);
          //printArray(dataArray, SAVE_DATA_SIZE);
          readData(dataArray, SAVE_DATA_SIZE);
          endMonitor();
          
        }
      }else{
        if (btnFlag == 1){
          btnFlag = 0;
          Serial.println("중복방지");
        }
      }
    }
}

void dataReceive(){
  do{
    menuFlag = 1;
    if (IrReceiver.decode() && (dataFlag==0)) {
      Serial.println();
      // Print a short summary of received data1바이트 형식
      IrReceiver.printIRResultShort(&Serial); // 받은 데이터 시리얼에 표시
      // 알 수 없는 것이 들어오면 추가 정보 출력
      if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
          // We have an unknown protocol, print more info
          IrReceiver.printIRResultRawFormatted(&Serial, true);
      }
      IrReceiver.resume(); // Enable receiving of the next value
      dataArray[menuCount] = IrReceiver.decodedIRData.command;
      //saveData(dataArray[menuCount], menuCount);
      Serial.print("dataArray[");
      Serial.print(menuCount);
      Serial.print("] : 0x");
      Serial.println(dataArray[menuCount], HEX);
      //sendData();
      printRawData();
      //executeCommand();
      dataFlag = 1;
    }else{
      dataFlag = 0;
      delay(500);
    }
    
    if(menuCount == 0){
      if(dataArray[menuCount] != 0) {
        saveData(dataArray[menuCount], menuCount);
        menuCount++;
      }
    }else if ((menuCount > 0)&&(menuCount < 6)){
      if((dataArray[menuCount]!=0)&&(dataArray[menuCount] != dataArray[menuCount -1])){
        saveData(dataArray[menuCount], menuCount);
        menuCount++;
      }
    } else if ((EEPROM.readInt(0) == dataArray[menuCount])&&(menuCount == 6)){
      arrangeOrder();
      menuCount++;  // 메뉴
    } else if ((EEPROM.readInt(16) == dataArray[menuCount])&&(menuCount > 6)){
        if((dataArray[menuCount] == dataArray[menuCount-1])&&(tempData==0)){
            //pressEnterTwice();
            Serial.println("확인버튼 다시 누름");
            saveData(dataArray[menuCount], menuCount);
            menuCount++;
            
        }else if((dataArray[menuCount-1] == EEPROM.readInt(0))&&(tempData==0)){
            pressEnterTwice();
            Serial.println("잘못누름");
        }else{
          saveData(tempData, menuCount);
          //EEPROM.writeInt(4*menuCount, tempData);
          //EEPROM.commit();
          showComfirm();
          menuCount++;  // 확인 버튼 누르면 +1
          delay(1000);
          dataArray[menuCount] = 0;
        }
    }
    if (menuCount > 6){
      if (dataArray[menuCount] == EEPROM.readInt(4)){
        Serial.println("오른쪽");
        showDirection(RIGHT);
        tempData = EEPROM.readInt(4);
      }else if(dataArray[menuCount] == EEPROM.readInt(8)){
        Serial.println("왼쪽");
        showDirection(LEFT);
        tempData = EEPROM.readInt(8);
      }else if(dataArray[menuCount] == EEPROM.readInt(12)){
        Serial.println("아래");
        showDirection(DOWN);
        tempData = EEPROM.readInt(12);
      }else if(dataArray[menuCount] == EEPROM.readInt(20)){
        clearData(menuCount);
        break;
      }else{
        tempData = 0;
      }
    }
    

    switch(menuCount){
      case 1: setRightButton(); break; 
      case 2: setLeftButton();  break;
      case 3: setDownButton();  break;
      case 4: setEnterButton(); break;
      case 5: setExitButton();  break;
      case 6: setOrder(); break;   // 메뉴
    }
    
    if(menuCount == SAVE_DATA_SIZE){
      menuFlag = 0;
    }
  }while(menuFlag);
}

void saveData(int target ,int address){ 
  EEPROM.writeInt(4*address, target);
  EEPROM.commit();
  Serial.println("intData write in EEPROM is Successful");
}

void readData(int ARRAY[] ,int SIZE){
  int rInt;
  //String rString = EEPROM.readString();
  for(int i = 0; i < SIZE; i++){
    rInt = EEPROM.readInt(4*i);
    EEPROM.readInt(rInt);
    snprintf(databuffer, DATA_BUFFER_SIZE ,"[%d] : 0x%x", i, rInt);
    Serial.println(databuffer);
  } 
}

void clearData(int _position){
  for(int i = _position*4; i < EEPROM.length(); i++){
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  //EEPROM.end();
  Serial.println("EEPROM Clear Done!");
}

void initialArray(int ARRAY[] ,int SIZE){
  menuCount = 0;
  for(int i = 0; i < SIZE; i++){
    ARRAY[i] = 0;
  }  
}

void printArray(int ARRAY[] ,int SIZE){
  for(int i = 0; i < SIZE; i++){
    Serial.print("[");
    Serial.print(i);
    Serial.print("] : 0x");
    Serial.println(ARRAY[i], HEX);
  }  
}

void detectDistance(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = 340.0 * duration / 10000.0 / 2.0;
  sonarDistance = (int)distance;
  sonarDistance = constrain(sonarDistance, 0, 500);
}

void setMenuButton(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(20, 16);
  u8g2.print("안녕하세요.");
  u8g2.setCursor(20, 40);
  u8g2.print("메뉴버튼을");
  u8g2.setCursor(30, 56);
  u8g2.print("누르세요");
  u8g2.sendBuffer();
}

void setRightButton(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(20, 16);
  u8g2.print("1.오른쪽");
  u8g2.setCursor(20, 40);
  u8g2.print("오른쪽버튼을");
  u8g2.setCursor(30, 56);
  u8g2.print("누르세요");
  u8g2.sendBuffer();
}

void setLeftButton(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(20, 16);
  u8g2.print("2.왼쪽방향키");
  u8g2.setCursor(20, 40);
  u8g2.print("왼쪽버튼을");
  u8g2.setCursor(30, 56);
  u8g2.print("누르세요");
  u8g2.sendBuffer();
}

void setDownButton(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(20, 16);
  u8g2.print("3.아래방향키");
  u8g2.setCursor(20, 40);
  u8g2.print("아래버튼을");
  u8g2.setCursor(30, 56);
  u8g2.print("누르세요");
  u8g2.sendBuffer();
}

void setEnterButton(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(20, 16);
  u8g2.print("4.확인");
  u8g2.setCursor(40, 40);
  u8g2.print("버튼을");
  u8g2.setCursor(30, 56);
  u8g2.print("누르세요");
  u8g2.sendBuffer();
}

void setExitButton(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(10, 16);
  u8g2.print("5.종료(나가기)");
  u8g2.setCursor(40, 40);
  u8g2.print("버튼을");
  u8g2.setCursor(30, 56);
  u8g2.print("누르세요");
  u8g2.sendBuffer();
}

void setOrder(){
  if(menuCount == 6){
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.setCursor(0, 16);
    u8g2.print("메뉴버튼을 눌러");
    u8g2.setCursor(10, 40);
    u8g2.print("밝기메뉴까지");
    u8g2.setCursor(20, 56);
    u8g2.print("들어가주세요");
    u8g2.sendBuffer();
  }else{
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.setCursor(20, 16);
    u8g2.print("버튼을");
    u8g2.setCursor(40, 40);
    u8g2.print("다시 눌러");
    u8g2.setCursor(30, 56);
    u8g2.print("재설정");
    u8g2.sendBuffer();
  }
}

void arrangeOrder(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(0, 16);
  u8g2.print("방향키버튼을.");
  u8g2.setCursor(0, 40);
  u8g2.print("눌러");
  u8g2.setCursor(0, 56);
  u8g2.print("순서 나열");
  u8g2.sendBuffer();
}

void showDirection(int _direction){
  if(menuCount > 6){
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.setCursor(20, 16);
    switch(_direction){
      case RIGHT: u8g2.print("오른쪽버튼"); break;
      case LEFT: u8g2.print("왼쪽버튼"); break;
      case DOWN: u8g2.print("아래버튼"); break;
    }
    
    u8g2.setCursor(30, 40);
    u8g2.print("맞으시면");
    u8g2.setCursor(0, 56);
    u8g2.print("확인을 눌러주세요");
    
    u8g2.sendBuffer();
  }else{
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.setCursor(20, 16);
    u8g2.print("전원을");
    u8g2.setCursor(20, 40);
    u8g2.print("다시 연결하여");
    u8g2.setCursor(30, 56);
    u8g2.print("재설정");
    u8g2.sendBuffer();
  }
}

void showComfirm(){
  if((menuCount>6)&&(SAVE_DATA_SIZE>menuCount)){
      u8g2.setFontDirection(0);
      u8g2.clearBuffer();
      u8g2.setCursor(0, 16);
      u8g2.print("확인되었습니다.");
      u8g2.setCursor(0, 40);
      u8g2.print("종료(나가기)또는");
      u8g2.setCursor(10, 56);
      u8g2.print("다음 버튼클릭");
      u8g2.sendBuffer();
  }else{
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.setCursor(20, 16);
    u8g2.print("전원을");
    u8g2.setCursor(20, 40);
    u8g2.print("다시 연결하여");
    u8g2.setCursor(30, 56);
    u8g2.print("재설정");
    u8g2.sendBuffer();
  }

}

void pressEnterTwice(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(10, 16);
  u8g2.print("잘못눌렀습니다.");
  u8g2.setCursor(10, 40);
  u8g2.print("방향키를");
  u8g2.setCursor(30, 56);
  u8g2.print("눌러주세요");
  u8g2.sendBuffer();
}


void endMonitor(){
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(10, 16);
  u8g2.print("세팅이");
  u8g2.setCursor(10, 40);
  u8g2.print("완료되었습니다.");
  u8g2.setCursor(30, 56);
  u8g2.print("감사합니다.");
  u8g2.sendBuffer();
  delay(5000);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void executeCommand(){
  uint8_t myCommand = IrReceiver.decodedIRData.command;
  switch(myCommand){
    case 0x45: LED_ON_OFF(); break;
    case 0x44: RGB_ON_OFF(0, 255); break;
    case 0x40: RGB_ON_OFF(1, 255); break;
    case 0x43: RGB_ON_OFF(2, 127); break;
  }
}

void sendData(int Command){
    startTimer = true;
    lastTrigger = millis();
    Serial.print("작동 시간 : ");
    Serial.println(lastTrigger);
    sAddress = 0x0001;
    //sCommand = 0x45;
    sRepeats = 2;
    
    //Serial.println(F("Send NEC with 16 bit address"));
    //Serial.flush();

    // Results for the first loop to: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    //IrSender.sendNEC(sAddress, sCommand, sRepeats);
    if (flag == 0){  
      flag = 1;
    //Serial.println();
      Serial.print(F("Send now: address=0x"));
      Serial.print(sAddress, HEX);
      Serial.print(F(" command=0x"));
      Serial.print(Command, HEX);
      Serial.print(F(" repeats="));
      Serial.print(sRepeats);
      Serial.println();
      Serial.flush(); // 전송하고 있는 시리얼 데이터가 전송 완료될때까지 대기하는 함수, 큰 차이는 없음 
      
      IrSender.sendNEC(sAddress, Command, sRepeats);
    }
    /*
     * If you cannot avoid to send a raw value directly like e.g. 0xCB340102 you must use sendNECRaw()
     */
//    Serial.println(F("Send NECRaw 0xCB340102"));
//    IrSender.sendNECRaw(0xCB340102, sRepeats);
}


void timeInterval(){
    now = millis(); // 현재 시간은 계속 측정 중 -> 신호 때 받은 시간과 비교
    if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
        Serial.print("초기화시간 : ");
        Serial.println(now);
        float interval = (float)(now-lastTrigger)/1000;
        snprintf(strTime, TIME_BUFFER_SIZE, "간격 : %.1f 초", interval);
        Serial.println(strTime);
        flag = 0;   // flag -> 0일 때 작업가능
        startTimer = false;
        
        Serial.println("Initializing");   // 작업간 충돌 방지
    }
}


void printRawData(){
    Serial.println("=============================");
    Serial.print("RawData[DECIMAL] : ");
    Serial.println(IrReceiver.decodedIRData.decodedRawData);
    
    snprintf(hexbuf, RAW_BUFFER_SIZE, "0x%08X", IrReceiver.decodedIRData.decodedRawData);
    Serial.print("RawData[HEX]     : ");
    Serial.println(hexbuf);
    Serial.println("=============================");
}


void LED_ON_OFF(){
    startTimer = true;
    lastTrigger = millis();   // 신호가 들어온 시간
    Serial.print("작동 시간 : ");
    Serial.println(lastTrigger);
    if(flag == 0){
      flag = 1;           //flag = 1 일 때는 추가 신호 안받음
      state = !state;
      delay(100);
      Serial.print("state : ");
      Serial.println(state);
      digitalWrite(LED_PIN, state);
      Serial.println("LED state Changed");
  }
}

void RGB_ON_OFF(int color, int brightness){
    startTimer = true;
    lastTrigger = millis();   // 신호가 들어온 시간
    Serial.print("작동 시간 : ");
    Serial.println(lastTrigger);
    if(flag == 0){
      flag = 1;           //flag = 1 일 때는 추가 신호 안받음
      colorState[color] = !colorState[color];
      delay(100);
      Serial.print("state : ");
      Serial.println(colorState[color]);
      ledcWrite(color, brightness*colorState[color]);
      Serial.println("LED state Changed");
  }
}
