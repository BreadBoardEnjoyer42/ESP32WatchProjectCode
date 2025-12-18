#include <Arduino.h>
#include "LCD_Test.h"
#include "driver/rtc_io.h"
#include <math.h>
#include <WiFi.h>
#include "time.h"
UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage;
CST816S touch(6, 7, 13, 5);	// sda, scl, rst, irq

const char* ssid     = "########";
const char* password = "########";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -18000;
const int   daylightOffset_sec = 3600;

#include <ESP32Time.h>
ESP32Time rtc(3600);  // offset in seconds GMT+1
const int wakeupPin = 16;  // GPIO 16 for external wake-up
RTC_DATA_ATTR int bootCount = 0;  // Number of reboots
RTC_DATA_ATTR unsigned long sleepStartTime = 0;
bool loc[4];
void time(int, int);
void brightness();
void clockface();
void timebutton();
void date(int, int);
void bye(int, int);

int trackx = 2;
int tracky = 2;
int back = 15;
int currents = 0;
int prevs = 0;

int prevx;
int prevy; //debounce shit

void setup()
{
    Serial.begin(115200);
    setCpuFrequencyMhz(40);
    pinMode(wakeupPin, INPUT_PULLUP);  // Declaring the pin with the push button as INPUT_PULLUP
    rtc_gpio_pullup_en(GPIO_NUM_16);
    touch.begin();
for(int xint = 0; xint <= 9; xint++){
  loc[xint] = false; //writing everything to false
}
loc[2] = true; //set default location as center of array
    if (bootCount == 0){
delay(1000);
     setCpuFrequencyMhz(80);
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
   struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  setCpuFrequencyMhz(40);
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  int inth = atoi(timeHour)-1; //fuck daylight savings, making me change my code and shit

  char timeMinute[3];
  strftime(timeMinute,3, "%M", &timeinfo);
  int intmin = atoi(timeMinute);

  char timeSecond[3];
  strftime(timeSecond,3, "%S", &timeinfo);
  int intsec = atoi(timeSecond);

  char timeDay[3];
  strftime(timeDay,3, "%d", &timeinfo);
  int intday = atoi(timeDay);

  char timeMonth[3];
  strftime(timeMonth,3, "%m", &timeinfo);
  int intmon = atoi(timeMonth);

  char timeYear[5];
  strftime(timeYear,5, "%Y", &timeinfo);
  int intyear = atoi(timeYear);
  if(intmon == 1){
    intmon = 2;
  }
  rtc.setTime(intsec, intmin, inth, intday, intmon, intyear);
    }
    // PSRAM Initialize
    if(psramInit()){
      Serial.println("\nPSRAM is correctly initialized");
    }else{
      Serial.println("PSRAM not available");
    }
    if ((BlackImage = (UWORD *)ps_malloc(Imagesize)) == NULL){
        Serial.println("Failed to apply for black memory...");
        exit(0);
    }
    // put your setup code here, to run once:
    if (DEV_Module_Init() != 0)
      Serial.println("GPIO Init Fail!");
    else
      Serial.println("GPIO Init successful!");
      LCD_1IN28_Init(HORIZONTAL);
      LCD_1IN28_Clear(BLACK);  
      /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
      Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
      Paint_SetScale(65);
      Paint_SetRotate(ROTATE_90);
      Paint_Clear(BLACK);
      unsigned long sleepDuration = millis() - sleepStartTime; // Calculate sleep time [1, 2, 7]
      Serial.print("Sleep duration: "); 
      Serial.println(sleepDuration); 
pinMode(15, INPUT_PULLUP);
pinMode(17, OUTPUT);
}
void loop(){
  DEV_SET_PWM(back);
  if (digitalRead(15) == 0){
    digitalWrite(17, HIGH);
  }else{
    digitalWrite(17,LOW);
  }
  if(digitalRead(16) == 0){
    bootCount++;  // Increment the number of boots by 1
    bye(100,50);
  Serial.println("Boot number: " + String(bootCount));  // Print the boot number
  esp_sleep_enable_ext0_wakeup((gpio_num_t)wakeupPin, LOW);  // Configure external wake-up
  delay(1000);  // Adding a 1 second delay to avoid multiple presses
  Serial.println("I'm going to sleep now.");  // Print a statement before entering deep sleep
  sleepStartTime = millis();
  esp_deep_sleep_start();  // Enter deep sleep mode
  }
if (touch.available()){
if(touch.data.y>165 && touch.data.x>80 && touch.data.x<160){
  currents = touch.data.y;
  if (currents != prevs){
loc[tracky] = false; //previous location is voided
  tracky++; //go to new location
  if(tracky >= 5){
    tracky = 5;
  }
loc[tracky] = true;
  }
        }
else if(touch.data.y<75){
    currents = touch.data.y;
  if (currents != prevs){
loc[tracky] = false;
  tracky--;
   if(tracky <= 1){
    tracky = 1;
  }
loc[tracky] = true;
  }
        }
}
prevs = currents;
if (loc[2] == true){
  Paint_SetScale(65); 
  Paint_Clear(BLACK);
  time(110,0);
  LCD_1IN28_Display(BlackImage); 
} else if(loc[3] == true){
  Paint_SetScale(65); 
  Paint_Clear(BLACK);
  date(110,0);
  LCD_1IN28_Display(BlackImage);
} else if(loc[1] == true){
  Paint_SetScale(65); 
  Paint_Clear(BLACK);
  clockface();
  LCD_1IN28_Display(BlackImage);
} else if (loc[4] == true){
  Paint_SetScale(65); 
  Paint_Clear(BLACK);
  brightness();
  LCD_1IN28_Display(BlackImage);
} else if (loc[5] == true){
  Paint_SetScale(65); 
  Paint_Clear(BLACK);
  timebutton();
  LCD_1IN28_Display(BlackImage);
  }
//Serial.print(touch.data.x);
//Serial.print("  ");
//Serial.println(touch.data.y);
  delay(150);//allow the cpu to switch to other tasks
}
void time(int y, int x){

  int hourrtc = rtc.getHour();
  bool ampm = false;
  if (bootCount != 0){
    hourrtc -= 5; 
    if(hourrtc <= 0){
    hourrtc = hourrtc + 12;
    } else{
    }
  }
 int hourrtcampm = rtc.getHour(true);
  // bool ampm = false;
  if (bootCount != 0){
    hourrtcampm -= 5; 
    if(hourrtcampm <= 0){
    hourrtcampm = hourrtcampm + 24;
    } else{
    }
  if (hourrtcampm >= 0 && hourrtcampm <= 11){
  ampm = true;
}else{
  ampm = false;
}
  }


  Paint_DrawNum(30+x, y, hourrtc, &Font24, 0, WHITE, BLACK);
  Paint_DrawString_EN(60+x, y, ":", &Font24, BLACK, WHITE);
  Paint_DrawNum(80+x, y, rtc.getMinute(), &Font24, 0, WHITE, BLACK);
  Paint_DrawString_EN(110+x, y, ":", &Font24, BLACK, WHITE);
  Paint_DrawNum(130+x, y, rtc.getSecond(), &Font24, 0, WHITE, BLACK);
 
if (bootCount == 0){
  if (rtc.getHour(true) <= 11){
    Paint_DrawString_EN(180+x,y,"AM", &Font24, BLACK, WHITE);
    Paint_DrawRectangle(0, 00, 240, 47, 0x00D4, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 193, 240, 240, 0x00D4, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  }else{
    Paint_DrawString_EN(180+x,y,"PM", &Font24, BLACK, WHITE);
    Paint_DrawRectangle(0, 00, 240, 47, 0x48CF, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 193, 240, 240, 0x48CF, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  }
} else {
  if (ampm == true){
    Paint_DrawString_EN(180+x,y,"AM", &Font24, BLACK, WHITE);
    Paint_DrawRectangle(0, 00, 240, 47, 0x00D4, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 193, 240, 240, 0x00D4, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  }else{
    Paint_DrawString_EN(180+x,y,"PM", &Font24, BLACK, WHITE);
    Paint_DrawRectangle(0, 00, 240, 47, 0x48CF, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 193, 240, 240, 0x48CF, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  }
}

}


void brightness(){
  Paint_DrawRectangle(0, 0, 240, 120, RED, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  Paint_DrawRectangle(0, 120, 240, 240, BLUE, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  Paint_DrawLine(100, 50, 140, 50, BLACK, DOT_PIXEL_4X4, LINE_STYLE_SOLID);
  Paint_DrawLine(120, 30, 120, 70, BLACK, DOT_PIXEL_4X4, LINE_STYLE_SOLID);
  Paint_DrawLine(100, 50, 140, 50, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Paint_DrawLine(120, 30, 120, 70, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Paint_DrawLine(100, 180, 140, 180, BLACK, DOT_PIXEL_4X4, LINE_STYLE_SOLID);
  Paint_DrawLine(100, 180, 140, 180, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Paint_DrawCircle(115, 125, 37, 0x3A2A, DOT_PIXEL_6X6, DRAW_FILL_FULL);
  Paint_DrawCircle(120, 120, 35, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(120, 120, 30, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawNum(102,110,back, &Font24, 0, WHITE, BLACK);
  if (touch.available()){
  if(touch.data.x >= 120 && touch.data.y>=70 && touch.data.y<=170){
    back = back + 5;
    if (back >= 100){
        back = 100;
    }
  }else if(touch.data.x <= 120 && touch.data.y>=70 && touch.data.y<=170){
    back = back - 5;
    if (back <= 5){
        back = 5;
    }
  }
  } 
}
void clockface(){
  int hourrtc = rtc.getHour();
  bool ampm = false;
  if (bootCount != 0){
    hourrtc -= 5; 
    if(hourrtc <= 0){
    hourrtc = hourrtc + 12;
    } else{
    }
  }
 int hourrtcampm = rtc.getHour(true);
  // bool ampm = false;
  if (bootCount != 0){
    hourrtcampm -= 5; 
    if(hourrtcampm <= 0){
    hourrtcampm = hourrtcampm + 24;
    } else{
    }
  if (hourrtcampm >= 0 && hourrtcampm <= 11){
  ampm = true;
}else{
  ampm = false;
}
  }

int theta1 = map(rtc.getSecond(),0, 59, 0, 360);
int theta2 = map(rtc.getMinute(),0, 59, 0, 360);
int theta3 = map(hourrtc,0, 12, 0, 360);
float vx1 = 120+cos(0.0174532925199*(-theta1+90))*70;
float vy1 = 120+-sin(0.0174532925199*(-theta1+90))*70;
float vx2 = 120+cos(0.0174532925199*(-theta2+90))*70;
float vy2 = 120+-sin(0.0174532925199*(-theta2+90))*70;
float vx3 = 120+cos(0.0174532925199*(-theta3+90))*50;
float vy3 = 120+-sin(0.0174532925199*(-theta3+90))*50;
Paint_DrawCircle(120, 120, 110, WHITE, DOT_PIXEL_6X6, DRAW_FILL_EMPTY);
Paint_DrawCircle(120, 120, 35.5, ASH, DOT_PIXEL_6X6, DRAW_FILL_EMPTY);

if (bootCount == 0){
  if (rtc.getHour(true) <= 11){
    Paint_DrawCircle(120, 120, 35, 0xFCA1, DOT_PIXEL_6X6, DRAW_FILL_FULL);
  }else{
    Paint_DrawCircle(120, 120, 35, 0x255A, DOT_PIXEL_6X6, DRAW_FILL_FULL);
  }
} else {
  if (ampm == true){
    Paint_DrawCircle(120, 120, 35, 0xFCA1, DOT_PIXEL_6X6, DRAW_FILL_FULL);
  }else{
    Paint_DrawCircle(120, 120, 35, 0x255A, DOT_PIXEL_6X6, DRAW_FILL_FULL);
  }
}
Paint_DrawLine(120, 20, 120, 5, 0xE947, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120, 220, 120, 235, 0xE947, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(20, 120, 5, 120, 0xE947, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(220, 120, 235, 120, 0xE947, DOT_PIXEL_6X6, LINE_STYLE_SOLID);

Paint_DrawLine(120+cos(0.0174532925199*(-30))*100, 120+-sin(0.0174532925199*(-30))*100, 120+cos(0.0174532925199*(-30))*115, 120+-sin(0.0174532925199*(-30))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-60))*100, 120+-sin(0.0174532925199*(-60))*100, 120+cos(0.0174532925199*(-60))*115, 120+-sin(0.0174532925199*(-60))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-120))*100, 120+-sin(0.0174532925199*(-120))*100, 120+cos(0.0174532925199*(-120))*115, 120+-sin(0.0174532925199*(-120))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-150))*100, 120+-sin(0.0174532925199*(-150))*100, 120+cos(0.0174532925199*(-150))*115, 120+-sin(0.0174532925199*(-150))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-210))*100, 120+-sin(0.0174532925199*(-210))*100, 120+cos(0.0174532925199*(-210))*115, 120+-sin(0.0174532925199*(-210))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-240))*100, 120+-sin(0.0174532925199*(-240))*100, 120+cos(0.0174532925199*(-240))*115, 120+-sin(0.0174532925199*(-240))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-300))*100, 120+-sin(0.0174532925199*(-300))*100, 120+cos(0.0174532925199*(-300))*115, 120+-sin(0.0174532925199*(-300))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);
Paint_DrawLine(120+cos(0.0174532925199*(-330))*100, 120+-sin(0.0174532925199*(-330))*100, 120+cos(0.0174532925199*(-330))*115, 120+-sin(0.0174532925199*(-330))*115, ASH, DOT_PIXEL_6X6, LINE_STYLE_SOLID);

Paint_DrawLine(120, 120, vx2, vy2, WHITE, DOT_PIXEL_3X3, LINE_STYLE_SOLID); //minutes
Paint_DrawLine(120, 120, vx1, vy1, RED, DOT_PIXEL_2X2, LINE_STYLE_SOLID); //seconds
Paint_DrawLine(120, 120, vx3, vy3, GRAY, DOT_PIXEL_4X4, LINE_STYLE_SOLID); //hours

Paint_DrawCircle(120, 120, 4, 0x0257, DOT_PIXEL_6X6, DRAW_FILL_FULL);
}
void date(int y, int x){
 int dayoff = rtc.getDay();
  if (bootCount != 0){
    dayoff; 
  }
  Paint_DrawNum(30+x, y, rtc.getMonth(), &Font24, 0, WHITE, BLACK);
  Paint_DrawString_EN(60+x, y, "/", &Font24, BLACK, WHITE);
  Paint_DrawNum(80+x, y, dayoff, &Font24, 0, WHITE, BLACK);
  Paint_DrawString_EN(110+x, y, "/", &Font24, BLACK, WHITE);
  Paint_DrawNum(130+x, y, rtc.getYear(), &Font24, 0, WHITE, BLACK);

  Paint_DrawRectangle(0, 0, 240, 47, BLUEV, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  Paint_DrawRectangle(0, 193, 240, 240, BLUEV, DOT_PIXEL_2X2, DRAW_FILL_FULL);
}
void timebutton(){
int currentsy = touch.data.y;
int currentsx = touch.data.x;
Paint_DrawCircle(120, 120, 118, 0x8430, DOT_PIXEL_6X6, DRAW_FILL_FULL);
Paint_DrawCircle(120, 130, 50, 0x3A2A, DOT_PIXEL_6X6, DRAW_FILL_FULL);
Paint_DrawCircle(120, 120, 50, RED, DOT_PIXEL_6X6, DRAW_FILL_FULL);
Paint_DrawString_EN(100, 100, "SET", &Font20, RED, BLACK);
Paint_DrawString_EN(100, 120, "NTP", &Font20, RED, BLACK);
if(touch.data.y <= 160 && touch.data.y >= 80 && currentsy != prevy && currentsx != prevx){
  bootCount = 0;
  bye(100,50);
 Serial.println("Boot number: " + String(bootCount));  // Print the boot number
  esp_sleep_enable_ext0_wakeup((gpio_num_t)wakeupPin, LOW);  // Configure external wake-up
  delay(1000);  // Adding a 1 second delay to avoid multiple presses
  Serial.println("I'm going to sleep now.");  // Print a statement before entering deep sleep
  sleepStartTime = millis();
  esp_deep_sleep_start();  // Enter deep sleep mode
}
prevy = currentsy;
prevx = currentsx;
}

void bye(int y, int x){
  Paint_SetScale(65); 
  Paint_Clear(BLACK);
    LCD_1IN28_Display(BlackImage);
  Paint_DrawString_EN(x, y, "Goodbye!", &Font24, BLACK, BLUE);
  delay(250);
    LCD_1IN28_Display(BlackImage);
  Paint_DrawString_EN(x, y, "Goodbye!", &Font24, BLACK, BRED);
  delay(250);
    LCD_1IN28_Display(BlackImage);
  Paint_DrawString_EN(x, y, "Goodbye!", &Font24, BLACK, GRED);
  delay(250);
    LCD_1IN28_Display(BlackImage);
  Paint_DrawString_EN(x, y, "Goodbye!", &Font24, BLACK, GBLUE);
  delay(250);
  LCD_1IN28_Display(BlackImage);
  Paint_DrawString_EN(x, y, "Goodbye!", &Font24, BLACK, RED);
  delay(250);
   LCD_1IN28_Display(BlackImage);
  Paint_DrawString_EN(x, y, "Goodbye!", &Font24, BLACK, MAGENTA);
  delay(250);
  DEV_Delay_ms(1000);
}
