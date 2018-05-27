#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define BEEP_PIN 8
#define BEEP_GROUND 9
#define BUTTON_GROUND 7
#define BUTTON_PIN 6
#define DEG2RAD 57.295779
#define BOUNCETIME 333
#define MAX_RADIUS 180
#define MIN_RADIUS 10
#define RADIUS_INC 10
#define PRESSED LOW
#define RELEASED HIGH
#define ONE_SECOND 1000
#define INCREMENT_TIME 1000
#define ALARM_TIME_THRESHOLD 20
#define GPS_TIME_THRESHOLD 120

SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean);

int alarm_radius = MIN_RADIUS;
float dist = 0;
float angle = 0;

float anchor_lat;
float anchor_lon;
float boat_lat;
float boat_lon;

boolean set = false;
int current = HIGH;
int previous = HIGH;
uint32_t last_change = millis();
uint32_t since_previous = 0;
int change;
boolean button_change = false;
boolean previous_alarm = false;
boolean previous_fixalarm = false;
uint32_t alarm_time = 0;
uint32_t alarm_since = 0;
uint32_t nofix_time = 0;
uint32_t nofix_since = 0;
boolean alarm = false;
boolean nofix = false;
uint32_t last_fix = 0;

void setup()  
{  
  pinMode(BEEP_PIN, OUTPUT);
  digitalWrite(BEEP_PIN, LOW);
  pinMode(BEEP_GROUND, OUTPUT);
  digitalWrite(BEEP_GROUND, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_GROUND, OUTPUT);
  digitalWrite(BUTTON_GROUND, LOW);
    
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PMTK_ENABLE_WAAS);
  
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);  delay(1000);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("iWATCH 1.2");
  display.display();
  delay(1000);
  
  while(!GPS.fix){
    if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.fillRect(0, 0, 48, 32, BLACK);
    display.print("S"); display.print((int) GPS.satellites);
    display.print(" H"); display.print((float) GPS.HDOP, 2);
    if(GPS.fix == 1) display.print("FIX");
    display.setCursor(0,12);
    display.setTextSize(3);
    display.print(millis() / ONE_SECOND - 2); 
    display.display();
  }

  anchor_lat = GPS.latitudeDegrees;
  anchor_lon = GPS.longitudeDegrees;
  boat_lat = GPS.latitudeDegrees;
  boat_lon = GPS.longitudeDegrees;
  beep(100); delay(100); beep(100);
  display.clearDisplay();

}

SIGNAL(TIMER0_COMPA_vect) {

  // GPS stuff
  char c = GPS.read();

  button_change = false;
  // Button stuff
  if (digitalRead(BUTTON_PIN) == PRESSED && previous == RELEASED && last_change + BOUNCETIME < millis()){
    previous = PRESSED;
    change = PRESSED;
    since_previous = millis() - last_change;
    last_change = millis();
    button_change = true;
    }
    
  if (digitalRead(BUTTON_PIN) == RELEASED && previous == PRESSED && last_change + BOUNCETIME < millis()){
    previous = RELEASED;
    change = RELEASED;
    since_previous = millis() - last_change;
    last_change = millis();
    button_change = true;
    }    
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // do often
  
  if(dist > alarm_radius && set){
    if(!previous_alarm) alarm_time = millis();
    previous_alarm = true;
    if((millis() - alarm_time) > ALARM_TIME_THRESHOLD * ONE_SECOND)   alarm = true;
    alarm_since = millis() - alarm_time;
  }else{
    alarm_since = 0;
    previous_alarm = false;
    alarm = false;
  }

    if(!GPS.fix && set){
    if(!previous_fixalarm) nofix_time = millis();
    previous_fixalarm = true;
    if((millis() - nofix_time) > GPS_TIME_THRESHOLD * ONE_SECOND)   nofix = true;
    nofix_since = millis() - nofix_time;
  }else{
    nofix_since = 0;
    previous_fixalarm = false;
    nofix = false;
  }
  
  if (GPS.newNMEAreceived()) {
  if (!GPS.parse(GPS.lastNMEA())) return;
  }

    if(button_change && change == RELEASED && since_previous < INCREMENT_TIME){
      alarm_radius = alarm_radius + RADIUS_INC; if (alarm_radius > MAX_RADIUS) alarm_radius = MIN_RADIUS;
      display.clearDisplay();
      beep(5);
    }

    if(button_change && change == RELEASED && since_previous > INCREMENT_TIME){
      if(GPS.fix){
      anchor_lat = GPS.latitudeDegrees;
      anchor_lon = GPS.longitudeDegrees;
      display.fillScreen(WHITE);
      display.setTextColor(BLACK);
      display.setCursor(0, 8);
      display.setTextSize(3);
      display.print(" SET ");
      display.display();
      beep(100);
      display.clearDisplay();
      set = true;
        
      }else{
      display.fillScreen(WHITE);
      display.setTextColor(BLACK);
      display.setCursor(10, 8);
      display.setTextSize(3);
      display.print("NO FIX");
      display.display();
      beep(500);
      display.clearDisplay();
      }

    }

  // do every second
  if (millis() - timer > ONE_SECOND) { 
    timer = millis();

    if(alarm || nofix) beep(100);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.fillRect(0, 0, 48, 32, BLACK);
    display.print("S"); display.print((int) GPS.satellites);
    display.print(" H"); display.print((float) GPS.HDOP, 2);
    if(GPS.fix == 1) display.print(" FIX ");
    if(set){
      display.setTextWrap(false);
      display.setTextSize(2);
      display.setCursor(0, 12);
      display.println(alarm_radius);
      display.setCursor(50,12);
      display.println(dist, 1);
      display.setCursor(80, 0);
      display.setTextSize(1);
      if(alarm_since > 0){
        display.print(" A");
        display.print(alarm_since / ONE_SECOND);
      }
      if(nofix_since > 0){
        display.print(" F");
        display.print(nofix_since / ONE_SECOND);
      }
    }
    display.display();
    
    if (GPS.fix) {
      boat_lat = GPS.latitudeDegrees;
      boat_lon = GPS.longitudeDegrees;
      dist = haversine(boat_lat, boat_lon, anchor_lat, anchor_lon);
    }
  }
}

float haversine(float lat1, float lon1, float lat2, float lon2){
  float r = (6356752.0 + 6378137.0) / 2.0; 
  lat1 = lat1 / DEG2RAD;
  lat2 = lat2 / DEG2RAD;
  lon1 = lon1 / DEG2RAD;
  lon2 = lon2 / DEG2RAD;
  
  return(2.0 * r * asin(sqrt(sin(pow((lat2 - lat1) / 2.0, 2.0)) +
                      cos(lat1) * cos(lat2) * sin(pow((lon2 - lon1) / 2.0, 2.0)))));
}

//θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )

void beep(int duration){
  digitalWrite(BEEP_PIN, HIGH);
  delay(duration);
  digitalWrite(BEEP_PIN, LOW);
}
