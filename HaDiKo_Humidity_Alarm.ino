#include <Adafruit_SSD1306.h>
#include <math.h>
#include <avr/sleep.h>
#include "DHT.h" //Grove Temperature And Humidity Sensor (by Seeed Studio), v1.0.0 (!)
#include "HumiditySensor.h"

#define ALARM_START_HUMIDITY 65.0f
#define ALARM_END_HUMIDITY 60.0f
#define MEASURE_CYCLE_SEC 10
#define VIGILANCE_DURATION_SEC 60 //should be a multiple of MEASURE_CYCLE_SEC
#define PIEZO_MAX_DURATION_SEC 10 //should be a multiple of MEASURE_CYCLE_SEC
#define PIEZO_OFF_DURATION_SEC 300 //should be a multiple of MEASURE_CYCLE_SEC
#define SCREEN_ON_DURATION_SEC 5

#define PIEZO_PIN 13
#define DISPLAY_VCC_PIN 17
#define DHT_VCC_PIN 16
#define PUSHBUTTON_PIN 2

#define APP_DEBUG

#ifdef APP_DEBUG
  #define DEBUG_PRINT(...)		Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)		Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif


Adafruit_SSD1306 display(128,32);
DHT dht(A1, DHT22);

bool piezoOn = false;

typedef enum { //don't change order! is being used as index
  STATE_NO_ALARM = 0,
  STATE_VIGILANCE,
  STATE_ALARM
} alarm_state_t;

const char * alarm_state_str[] = { //don't change order
  "NO_ALARM",
  "VIGILANCE",
  "ALARM"
};

typedef struct {
  float temp;
  float humid;
} weatherData_t;

bool readWeather(weatherData_t *weather) {
  float h;
  float t;
  for(int i=0;i<3;i++) {
    h = dht.readHumidity();
    t = dht.readTemperature();
    if(!(isnan(h) || h == 0.0f || isnan(t))) {
      break;
    } else if(i==3) {
      return false;
    }
    delay(200);
  }

  weather->humid = h;
  weather->temp = t;
  
  DEBUG_PRINT("Temperature:");
  DEBUG_PRINT(t, 1);
  DEBUG_PRINT(" Humidity:");
  DEBUG_PRINTLN(h, 1);

  return true;
}

void initScreen(void) {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.cp437(true);
  display.setRotation(0);
  //screenOn = true;
  //screenOnTime = millis();
}


ISR(WDT_vect) {}
ISR(INT0_vect) {EIMSK &= ~(1 << INT0);}

bool goToSleep(unsigned int sleepSec) {
  Serial.flush();
  digitalWrite(DHT_VCC_PIN, LOW);

  bool buttonPressed = false;

  if (sleepSec < 2) {sleepSec = 2;}
  sleepSec = ((sleepSec - 2) / 4) * 4;
  //sleepSec = (sleepSec > 4) ? sleepSec : 4;
  
  /*if(screenOn) {
    // can't sleep because screen needs millis() clock for shutdown timer
    unsigned long pauseStart = millis();
    while(1) {
      if(!digitalRead(PUSHBUTTON_PIN)) {
        buttonPressed = true;
        break;
      }
      if((millis() - pauseStart) >= (sleepSec * (unsigned long) 1000)) {
        break;
      }
    }
    if(buttonPressed) {
      DEBUG_PRINTLN(F("Button pressed!"));
      if(piezoOn) {
        digitalWrite(PIEZO_PIN, LOW);
        DEBUG_PRINTLN(F("Piezo stop by button. Alarm mute start."));
      }
      screenOnTime = millis();
    } else {
      if((millis() - screenOnTime) >= (SCREEN_ON_DURATION_SEC * 1000)) { //this is roll-over safe
        screenOn = false;
        digitalWrite(DISPLAY_VCC_PIN, LOW);
        TWCR = 0; //I2C connection reset
      }
    }
  } else {*/
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  //configure watchDog
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP3; //Watchdog-Dauer (4 sec)
  //configure INT0
  EICRA = (EICRA & ~((1<<ISC00) | (1<<ISC01))); //LOW level interrupt on INT0
  sei();

  //enable watchDog
  WDTCSR |= (1<<WDIE);
  //enable INT0
  EIMSK |= (1<<INT0);

  for(uint8_t i = 0; i<sleepSec/4; i++) {
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    if(!digitalRead(PUSHBUTTON_PIN)) {
      buttonPressed = true;
      break;
    }
    sleep_disable();
  }
  
  //disable watchDog
  WDTCSR &= ~(1<<WDIE);
  //disable INT0
  EIMSK &= ~(1 << INT0);
  
  if(buttonPressed) {
    DEBUG_PRINTLN(F("Button pressed!"));
    if(piezoOn) {
      digitalWrite(PIEZO_PIN, LOW);
      DEBUG_PRINTLN(F("Piezo stop by button. Alarm mute start."));
    }
    //digitalWrite(DISPLAY_VCC_PIN, HIGH);
    //delay(1500); //allow enough time for display to become operational. This gets added to the DHT's 1000ms of boot time to yield 2500ms before calling display.begin()
  }
  //}

  digitalWrite(DHT_VCC_PIN, HIGH);
  dht.begin();
  delay(1000); //DHT needs time to boot

  /*if(buttonPressed && !screenOn) {
    initScreen();
  }*/
  //returns bool that describes cause of wakeup
  //  true:  button was pressed
  //  false: sleep lasted sleepSec seconds and was woken up by Watchdog timer
  return buttonPressed;
}

bool displayWeather(weatherData_t *weather, bool alarm) {
  unsigned long pauseStart = millis();
  bool buttonPressed = false;

  //power up screen
  digitalWrite(DISPLAY_VCC_PIN, HIGH);
  delay(2500);
  initScreen();

  display.setCursor(0,0);
  display.clearDisplay();
  
  if(!alarm) {
    display.print(F("T "));
    display.print(weather->temp, 1);
    display.print(" ");
    display.write(0xF8);
    display.println("C");
  } else { //Instead of temperature, alarm is printed
    display.println(F("ALARM"));
  }
  display.print(F("H "));
  display.print(weather->humid, 1);
  display.print(" % ");
  display.display();

  while(1) {
    if(!buttonPressed) {
      if(!digitalRead(PUSHBUTTON_PIN)) {
        buttonPressed = true;
        DEBUG_PRINTLN(F("Button pressed!"));
        if(piezoOn) {
          digitalWrite(PIEZO_PIN, LOW);
          DEBUG_PRINTLN(F("Piezo stop by button. Alarm mute start."));
          piezoOn = false;
        }
      }
    }
    if((millis() - pauseStart) >= (SCREEN_ON_DURATION_SEC * (unsigned long) 1000)) { //this is roll-over safe
      break;
    }
  }
  digitalWrite(DISPLAY_VCC_PIN, LOW);
  TWCR = 0; //I2C connection reset

  if(buttonPressed) {
    return true;
  } else {
    return false;
  }
}


alarm_state_t stateNoAlarm (void) {
  weatherData_t currentWeather;
  bool buttonPressed;
  while(1) {
    buttonPressed = goToSleep(MEASURE_CYCLE_SEC);
    if(readWeather(&currentWeather)) {
      if(currentWeather.humid >= ALARM_START_HUMIDITY) {
        displayWeather(&currentWeather, true);
        return STATE_VIGILANCE;
      }
      if(buttonPressed) {
        displayWeather(&currentWeather, false);
      }
    }
  }
}

alarm_state_t stateVigilance (void) {
  weatherData_t currentWeather;
  const int j = VIGILANCE_DURATION_SEC / MEASURE_CYCLE_SEC;
  bool buttonPressed;
  for(int i=0;i<j;i++) {
    buttonPressed = goToSleep(MEASURE_CYCLE_SEC);
    if(readWeather(&currentWeather)) {
      if(currentWeather.humid < ALARM_START_HUMIDITY) {
        displayWeather(&currentWeather, false);
        return STATE_NO_ALARM;
      }
      if(buttonPressed) { //button was pressed, don't count this
        i--;
        displayWeather(&currentWeather, true);
      }
    }
  }
  //Alarm!
  
  piezoOn = true;
  digitalWrite(PIEZO_PIN, HIGH);
  DEBUG_PRINTLN(F("Piezo start"));
  displayWeather(&currentWeather, true);
    piezoOn = false;
  }
  
  return STATE_ALARM;
}

alarm_state_t stateAlarm (void) {
  weatherData_t currentWeather;
  
  while(1) {
    int j;
    if(piezoOn) {
      j = PIEZO_MAX_DURATION_SEC / MEASURE_CYCLE_SEC;
    } else {
      j = PIEZO_OFF_DURATION_SEC / MEASURE_CYCLE_SEC;
    }

    int i;
    bool buttonPressed;
    for(i=j;i>0;i--) {
      buttonPressed = goToSleep(MEASURE_CYCLE_SEC);
      if(buttonPressed) { //button was pressed
        if(piezoOn) {
          piezoOn = false;
          break;
        }
        i++;
      }
      if(readWeather(&currentWeather)) {
        if(currentWeather.humid <= ALARM_END_HUMIDITY) {
          displayWeather(&currentWeather, false);
          if(piezoOn) {
            digitalWrite(PIEZO_PIN, LOW);
            DEBUG_PRINTLN(F("Piezo stop by end of alarm"));
            piezoOn = false;
          }
          return STATE_NO_ALARM;
        }
        displayWeather(&currentWeather, true);
      }
    }

    if(i==0) { //button was NOT pressed = TIMEOUT
      if(piezoOn) {
        digitalWrite(PIEZO_PIN, LOW);
        DEBUG_PRINTLN(F("Piezo stop by timeout. Alarm mute start."));
        piezoOn = false;
      } else {
        digitalWrite(PIEZO_PIN, HIGH);
        DEBUG_PRINTLN(F("Alarm mute stop by timeout. Piezo start."));
        piezoOn = true;
      }
    }
  }
}

typedef alarm_state_t (*state_run_t) (void);

state_run_t state_run[] = { //don't change order! index is alarm_state_t
  stateNoAlarm,
  stateVigilance,
  stateAlarm
};

void setup() {
  // put your setup code here, to run once:
#ifdef APP_DEBUG
  Serial.begin(9600);
#endif
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(DISPLAY_VCC_PIN, OUTPUT);
  pinMode(DHT_VCC_PIN, OUTPUT);
  
  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP);

  ADCSRA = 0x00; //turn ADC off
  ACSR = (1 << ACD); //turn AC off
}

void loop() {
  // put your main code here, to run repeatedly:
  alarm_state_t currentState = STATE_NO_ALARM;
  while(1) {
    DEBUG_PRINT(F("Entering state "));
    DEBUG_PRINTLN(alarm_state_str[currentState]);
    currentState = state_run[currentState]();
  }
}
