#include <Adafruit_SSD1306.h>
#include <math.h>
#include <avr/sleep.h>
#include "DHT.h" //Grove Temperature And Humidity Sensor (by Seeed Studio), v1.0.0 (!)


#define ALARM_START_HUMIDITY 65.0f
#define ALARM_END_HUMIDITY 60.0f
#define MEASURE_CYCLE_SEC 20
#define VIGILANCE_DURATION_SEC 60 //should be a multiple of MEASURE_CYCLE_SEC
#define PIEZO_MAX_DURATION_SEC 20 //should be a multiple of MEASURE_CYCLE_SEC
#define PIEZO_OFF_DURATION_SEC 300 //should be a multiple of MEASURE_CYCLE_SEC
#define SCREEN_ON_DURATION_SEC 10

#define PIEZO_PIN 13
#define DISPLAY_VCC_PIN 17
#define DHT_VCC_PIN 16
#define PUSHBUTTON_PIN 2

//#define APP_DEBUG

#ifdef APP_DEBUG
  #define DEBUG_PRINT(...)		Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)		Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif


Adafruit_SSD1306 display(128,32);
DHT dht(A1, DHT22);

typedef enum { //don't change order! is being used as index
  STATE_NO_ALARM = 0,
  STATE_VIGILANCE,
  STATE_ALARM_PIEZO,
  STATE_ALARM_SILENT
} alarm_state_t;

const char * alarm_state_str[] = { //don't change order
  "NO_ALARM",
  "VIGILANCE",
  "ALARM_PIEZO",
  "ALARM_SILENT"
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

void onOffScreen(bool onOff) {
  if(onOff) {
    pinMode(DISPLAY_VCC_PIN, OUTPUT);
    digitalWrite(DISPLAY_VCC_PIN, LOW); //inverted logic because PNP
    delay(2500);
  } else {
    pinMode(DISPLAY_VCC_PIN, INPUT);
    //digitalWrite(DISPLAY_VCC_PIN, HIGH);
    TWCR = 0; //I2C connection reset
    pinMode(18, INPUT);
    digitalWrite(18, LOW);
    pinMode(19, INPUT);
    digitalWrite(19, LOW);
  }
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

void onOffPiezo(bool onOff) {
  if(onOff) {
    digitalWrite(PIEZO_PIN, HIGH);
  } else {
    digitalWrite(PIEZO_PIN, LOW);
  }
}


ISR(WDT_vect) {}
ISR(INT0_vect) {EIMSK &= ~(1 << INT0);}

bool goToSleep(unsigned int sleepSec) {
  Serial.flush();
  digitalWrite(DHT_VCC_PIN, LOW);
  pinMode(DHT_VCC_PIN, INPUT);

  bool buttonPressed = false;

  if (sleepSec < 2) {sleepSec = 2;} //avoid roll-over in next line
  sleepSec = ((sleepSec - 2) / 4) * 4;

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
    onOffPiezo(false);
  }
  
  pinMode(DHT_VCC_PIN, OUTPUT);
  digitalWrite(DHT_VCC_PIN, HIGH);
  dht.begin();
  delay(1000); //DHT needs time to boot

  //returns bool that describes cause of wakeup
  //  true:  button was pressed
  //  false: sleep lasted sleepSec seconds and was woken up by Watchdog timer
  return buttonPressed;
}

bool displayWeather(weatherData_t *weather, bool alarm) {
  bool buttonPressed = false;

  //power up screen
  onOffScreen(true);
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

  unsigned long displayStart = millis();

  while(1) {
    if(!buttonPressed) {
      if(!digitalRead(PUSHBUTTON_PIN)) {
        buttonPressed = true;
        DEBUG_PRINTLN(F("Button pressed!"));
        onOffPiezo(false);
      }
    }
    if((millis() - displayStart) >= (SCREEN_ON_DURATION_SEC * (unsigned long) 1000)) { //this is roll-over safe
      break;
    }
  }
  onOffScreen(false);

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
        if(buttonPressed) {
          displayWeather(&currentWeather, true);
        }
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
        if(buttonPressed) {
          displayWeather(&currentWeather, false);
        }
        return STATE_NO_ALARM;
      }
      if(buttonPressed) { //button was pressed, don't count this
        i--;
        displayWeather(&currentWeather, true);
      }
    } else { //read fail
      return STATE_NO_ALARM;
    }
  }
  
  //Alarm!
  return STATE_ALARM_PIEZO;
}

alarm_state_t stateAlarmPiezo (void) {
  weatherData_t currentWeather;

  if(!readWeather(&currentWeather)) {
    return STATE_NO_ALARM;
  }
  if(currentWeather.humid < ALARM_START_HUMIDITY) {
    return STATE_NO_ALARM;
  }
  
  onOffPiezo(true);

  if(displayWeather(&currentWeather, true)) { //button pressed, piezo already off
    return STATE_ALARM_SILENT;
  }

  int i;
  const int j = (PIEZO_MAX_DURATION_SEC - SCREEN_ON_DURATION_SEC) / MEASURE_CYCLE_SEC;
  bool buttonPressed;
 
  for(i=j;i>0;i--) {
    buttonPressed = goToSleep(MEASURE_CYCLE_SEC);
    if(!readWeather(&currentWeather)) {
      return STATE_NO_ALARM;
    }
    if(currentWeather.humid <= ALARM_END_HUMIDITY) {
      if(buttonPressed) {
        displayWeather(&currentWeather, false);
      }
      return STATE_NO_ALARM;
    }
    if(buttonPressed) {
      displayWeather(&currentWeather, true);
      return STATE_ALARM_SILENT;
    }
  } 
  
  return STATE_ALARM_SILENT;
}


alarm_state_t stateAlarmSilent(void) {
  weatherData_t currentWeather;
  int i;
  const int j = PIEZO_OFF_DURATION_SEC / MEASURE_CYCLE_SEC;
  bool buttonPressed;
  
  for(i=j;i>0;i--) {
    buttonPressed = goToSleep(MEASURE_CYCLE_SEC);
    if(!readWeather(&currentWeather)) {
      return STATE_NO_ALARM;
    }
    if(currentWeather.humid <= ALARM_END_HUMIDITY) {
      if(buttonPressed) {
        displayWeather(&currentWeather, false);
      }
      return STATE_NO_ALARM;
    }

    if(buttonPressed) {
      displayWeather(&currentWeather, true);
      i++;
    }
  }

  return STATE_ALARM_PIEZO;
}


typedef alarm_state_t (*state_run_t) (void);

state_run_t state_run[] = { //don't change order! index is alarm_state_t
  stateNoAlarm,
  stateVigilance,
  stateAlarmPiezo,
  stateAlarmSilent
};

void setup() {
  // put your setup code here, to run once:
#ifdef APP_DEBUG
  Serial.begin(9600);
#endif
  pinMode(PIEZO_PIN, OUTPUT);
  //digitalWrite(DISPLAY_VCC_PIN, HIGH); //Pin is high directly when declaring output
  //pinMode(DISPLAY_VCC_PIN, OUTPUT);
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
