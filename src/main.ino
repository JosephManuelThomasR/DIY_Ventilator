// DIY Ventilator - Combined sketch for NodeMCU + JHD204A (20x4) LCD
// WARNING: Educational prototype only - not medical grade.
// Requires libraries: LiquidCrystal, DHT, Adafruit_MAX30100 (or similar)

#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_MAX30100.h> // install via Library Manager if available

// ----------------------------- PIN CONFIG ----------------------------------
// NodeMCU pin names (Dx) map to GPIO numbers in Arduino core for ESP8266
// You can change these if you run into boot problems (see notes below).

// Buttons & LEDs
#define P_SETTING_BUTTON D7   // Setting / Age / Mode change (GPIO13)
#define P_START_BUTTON   D8   // Start / Stop ventilation (GPIO15) -- must be pulled LOW at boot; leave as is
#define P_RED_LED        D5   // Status / warning LED (GPIO14)
#define P_GREEN_LED      D6   // Run LED (GPIO12)

// Servo and sensors
#define P_SERVO          D4   // Servo signal (GPIO2) — avoid pulling low at boot
#define DHT_PIN          D3   // DHT11 data (GPIO0) - must be pulled HIGH at boot, DHT has internal pull-ups
#define POT_PIN          A0   // Single analog potentiometer (middle wiper to A0)

// LCD (JHD204A parallel, 4-bit mode)
// RS, EN, D4, D5, D6, D7
#define LCD_RS           D1   // GPIO5
#define LCD_EN           D2   // GPIO4
#define LCD_D4           D3   // GPIO0  (note: D3 is a boot pin; see notes)
#define LCD_D5           D4   // GPIO2  (boot pin)
#define LCD_D6           D5   // GPIO14
#define LCD_D7           D6   // GPIO12

// MAX30100 (I2C) uses default SDA / SCL pins (D2/D1 or as per board). Wire library will handle.
// For NodeMCU: SDA = D2 (GPIO4), SCL = D1 (GPIO5) if using those pins; Wire uses default SCL/SDA for board.

// ----------------------------- HARDWARE CONSTANTS ---------------------------
#define DHTTYPE DHT11
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Age group indices
#define AGE_ADULT 0
#define AGE_CHILD 1
#define AGE_INFANT 2

// UI states for manual adjust
enum AdjustStep { ADJ_NONE = 0, ADJ_VOLUME = 1, ADJ_PRESSURE = 2, ADJ_RATE = 3 };

// Nominal values: nom_vals[param][age]
// params: 0=Volume (mL), 1=Pressure (/10 arbitrary unit), 2=BPM (breaths per minute)
const uint16_t nom_vals[3][3] = {
  {500, 300, 100}, // volume nominals per age (adult, child, infant) — note: ordering will be adapted below
  {5,   5,   5  }, // pressure base (arbitrary)
  {12,  25,  50 }  // breath rate nominals (adult=12, child=25, infant=50)
};
// Per-step increment when using pot (scaling)
const uint8_t val_inc[3][3] = {
  {10, 5, 1}, // volume increments per pot unit (scaled)
  {1,  1, 1}, // pressure increment
  {1,  1, 2}  // rate increment
};

// base delays used in breath cycle timing (ms; inspired by original code)
const uint16_t base_delays[3] = {950, 550, 200}; // adult, child, infant

const char *age_titles[3] = {"Adult", "Child", "Infant"};
const char *unit_titles[3] = {" mL", "/10", " br/m"};

// ----------------------------- GLOBAL OBJECTS -------------------------------
DHT dht(DHT_PIN, DHTTYPE);
Adafruit_MAX30100 max30100;
Servo ambuServo;
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// UI & ventilator state
volatile bool ventilating = false;
volatile bool compression_state = false;

uint8_t age_state = AGE_ADULT;
AdjustStep adjust_mode = ADJ_NONE;

unsigned long squeeze_time = 0;
unsigned long release_time = 0;

bool can_read_age_button = true;
unsigned long age_button_time = 0;

bool can_read_start_button = true;
unsigned long start_button_time = 0;

unsigned long last_display_update = 0;
const unsigned long DISPLAY_UPDATE_MS = 500;

// Pot value (single ADC used to edit selected param)
int raw_pot = 0;         // 0..1023
int8_t pot_mapped = 0;   // -5..+5 mapping
const int POT_MAP_RADIUS = 5;

// LCD helpers
const char clear_value[] = "        ";

// ----------------------------- UTILITIES -----------------------------------
int map_pot(int raw) {
  // Map raw 0..1023 to -POT_MAP_RADIUS..POT_MAP_RADIUS
  return map(raw, 0, 1023, -POT_MAP_RADIUS, POT_MAP_RADIUS);
}

uint16_t compute_param_value(uint8_t paramIndex, uint8_t ageIdx, int potOffset) {
  // Compute a final integer value for a parameter given nominal, increment, and pot offset
  int base = nom_vals[paramIndex][ageIdx];
  int inc  = val_inc[paramIndex][ageIdx];
  int val = base + inc * potOffset;
  if (paramIndex == 2) { // breath rate sanity
    if (val < 5) val = 5;
    if (val > 200) val = 200;
  }
  return (uint16_t)val;
}

// Convert desired tidal volume (mL) to servo angle (approx polynomial from original sketch)
// Returns servo angle (0..180)
int vol_to_angle(uint16_t vol) {
  // Use original polynomial and clamp
  double v = (double)vol;
  double ang = 4.89427e-7 * pow(v, 3) - 8.40105e-4 * pow(v, 2) + 0.64294 * v + 28.072327;
  if (ang < 0) ang = 0;
  if (ang > 180) ang = 180;
  return (int)round(ang);
}

uint32_t ms_per_breath(uint16_t breaths_per_min) {
  // ms per breath = 60,000 / bpm
  if (breaths_per_min == 0) return 60000;
  return (uint32_t)round(60000.0 / breaths_per_min);
}

// Drive servo smoothly from its current angle to desiredAngle (deg)
void drive_servo_smooth(int desiredAngle, unsigned int stepDelayMs = 6) {
  int cur = ambuServo.read(); // 0..180
  if (cur == desiredAngle) return;
  if (cur < desiredAngle) {
    for (int a = cur; a <= desiredAngle; ++a) {
      ambuServo.write(a);
      delay(stepDelayMs);
    }
  } else {
    for (int a = cur; a >= desiredAngle; --a) {
      ambuServo.write(a);
      delay(stepDelayMs);
    }
  }
}

// ----------------------------- DISPLAY -------------------------------------
void lcd_startup() {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("YOUR");
  lcd.setCursor(0, 1);
  lcd.print("Automatic Inhalation");
  lcd.setCursor(4, 3);
  lcd.print("VENTILATOR");
}

// Update the main parameter display (age, vol, pres, rate)
void lcd_update_display(uint8_t age, int potOffset, AdjustStep adjStep) {
  // read current nominal values
  uint16_t vol = compute_param_value(0, age, (adjStep == ADJ_VOLUME) ? potOffset : 0);
  uint16_t pres = compute_param_value(1, age, (adjStep == ADJ_PRESSURE) ? potOffset : 0);
  uint16_t rate = compute_param_value(2, age, (adjStep == ADJ_RATE) ? potOffset : 0);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Age Group:");
  lcd.setCursor(0,1); lcd.print("Volume:");
  lcd.setCursor(0,2); lcd.print("Pressure:");
  lcd.setCursor(0,3); lcd.print("Breath Rate:");

  lcd.setCursor(12,0);
  lcd.print(clear_value);
  lcd.setCursor(12,0);
  lcd.print(age_titles[age]);

  lcd.setCursor(12,1);
  lcd.print(clear_value);
  lcd.setCursor(12,1);
  lcd.print(vol);
  lcd.print(unit_titles[0]);

  lcd.setCursor(12,2);
  lcd.print(clear_value);
  lcd.setCursor(12,2);
  lcd.print(pres);
  lcd.print(unit_titles[1]);

  lcd.setCursor(12,3);
  lcd.print(clear_value);
  lcd.setCursor(12,3);
  lcd.print(rate);
  lcd.print(unit_titles[2]);

  // Indicate adjust mode on top right
  if (adjStep != ADJ_NONE) {
    lcd.setCursor(0,0);
    lcd.print("EDIT:");
    switch (adjStep) {
      case ADJ_VOLUME: lcd.print("Vol"); break;
      case ADJ_PRESSURE: lcd.print("Pres"); break;
      case ADJ_RATE: lcd.print("Rate"); break;
      default: break;
    }
  }
}

// ----------------------------- BUTTON HANDLING ------------------------------
void check_buttons_and_state() {
  // Age/setting button short press cycles age.
  // Long-press (hold > 800 ms) enters adjust mode. In adjust mode, repeated short press cycles which param to adjust.
  static unsigned long btnDownTime = 0;
  static bool btnWasDown = false;

  bool settingPressed = (digitalRead(P_SETTING_BUTTON) == LOW); // buttons wired to GND (INPUT_PULLUP)
  if (settingPressed && !btnWasDown) {
    // just pressed
    btnDownTime = millis();
    btnWasDown = true;
  } else if (!settingPressed && btnWasDown) {
    // released
    unsigned long held = millis() - btnDownTime;
    btnWasDown = false;
    if (held < 800) {
      // short press
      if (adjust_mode == ADJ_NONE) {
        // cycle age
        age_state = (age_state + 1) % 3;
      } else {
        // in adjust mode: cycle selected adjust step
        if (adjust_mode == ADJ_VOLUME) adjust_mode = ADJ_PRESSURE;
        else if (adjust_mode == ADJ_PRESSURE) adjust_mode = ADJ_RATE;
        else adjust_mode = ADJ_VOLUME;
      }
    } else {
      // long press
      if (adjust_mode == ADJ_NONE) {
        adjust_mode = ADJ_VOLUME; // enter adjust mode with Volume selected
      } else {
        adjust_mode = ADJ_NONE; // exit adjust mode
      }
    }
  }

  // Start button toggles ventilation (short press)
  static bool startWasDown = false;
  bool startPressed = (digitalRead(P_START_BUTTON) == LOW);
  if (startPressed && !startWasDown) {
    startWasDown = true;
  } else if (!startPressed && startWasDown) {
    startWasDown = false;
    ventilating = !ventilating;
    digitalWrite(P_GREEN_LED, ventilating ? HIGH : LOW);
    digitalWrite(P_RED_LED, ventilating ? LOW : HIGH);
    // reset timing to avoid accidental immediate squeeze
    squeeze_time = 0;
    release_time = millis();
  }
}

// ----------------------------- SENSOR READS --------------------------------
float read_temperature() {
  float t = dht.readTemperature();
  if (isnan(t)) return -1000;
  return t;
}
float read_humidity() {
  float h = dht.readHumidity();
  if (isnan(h)) return -1000;
  return h;
}

// MAX30100 wrapper (non-blocking)
float lastHR = 0.0, lastSpO2 = 0.0;
void read_pulseox() {
  // depends on Adafruit_MAX30100 library API: adapt if needed
  // many MAX30100 libs use update() then getHeartRate()/getSpO2()
  max30100.update();
  float hr = max30100.getHeartRate();
  float sp = max30100.getSpO2();
  if (hr > 0) lastHR = hr;
  if (sp > 0) lastSpO2 = sp;
}

// ----------------------------- VENTILATOR LOGIC -----------------------------
void ventilator_loop_once(int potOffset) {
  // potOffset used only when adjust_mode == ADJ_NONE? Actually we use potOffset for immediate selected adjustment,
  // but for ventilation we compute current target values based on base + potOffset only for Volume when user adjusts.
  uint16_t vol = compute_param_value(0, age_state, (adjust_mode == ADJ_VOLUME) ? potOffset : 0);
  uint16_t pres = compute_param_value(1, age_state, (adjust_mode == ADJ_PRESSURE) ? potOffset : 0);
  uint16_t rate = compute_param_value(2, age_state, (adjust_mode == ADJ_RATE) ? potOffset : 0);

  uint32_t msPerBreath = ms_per_breath(rate);
  // define inhale (squeeze) duration and exhale (release) duration heuristically
  // We'll use base_delays[age] for squeeze time, remainder for exhale
  uint16_t squeeze_ms = base_delays[age_state];
  if (squeeze_ms > (int)msPerBreath - 50) squeeze_ms = (msPerBreath / 3); // fallback
  uint32_t exhale_ms = (msPerBreath > squeeze_ms) ? (msPerBreath - squeeze_ms) : (msPerBreath);

  // Squeeze action: move servo to target angle and hold for squeeze_ms,
  // then return to start position and hold for exhale_ms.
  // Start position = 0 degrees; target angle = vol_to_angle(vol)
  static bool inSqueeze = false;
  static unsigned long squeezeStartAt = 0;
  static unsigned long releaseStartAt = 0;

  int startAngle = 0;
  int targetAngle = vol_to_angle(vol);

  if (!inSqueeze) {
    // time to start squeeze if enough time passed
    if (millis() - releaseStartAt >= exhale_ms) {
      // begin squeeze
      inSqueeze = true;
      drive_servo_smooth(targetAngle, 3); // fast sweep to squeeze
      squeezeStartAt = millis();
    }
  } else {
    // currently squeezing
    if (millis() - squeezeStartAt >= squeeze_ms) {
      // release
      drive_servo_smooth(startAngle, 3);
      inSqueeze = false;
      releaseStartAt = millis();
    }
  }
}

// ----------------------------- SETUP & LOOP --------------------------------
void safePinModeInit() {
  // set INPUT_PULLUP for buttons and safe initial states for critical pins
  pinMode(P_SETTING_BUTTON, INPUT_PULLUP);
  pinMode(P_START_BUTTON, INPUT_PULLUP);
  pinMode(P_RED_LED, OUTPUT);
  pinMode(P_GREEN_LED, OUTPUT);

  // set LEDs off initially
  digitalWrite(P_RED_LED, LOW);   // RED off
  digitalWrite(P_GREEN_LED, LOW); // GREEN off
}

void setup() {
  Serial.begin(115200);
  delay(50);

  safePinModeInit();

  // Servo attach and set to start
  ambuServo.attach(P_SERVO);
  ambuServo.write(0);

  // LCD init
  lcd.begin(20, 4);
  lcd_startup();
  delay(1500);

  // DHT init
  dht.begin();

  // MAX30100 init (I2C)
  Wire.begin(); // uses default SDA/SCL
  if (!max30100.begin()) {
    Serial.println("MAX30100 init failed (optional).");
    // we continue — user can still use ventilator without pulseox
  } else {
    max30100.setMode(MAX30100_MODE_SPO2_HR);
    Serial.println("MAX30100 ready.");
  }

  // Initialize timing & display
  last_display_update = millis();
  lcd_update_display(age_state, 0, ADJ_NONE);
}

void loop() {
  // read buttons -> may change age or enter adjust mode
  check_buttons_and_state();

  // read pot
  raw_pot = analogRead(POT_PIN);
  pot_mapped = map_pot(raw_pot);

  // update sensors periodically
  static unsigned long lastSensorMs = 0;
  if (millis() - lastSensorMs > 1000) {
    lastSensorMs = millis();
    float T = read_temperature();
    float H = read_humidity();
    read_pulseox();
    // optionally print to serial
    Serial.print("T=");
    Serial.print(T);
    Serial.print(" H=");
    Serial.print(H);
    Serial.print(" HR=");
    Serial.print(lastHR);
    Serial.print(" SpO2=");
    Serial.println(lastSpO2);
  }

  // Update LCD regularly
  if (millis() - last_display_update > DISPLAY_UPDATE_MS) {
    last_display_update = millis();
    lcd_update_display(age_state, pot_mapped, adjust_mode);
  }

  // If ventilating, perform ventilator step(s)
  if (ventilating) {
    ventilator_loop_once(pot_mapped);
  }

  // small loop delay
  delay(10);
}
