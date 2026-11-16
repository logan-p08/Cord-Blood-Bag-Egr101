/*
  Arduino Nano (ATmega328P) - two independently controlled heating plates

  BUTTONS: use normally-open MOMENTARY pushbuttons, not latching switches.
    MODE/SELECT: D2 to GND. Short press/release toggles 80% / 20%.
      Hold for 1000 ms to enter settings from PREHEATING or READY.
      Each separate hold advances: pads -> 80% bag -> 20% bag -> save/exit.
      In settings, short MODE press/release decreases the target by 0.5 C.
    START: D3 to GND. In settings, increases the target by 0.5 C per press.
      Outside settings, starts thawing only when READY.
    RESET:  Nano RST to GND. Hardware reset; no digital input pin needed.
            Use the NANO's RST, not the multiplexer RST.
    D2/D3 use internal pull-ups. A held button produces only one action.
    Buttons held at boot must be released before they can act.

  WIRING (all three temperature sensors are MLX90614, address 0x5A):
    A4 -> mux SDA; A5 -> mux SCL (upstream control bus).
    OLED SDA/SCL -> mux SD0/SC0, address 0x3C.
    Plate 1 / bottom sensor SDA/SCL -> mux SD7/SC7.
    Plate 2 / top sensor SDA/SCL -> mux SD3/SC3.
    Bag/product sensor SDA/SCL -> mux SD6/SC6.
    PCA9548A / HW-617 address 0x70; select only one branch at a time.
    Select channel 0 for OLED initialization and every screen transfer;
    deselect all branches after each sensor read or screen operation.
    D9 -> bottom MOSFET gate circuit; D10 -> top MOSFET gate circuit.
    Each MOSFET switches ONLY its own plate's pads. Use a suitable logic-level
    MOSFET/driver, gate resistor and gate-to-source pulldown on EACH channel.
    D6 green LED, D7 red LED. All control circuits share a common GND.
    Route heater return currents directly to the heater supply return;
    do not route them through Nano/sensor ground wires or a breadboard.
    Use supplies and I2C pull-up levels appropriate to the actual modules.

  COMPLETION SOUND (D8, previously unused):
    SPKM.15.8.A is a passive 15 mm, 8 ohm, 0.5 W speaker.
    Do NOT connect this 8 ohm speaker directly between a GPIO and GND.
    For a quiet beep, wire this series circuit:
      D8 -> 330 ohm resistor -> 100 uF capacitor (+)
      capacitor (-) -> speaker (+); speaker (-) -> Nano GND.
    Use a capacitor rated at least 10 V and a resistor rated at least 1/4 W.
    The resistor limits GPIO current; the capacitor blocks DC in the speaker.
    For more volume, use a suitable audio amplifier with input attenuation
    and keep speaker power within its rating; do not remove the resistor
    from the direct GPIO circuit. tone() uses Timer2 on this Nano, leaving
    D9 and D10 heater PWM (Timer1) available. D3 is only a button input here.

  SEQUENCE:
    Power on / RESET -> load saved targets, PREHEATING, red LED.
      Default targets: pads 37.1 C, 80% bag 30.0 C, 20% bag 26.0 C.
      Both heaters stay OFF until three valid pairs of plate readings pass.
    Each plate latches preheated within [pad target - 0.6, target + 0.4] C
      (36.5..37.5 C at the default 37.1 C target),
      once the initial sensor checks pass. READY / green LED requires BOTH
      plates to have reached that band during this boot. They do not have
      to reach the band in the same sample. Leaving the band does not clear
      either latch, so a cold bag does not block START.
      Each PID uses only its own plate's sensor and independent I/D history.
      Overtemperature and other faults still latch FAULT and block START.
      SELECT may be used during PREHEATING or READY; default is 80%.
      The bag sensor is not read until START.
    START -> THAWING, red LED, freeze selected mode and enable all 3 sensors.
      Both heaters pause until three valid sets of all 3 readings arrive.
      Each learned integral is retained during this sensor-check pause.
      Any valid bag reading >= goal finishes immediately, including
      the first reading after START if the product is already at its goal.
    FINISHED -> BOTH PWMs 0, green LED, "Finished thawing", mode and goal shown.
      One 2 kHz, 1-second beep on D8; sensor polling continues during it.
      Cooling below the goal NEVER restarts heating. SELECT/START are ignored.
      All three temperatures continue to be read for serial diagnostics only.
    RESET or a real Nano power cycle starts a NEW preheat with default 80%.

  Sensor/mux faults, EITHER plate >= saved pad target + 8 C, OLED faults, or missed sampling
  (>1 s) latch FAULT: BOTH heaters OFF, red LED, reset required after fixing
  the cause. The fault identifies the affected plate or bag sensor.
  Unlike the previous sketch, a fault or finished cycle does not auto-resume.
  Reset does not permit heating until fresh sensor checks pass.

  SETTINGS: heaters stay OFF while editing; plate fault checks continue.
    Pads adjustable 36.0..50.0 C; bag targets 0.0..35.0 C. Steps are 0.5 C
    from the current value (e.g. 37.1 -> 37.6). Steps clamp at the limits.
    Exit the final page to save all three targets. Reset/power loss before
    saving discards edits. EEPROM bytes 0..31 are reserved for two records;
    checksum and commit marker retain the previous record if a save is cut off.
    Editing is locked during THAWING, FINISHED and FAULT. Changing the pad
    target clears both READY latches; bag-only changes preserve them.
    Every menu exit requires three fresh pairs of readings before heating.

  SUPPLY/USB: automatic preheat begins when the NANO boots. This assumes the
  supply's power button powers the Nano as well as the pads. If USB keeps
  the Nano powered, use RESET to start a new cycle. Opening Serial Monitor
  can reset a classic Nano, which also starts a new preheat cycle.

  Serial Monitor: 9600 baud. Sensor polling: 500 ms. Display: 500 ms or events.
  Install Adafruit MLX90614, SSD1306, GFX and BusIO. Use Arduino AVR Boards
  with Wire timeout support. No additional button or PID library is needed.

  Bench-test both modes, button wiring and shutdown before powering heaters.
  "80%" / "20%" are user-specified preset names, not measured thaw fractions.
  An IR threshold does not establish the temperature throughout the product.
  Software cutoff cannot stop residual heat or a shorted MOSFET: retain an
  independent thermal cutoff. This prototype is not clinically validated.
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// -------------------- Settings --------------------
constexpr uint16_t CUTOFF_ABOVE_PAD_TENTHS = 80; // Either plate trips BOTH OFF
constexpr uint16_t DEFAULT_PAD_TENTHS = 371;
constexpr uint16_t DEFAULT_BAG80_TENTHS = 300;
constexpr uint16_t DEFAULT_BAG20_TENTHS = 260;
constexpr uint16_t PAD_MIN_TENTHS = 360;
constexpr uint16_t PAD_MAX_TENTHS = 500;
constexpr uint16_t BAG_MAX_TENTHS = 350;
constexpr uint16_t TARGET_STEP_TENTHS = 5;
constexpr uint32_t MODE_HOLD_MS = 1000;

// All displayed/serial temperatures, PID control, READY checks, bag goals
// and dynamic cutoffs use sensor readings directly, with no added offsets.

// More responsive starting gains; validate with the actual pads and load.
// Higher P gives more power below target; higher I corrects persistent droop.
// D adds braking as the pads warm. These values cannot guarantee no overshoot.
constexpr float Kp = 80.0f;              // PWM units / C (was 40)
constexpr float Ki = 2.0f;               // PWM units / (C*s) (was 0.10)
constexpr float Kd = 10.0f;              // PWM units * s / C (was 6)
constexpr float D_FILTER_SECONDS = 1.0f;
constexpr uint8_t MAX_PWM = 255;

constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t SAMPLE_MS = 500;
constexpr uint32_t DISPLAY_MS = 500;
constexpr uint32_t SERIAL_MS = 500;
constexpr uint32_t MAX_SAMPLE_GAP_MS = 1000;
constexpr uint8_t GOOD_READS_REQUIRED = 3;
constexpr bool SERIAL_DEBUG = true;

constexpr uint8_t SELECT_BUTTON_PIN = 2;
constexpr uint8_t START_BUTTON_PIN = 3;
constexpr uint8_t HEATER_PIN = 9;
constexpr uint8_t SECOND_HEATER_PIN = 10;
constexpr uint8_t GREEN_LED_PIN = 6;
constexpr uint8_t RED_LED_PIN = 7;
constexpr uint8_t BUZZER_PIN = 8;
constexpr unsigned int FINISH_BEEP_HZ = 2000;
constexpr unsigned long FINISH_BEEP_MS = 1000UL;
constexpr uint8_t OLED_CHANNEL = 0;
constexpr uint8_t PRIMARY_CHANNEL = 7;       // Bottom plate, PWM D9
constexpr uint8_t SECONDARY_CHANNEL = 6;     // Bag/product
constexpr uint8_t SECOND_PLATE_CHANNEL = 3;  // Top plate, PWM D10
constexpr uint8_t MUX_ADDRESS = 0x70;
constexpr uint8_t MLX_ADDRESS = 0x5A;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint32_t I2C_HZ = 100000UL;
constexpr uint32_t I2C_TIMEOUT_US = 25000UL;

static_assert(4 < CUTOFF_ABOVE_PAD_TENTHS,
              "READY band must remain below safety cutoff");
static_assert(PRIMARY_CHANNEL < 8 && SECONDARY_CHANNEL < 8 &&
              SECOND_PLATE_CHANNEL < 8 && OLED_CHANNEL < 8 &&
              PRIMARY_CHANNEL != SECONDARY_CHANNEL &&
              PRIMARY_CHANNEL != SECOND_PLATE_CHANNEL &&
              SECONDARY_CHANNEL != SECOND_PLATE_CHANNEL &&
              OLED_CHANNEL != PRIMARY_CHANNEL &&
              OLED_CHANNEL != SECONDARY_CHANNEL &&
              OLED_CHANNEL != SECOND_PLATE_CHANNEL, "Invalid mux channels");
static_assert(HEATER_PIN == 9 && SECOND_HEATER_PIN == 10,
              "Nano heater outputs must use the assigned Timer1 PWM pins");
static_assert(GOOD_READS_REQUIRED > 0, "Require fresh valid samples");

enum class Stage : uint8_t { PREHEATING, READY, THAWING, FINISHED, FAULT };
enum class Menu : uint8_t { OFF, PADS, BAG80, BAG20 };
enum class ModeEvent : uint8_t { NONE, SHORT_PRESS, LONG_HOLD };
enum class Fault : uint8_t { NONE, PRIMARY_SENSOR, SECONDARY_SENSOR,
                             SECOND_PLATE_SENSOR, PAD_OVER_TEMP,
                             SECOND_PAD_OVER_TEMP, OLED, SAMPLE_DELAY };

// One independent controller state per plate. Never share I or D history.
struct PIDState {
  float integralPWM;
  float previousTemperatureC;
  float filteredRateCPerSecond;
  uint8_t pwmOutput;
  bool havePreviousTemperature;
};

struct Button {
  uint8_t pin;
  uint8_t lastRaw;
  uint8_t stable;
  uint32_t changedMs;
  bool armed;
  uint32_t pressedMs;
  bool pressActive;
  bool holdHandled;
};

struct Targets {
  uint16_t pads;
  uint16_t bag80;
  uint16_t bag20;
};
Targets targets = {DEFAULT_PAD_TENTHS, DEFAULT_BAG80_TENTHS, DEFAULT_BAG20_TENTHS};
Targets draftTargets = targets;
Menu menu = Menu::OFF;
int8_t savedSlot = -1;
uint16_t savedSequence = 0;
bool settingsSaveFailed = false;
bool savedNotice = false;
uint32_t savedNoticeMs = 0;

Button selectButton = {SELECT_BUTTON_PIN, HIGH, HIGH, 0, false, 0, false, false};
Button startButton = {START_BUTTON_PIN, HIGH, HIGH, 0, false, 0, false, false};
Stage stage = Stage::PREHEATING;
Fault fault = Fault::NONE;
bool mode20Percent = false;              // Power-on / reset default: 80%
bool displayDirty = true;
bool sensorReady = false;
bool displayReady = false;
bool displayFault = false;
float temperatureC = NAN;               // Bottom / plate 1, mux channel 7
float secondPlateTemperatureC = NAN;    // Top / plate 2, mux channel 6
float secondaryTemperatureC = NAN;      // Bag/product, mux channel 3
uint8_t goodReads = 0;
PIDState plate1PID = {};
PIDState plate2PID = {};
bool plate1Preheated = false;
bool plate2Preheated = false;
uint32_t lastSampleMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastSerialMs = 0;

// One driver, one address, isolated mux branches; no sensor EEPROM writes.
Adafruit_MLX90614 mlx;
Adafruit_SSD1306 display(128, 64, &Wire, -1, I2C_HZ, I2C_HZ);

// Explicit prototypes keep custom types compatible with Arduino preprocessing.
void beginButton(Button &button);
bool buttonPressed(Button &button);
ModeEvent readModeEvent();
bool sameTargets(const Targets &a, const Targets &b);
bool validTargets(const Targets &value);
uint16_t settingsCRC(const uint8_t *data, uint8_t count);
bool readSettingsSlot(uint8_t slot, Targets &value, uint16_t &sequence);
void loadTargets();
bool saveTargets(const Targets &value);
float padTargetC();
float cutoffTemperatureC();
float readyMinC();
float readyMaxC();
uint16_t &editingTarget();
void adjustTarget(int8_t direction);
void advanceSettings();
void handleButtons();
void beginProcess();
void setStage(Stage next);
void tripFault(Fault reason);
bool i2cResponds(uint8_t address);
bool setMuxMask(uint8_t mask);
bool readObjectTemperature(uint8_t channel, float &valueC);
void stopHeaters();
void pauseHeaters();
void pausePlate(PIDState &pid, uint8_t pin);
void controlStep(float dtSeconds);
void updatePID(PIDState &pid, uint8_t pin, float measuredC, float dtSeconds);
void updateLEDs();
float goalTemperatureC();
const __FlashStringHelper *modeText();
const __FlashStringHelper *statusText();
const __FlashStringHelper *faultText();
void printDisplayTemperature(float valueC);
void updateDisplay();
void printDebug();

void setup() {
  // Each gate pulldown keeps its heater OFF while reset/bootloader runs.
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(SECOND_HEATER_PIN, LOW);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(SECOND_HEATER_PIN, OUTPUT);
  stopHeaters();
  loadTargets();
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  updateLEDs();
  beginButton(selectButton);
  beginButton(startButton);
  if (SERIAL_DEBUG) Serial.begin(9600);

  Wire.begin();
  Wire.setClock(I2C_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  Wire.clearWireTimeoutFlag();

  sensorReady = setMuxMask((uint8_t)(1U << PRIMARY_CHANNEL));
  if (sensorReady) sensorReady = mlx.begin(MLX_ADDRESS, &Wire);
  if (!setMuxMask(0) || Wire.getWireTimeoutFlag()) sensorReady = false;
  if (!sensorReady) tripFault(Fault::PRIMARY_SENSOR);
  Wire.clearWireTimeoutFlag();

  // The OLED is downstream of the mux, so select it before even probing.
  if (setMuxMask((uint8_t)(1U << OLED_CHANNEL)) && i2cResponds(OLED_ADDRESS)) {
    displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS,
                                 false, false);
  }
  // Always attempt isolation, including when selection or begin failed.
  bool displayIsolated = setMuxMask(0);
  displayFault = !displayReady || !displayIsolated || Wire.getWireTimeoutFlag();
  Wire.clearWireTimeoutFlag();
  if (displayFault) tripFault(Fault::OLED);
  if (displayReady) {
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
  }

  lastSampleMs = lastDisplayMs = lastSerialMs = millis();
  updateDisplay();
}

void loop() {
  // Edge events are consumed even when a button is not allowed in this stage.
  // A START press during preheat cannot be queued to run later.
  handleButtons();
  uint32_t now = millis();
  uint32_t elapsed = now - lastSampleMs;
  if (elapsed >= SAMPLE_MS) {
    lastSampleMs = now;
    if (elapsed > MAX_SAMPLE_GAP_MS &&
        stage != Stage::FINISHED && stage != Stage::FAULT) {
      temperatureC = secondPlateTemperatureC = secondaryTemperatureC = NAN;
      tripFault(Fault::SAMPLE_DELAY);
    }
    controlStep(elapsed / 1000.0f);
  }

  now = millis();
  if (displayDirty || (uint32_t)(now - lastDisplayMs) >= DISPLAY_MS) {
    lastDisplayMs = now;
    updateDisplay();
  }
  now = millis();
  if (SERIAL_DEBUG && (uint32_t)(now - lastSerialMs) >= SERIAL_MS) {
    lastSerialMs = now;
    printDebug();
  }
}

// -------------------- Persistent targets --------------------
bool sameTargets(const Targets &a, const Targets &b) {
  return a.pads == b.pads && a.bag80 == b.bag80 && a.bag20 == b.bag20;
}

bool validTargets(const Targets &value) {
  return value.pads >= PAD_MIN_TENTHS && value.pads <= PAD_MAX_TENTHS &&
         value.bag80 <= BAG_MAX_TENTHS && value.bag20 <= BAG_MAX_TENTHS;
}

uint16_t settingsCRC(const uint8_t *data, uint8_t count) {
  uint16_t crc = 0xFFFF;
  while (count--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

bool readSettingsSlot(uint8_t slot, Targets &value, uint16_t &sequence) {
  const int base = slot * 16;
  if (slot > 1 || EEPROM.length() < base + 12) return false;
  uint8_t bytes[12];
  for (uint8_t i = 0; i < sizeof(bytes); ++i) bytes[i] = EEPROM.read(base + i);
  if (bytes[0] != 0xA5 || bytes[1] != 1) return false;
  uint16_t crc = (uint16_t)bytes[10] | ((uint16_t)bytes[11] << 8);
  if (settingsCRC(bytes + 1, 9) != crc) return false;
  sequence = (uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8);
  value.pads = (uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8);
  value.bag80 = (uint16_t)bytes[6] | ((uint16_t)bytes[7] << 8);
  value.bag20 = (uint16_t)bytes[8] | ((uint16_t)bytes[9] << 8);
  return validTargets(value);
}

void loadTargets() {
  targets = {DEFAULT_PAD_TENTHS, DEFAULT_BAG80_TENTHS, DEFAULT_BAG20_TENTHS};
  savedSlot = -1;
  savedSequence = 0;
  for (uint8_t slot = 0; slot < 2; ++slot) {
    Targets candidate;
    uint16_t sequence;
    if (!readSettingsSlot(slot, candidate, sequence)) continue;
    uint16_t newerBy = (uint16_t)(sequence - savedSequence);
    if (savedSlot < 0 || (newerBy != 0 && newerBy < 0x8000)) {
      targets = candidate;
      savedSequence = sequence;
      savedSlot = slot;
    }
  }
  draftTargets = targets;
}

bool saveTargets(const Targets &value) {
  if (!validTargets(value)) return false;
  if (sameTargets(value, targets)) return true; // No unnecessary EEPROM writes.
  const uint8_t slot = savedSlot == 0 ? 1 : 0;
  const int base = slot * 16;
  if (EEPROM.length() < base + 12) return false;
  const uint16_t sequence = (uint16_t)(savedSequence + 1);
  uint8_t bytes[12] = {0xA5, 1, (uint8_t)sequence, (uint8_t)(sequence >> 8),
    (uint8_t)value.pads, (uint8_t)(value.pads >> 8),
    (uint8_t)value.bag80, (uint8_t)(value.bag80 >> 8),
    (uint8_t)value.bag20, (uint8_t)(value.bag20 >> 8), 0, 0};
  uint16_t crc = settingsCRC(bytes + 1, 9);
  bytes[10] = (uint8_t)crc;
  bytes[11] = (uint8_t)(crc >> 8);
  // Commit the inactive slot last; the previous slot remains intact.
  EEPROM.update(base, 0);
  for (uint8_t i = 1; i < sizeof(bytes); ++i) EEPROM.update(base + i, bytes[i]);
  EEPROM.update(base, bytes[0]);
  Targets verified;
  uint16_t verifiedSequence;
  if (!readSettingsSlot(slot, verified, verifiedSequence) ||
      verifiedSequence != sequence || !sameTargets(value, verified)) return false;
  savedSlot = slot;
  savedSequence = sequence;
  return true;
}

float padTargetC() { return targets.pads / 10.0f; }
float cutoffTemperatureC() { return (targets.pads + CUTOFF_ABOVE_PAD_TENTHS) / 10.0f; }
float readyMinC() { return (targets.pads - 6) / 10.0f; }
float readyMaxC() { return (targets.pads + 4) / 10.0f; }

uint16_t &editingTarget() {
  if (menu == Menu::PADS) return draftTargets.pads;
  if (menu == Menu::BAG80) return draftTargets.bag80;
  return draftTargets.bag20;
}

void adjustTarget(int8_t direction) {
  uint16_t &value = editingTarget();
  int16_t candidate = (int16_t)value + direction * (int16_t)TARGET_STEP_TENTHS;
  int16_t minimum = menu == Menu::PADS ? PAD_MIN_TENTHS : 0;
  int16_t maximum = menu == Menu::PADS ? PAD_MAX_TENTHS : BAG_MAX_TENTHS;
  value = (uint16_t)constrain(candidate, minimum, maximum);
  settingsSaveFailed = false;
  displayDirty = true;
}

void advanceSettings() {
  if (menu == Menu::OFF) {
    stopHeaters();
    goodReads = 0;
    draftTargets = targets;
    settingsSaveFailed = false;
    savedNotice = false;
    menu = Menu::PADS;
  } else if (menu == Menu::PADS) {
    menu = Menu::BAG80;
  } else if (menu == Menu::BAG80) {
    menu = Menu::BAG20;
  } else {
    if (!saveTargets(draftTargets)) {
      settingsSaveFailed = true;
      displayDirty = true;
      return; // Stay in settings with heaters OFF, allowing a retry.
    }
    bool padChanged = targets.pads != draftTargets.pads;
    targets = draftTargets;
    menu = Menu::OFF;
    stopHeaters();
    goodReads = 0;
    if (padChanged) {
      plate1Preheated = plate2Preheated = false;
      setStage(Stage::PREHEATING);
    }
    lastSampleMs = millis() - SAMPLE_MS;
    savedNotice = true;
    savedNoticeMs = millis();
  }
  updateLEDs();
  displayDirty = true;
}

// -------------------- Buttons and stages --------------------
void beginButton(Button &button) {
  pinMode(button.pin, INPUT_PULLUP);
  button.lastRaw = button.stable = digitalRead(button.pin);
  button.changedMs = millis();
  button.armed = (button.stable == HIGH);
  button.pressedMs = 0;
  button.pressActive = false;
  button.holdHandled = false;
}

bool buttonPressed(Button &button) {
  uint32_t now = millis();
  uint8_t raw = digitalRead(button.pin);
  if (raw != button.lastRaw) {
    button.lastRaw = raw;
    button.changedMs = now;
  }
  if ((uint32_t)(now - button.changedMs) < BUTTON_DEBOUNCE_MS ||
      raw == button.stable) return false;

  button.stable = raw;
  if (raw == HIGH) {                    // Released: arm a new press
    button.armed = true;
    return false;
  }
  bool event = button.armed;
  button.armed = false;
  return event;
}

ModeEvent readModeEvent() {
  Button &button = selectButton;
  uint32_t now = millis();
  uint8_t raw = digitalRead(button.pin);
  if (raw != button.lastRaw) {
    button.lastRaw = raw;
    button.changedMs = now;
  }
  if ((uint32_t)(now - button.changedMs) >= BUTTON_DEBOUNCE_MS && raw != button.stable) {
    button.stable = raw;
    if (raw == LOW && button.armed) {
      button.armed = false;
      button.pressActive = true;
      button.holdHandled = false;
      button.pressedMs = button.changedMs;
    } else if (raw == HIGH) {
      bool emit = button.pressActive && !button.holdHandled;
      uint32_t duration = button.changedMs - button.pressedMs;
      button.armed = true;
      button.pressActive = false;
      if (emit) return duration >= MODE_HOLD_MS ? ModeEvent::LONG_HOLD : ModeEvent::SHORT_PRESS;
    }
  }
  if (button.pressActive && !button.holdHandled && raw == LOW && button.stable == LOW &&
      (uint32_t)(now - button.pressedMs) >= MODE_HOLD_MS) {
    button.holdHandled = true;
    return ModeEvent::LONG_HOLD;
  }
  return ModeEvent::NONE;
}

void handleButtons() {
  ModeEvent modeEvent = readModeEvent();
  bool startPressed = buttonPressed(startButton);
  if (stage != Stage::PREHEATING && stage != Stage::READY) return;

  if (modeEvent == ModeEvent::LONG_HOLD) {
    advanceSettings();
    return; // Consume simultaneous START; holding MODE advances only once.
  }
  bool modeReleased = selectButton.lastRaw == HIGH && selectButton.stable == HIGH;
  if (menu != Menu::OFF) {
    if (modeEvent == ModeEvent::SHORT_PRESS) adjustTarget(-1);
    else if (startPressed && modeReleased) adjustTarget(1);
    return;
  }
  if (modeEvent == ModeEvent::SHORT_PRESS) {
    mode20Percent = !mode20Percent;
    displayDirty = true;
  }
  // SELECT takes priority over a simultaneous START. Never start with
  // SELECT held down. Release and press START again after choosing.
  if (startPressed && modeEvent == ModeEvent::NONE && modeReleased && stage == Stage::READY) {
    if ((uint32_t)(millis() - lastSampleMs) > MAX_SAMPLE_GAP_MS) {
      tripFault(Fault::SAMPLE_DELAY);
    } else {
      beginProcess();
    }
  }
}

void beginProcess() {
  if (stage != Stage::READY || fault != Fault::NONE || menu != Menu::OFF) return;
  pauseHeaters();                        // Both OFF, retain each holding power
  secondaryTemperatureC = NAN;
  goodReads = 0;
  setStage(Stage::THAWING);
  lastSampleMs = millis() - SAMPLE_MS;   // Read all three sensors now
}

void setStage(Stage next) {
  if (stage != next) {
    stage = next;
    displayDirty = true;
    if (next == Stage::FINISHED) {
      stopHeaters();                     // Power OFF before the completion sound
      tone(BUZZER_PIN, FINISH_BEEP_HZ, FINISH_BEEP_MS);
      // Nonblocking, timed by Arduino; only runs on entry, never each loop.
    }
  }
  updateLEDs();
}

void tripFault(Fault reason) {
  menu = Menu::OFF;                    // Fault screen overrides settings; edits discarded.
  savedNotice = false;
  stopHeaters();
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  goodReads = 0;
  plate1Preheated = plate2Preheated = false;
  if (fault == Fault::NONE) fault = reason; // Preserve first failure cause
  setStage(Stage::FAULT);               // No path back except hardware reset
}

// -------------------- I2C and control --------------------
bool i2cResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
}

bool setMuxMask(uint8_t mask) {
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(mask);
  return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
}

bool readObjectTemperature(uint8_t channel, float &valueC) {
  valueC = NAN;
  if (!sensorReady || channel > 7) return false;
  Wire.clearWireTimeoutFlag();
  bool ok = setMuxMask((uint8_t)(1U << channel));
  if (ok) ok = i2cResponds(MLX_ADDRESS);
  if (ok) {
    valueC = mlx.readObjectTempC();
    ok = !Wire.getWireTimeoutFlag() && !isnan(valueC) && !isinf(valueC) &&
         valueC >= -70.0f && valueC <= 380.0f;
  }
  // Attempt isolation after failures, without hiding the timeout flag.
  bool isolated = setMuxMask(0);
  if (!ok || !isolated) {
    valueC = NAN;
    return false;
  }
  return true;
}

void stopHeaters() {
  pauseHeaters();
  plate1PID.integralPWM = 0.0f;          // Fully reset on boot, fault or finish
  plate2PID.integralPWM = 0.0f;
}

void pauseHeaters() {
  pausePlate(plate1PID, HEATER_PIN);
  pausePlate(plate2PID, SECOND_HEATER_PIN);
}

void pausePlate(PIDState &pid, uint8_t pin) {
  pid.pwmOutput = 0;
  analogWrite(pin, 0);
  // Retain I only for the short pause while initial sensor checks run.
  // Reset D history so resuming does not interpret the pause as one sample.
  pid.filteredRateCPerSecond = 0.0f;
  pid.havePreviousTemperature = false;
}

void controlStep(float dtSeconds) {
  if (stage == Stage::FAULT) {
    stopHeaters();
    return;
  }
  if (stage == Stage::FINISHED) {
    // Diagnostics only. Neither cooling, sensor loss, nor any button
    // can route this completed cycle back to PID control.
    stopHeaters();
    readObjectTemperature(PRIMARY_CHANNEL, temperatureC);
    readObjectTemperature(SECOND_PLATE_CHANNEL, secondPlateTemperatureC);
    readObjectTemperature(SECONDARY_CHANNEL, secondaryTemperatureC);
    return;
  }

  if (!readObjectTemperature(PRIMARY_CHANNEL, temperatureC)) {
    tripFault(Fault::PRIMARY_SENSOR);
    return;
  }
  if (temperatureC >= cutoffTemperatureC()) {
    tripFault(Fault::PAD_OVER_TEMP);
    return;
  }

  // Validate BOTH plates before either PID may apply new power, and before
  // considering bag completion. A fault in either path stops both heaters.
  if (!readObjectTemperature(SECOND_PLATE_CHANNEL, secondPlateTemperatureC)) {
    tripFault(Fault::SECOND_PLATE_SENSOR);
    return;
  }
  if (secondPlateTemperatureC >= cutoffTemperatureC()) {
    tripFault(Fault::SECOND_PAD_OVER_TEMP);
    return;
  }

  if (menu != Menu::OFF) {
    stopHeaters();                     // Continue plate checks, but never heat while editing.
    return;
  }

  if (stage == Stage::THAWING) {
    if (!readObjectTemperature(SECONDARY_CHANNEL, secondaryTemperatureC)) {
      tripFault(Fault::SECONDARY_SENSOR);
      return;
    }
    // Cutoff is checked on EVERY valid reading, including initial samples.
    if (secondaryTemperatureC >= goalTemperatureC()) {
      stopHeaters();
      setStage(Stage::FINISHED);
      return;
    }
  } else {
    secondaryTemperatureC = NAN;        // Disabled during preheat / ready
  }

  if (goodReads < GOOD_READS_REQUIRED) ++goodReads;
  if (goodReads < GOOD_READS_REQUIRED) {
    pauseHeaters();
    return;
  }

  // Each plate remembers reaching the band, including if it cools before
  // the other plate is ready. Both latches are required to enable START.
  if (stage == Stage::PREHEATING) {
    if (temperatureC >= readyMinC() && temperatureC <= readyMaxC()) {
      plate1Preheated = true;
    }
    if (secondPlateTemperatureC >= readyMinC() &&
        secondPlateTemperatureC <= readyMaxC()) {
      plate2Preheated = true;
    }
    if (plate1Preheated && plate2Preheated) setStage(Stage::READY);
  }
  updatePID(plate1PID, HEATER_PIN, temperatureC, dtSeconds);
  updatePID(plate2PID, SECOND_HEATER_PIN, secondPlateTemperatureC, dtSeconds);
}

// -------------------- Retuned PID with anti-windup --------------------
void updatePID(PIDState &pid, uint8_t pin, float measuredC, float dtSeconds) {
  float errorC = padTargetC() - measuredC;
  float measuredRate = 0.0f;
  if (pid.havePreviousTemperature) {
    measuredRate = (measuredC - pid.previousTemperatureC) / dtSeconds;
  }
  pid.previousTemperatureC = measuredC;
  pid.havePreviousTemperature = true;

  // Derivative on measurement avoids a derivative kick from setpoint changes.
  // Filtering limits the effect of IR measurement noise on heater power.
  float alpha = dtSeconds / (D_FILTER_SECONDS + dtSeconds);
  pid.filteredRateCPerSecond += alpha * (measuredRate - pid.filteredRateCPerSecond);
  float pTerm = Kp * errorC;
  float dTerm = -Kd * pid.filteredRateCPerSecond;

  // Store the integral contribution directly in PWM units.
  float candidateI = constrain(pid.integralPWM + Ki * errorC * dtSeconds,
                               0.0f, (float)MAX_PWM);
  float candidateOutput = pTerm + candidateI + dTerm;

  // Integrate when unsaturated, or when the error drives output BACK
  // toward the allowed range. Also clamp the integral itself.
  if ((candidateOutput >= 0.0f && candidateOutput <= MAX_PWM) ||
      (candidateOutput > MAX_PWM && errorC < 0.0f) ||
      (candidateOutput < 0.0f && errorC > 0.0f)) {
    pid.integralPWM = candidateI;
  }

  float output = constrain(pTerm + pid.integralPWM + dTerm,
                           0.0f, (float)MAX_PWM);
  pid.pwmOutput = (uint8_t)(output + 0.5f);
  analogWrite(pin, pid.pwmOutput);
}

// -------------------- OLED, LEDs, and serial --------------------
float goalTemperatureC() {
  return (mode20Percent ? targets.bag20 : targets.bag80) / 10.0f;
}

const __FlashStringHelper *modeText() {
  return mode20Percent ? F("20%") : F("80%");
}

void updateLEDs() {
  bool green = menu == Menu::OFF && fault == Fault::NONE && !displayFault &&
               (stage == Stage::READY || stage == Stage::FINISHED);
  digitalWrite(GREEN_LED_PIN, green ? HIGH : LOW);
  digitalWrite(RED_LED_PIN, green ? LOW : HIGH);
}

const __FlashStringHelper *statusText() {
  if (menu == Menu::PADS) return F("SETUP PADS");
  if (menu == Menu::BAG80) return F("SETUP BAG80");
  if (menu == Menu::BAG20) return F("SETUP BAG20");
  switch (stage) {
    case Stage::PREHEATING: return F("PREHEATING");
    case Stage::READY: return F("READY");
    case Stage::THAWING: return F("THAWING");
    case Stage::FINISHED: return F("FINISHED");
    case Stage::FAULT: return F("FAULT");
  }
  return F("UNKNOWN");
}

const __FlashStringHelper *faultText() {
  switch (fault) {
    case Fault::PRIMARY_SENSOR: return F("PAD1 / MUX ERROR");
    case Fault::SECONDARY_SENSOR: return F("BAG / MUX ERROR");
    case Fault::SECOND_PLATE_SENSOR: return F("PAD2 / MUX ERROR");
    case Fault::PAD_OVER_TEMP: return F("PAD1 OVER TEMP");
    case Fault::SECOND_PAD_OVER_TEMP: return F("PAD2 OVER TEMP");
    case Fault::OLED: return F("OLED ERROR");
    case Fault::SAMPLE_DELAY: return F("SAMPLE DELAY");
    case Fault::NONE: return F("NONE");
  }
  return F("UNKNOWN");
}

void printDisplayTemperature(float valueC) {
  if (isnan(valueC)) display.print(F("--.-"));
  else display.print(valueC, 1);
  display.print(F(" C"));
}

void updateDisplay() {
  displayDirty = false;
  if (!displayReady || displayFault) return;
  Wire.clearWireTimeoutFlag();
  // Replace the selected sensor branch with the OLED branch for every update,
  // including fault and completion screens. Never enable multiple branches.
  if (!setMuxMask((uint8_t)(1U << OLED_CHANNEL)) || !i2cResponds(OLED_ADDRESS)) {
    setMuxMask(0);                     // Best-effort isolation on failure
    displayFault = true;
    tripFault(Fault::OLED);
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (menu != Menu::OFF && stage != Stage::FAULT) {
    if (menu == Menu::PADS) display.print(F("SETUP 1/3: PADS"));
    else if (menu == Menu::BAG80) display.print(F("SETUP 2/3: BAG 80%"));
    else display.print(F("SETUP 3/3: BAG 20%"));
    display.setCursor(0, 12);
    display.setTextSize(2);
    display.print(editingTarget() / 10.0f, 1);
    display.print(F(" C"));
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print(F("START +0.5 MODE -0.5"));
    display.setCursor(0, 40);
    if (menu == Menu::BAG20) display.print(F("Hold MODE 1s: save"));
    else display.print(F("Hold MODE 1s: next"));
    display.setCursor(0, 48);
    display.print(F("Heaters OFF"));
    display.setCursor(0, 56);
    if (settingsSaveFailed) display.print(F("Save failed: retry"));
    else if (menu == Menu::PADS) display.print(F("Range 36.0-50.0 C"));
    else display.print(F("Range 0.0-35.0 C"));
  } else if (stage == Stage::FINISHED) {
    display.print(F("Finished thawing"));
    display.setTextSize(2);
    display.setCursor(0, 12);
    display.print(F("("));
    display.print(modeText());
    display.print(F(")"));
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print(F("Goal temp: "));
    printDisplayTemperature(goalTemperatureC());
    display.setCursor(0, 44);
    display.print(F("Both heaters: OFF"));
    display.setCursor(0, 56);
    display.print(F("Press RESET"));
  } else if (stage == Stage::FAULT) {
    display.print(F("STOPPED - ERROR"));
    display.setCursor(0, 12);
    display.print(faultText());
    display.setCursor(0, 24);
    display.print(F("Both heaters: OFF"));
    display.setCursor(0, 36);
    display.print(F("Check system"));
    display.setCursor(0, 48);
    display.print(F("Press RESET to retry"));
  } else {
    display.print(statusText());
    display.setCursor(0, 10);
    display.print(F("Mode:"));
    display.print(modeText());
    display.print(F(" Goal:"));
    display.print(goalTemperatureC(), 1);
    display.print(F("C"));
    display.setCursor(0, 20);
    display.print(F("P1:"));
    printDisplayTemperature(temperatureC);
    display.print(F(" PWM:"));
    display.print((plate1PID.pwmOutput * 100UL + 127UL) / 255UL);
    display.print(F("%"));
    display.setCursor(0, 30);
    display.print(F("P2:"));
    printDisplayTemperature(secondPlateTemperatureC);
    display.print(F(" PWM:"));
    display.print((plate2PID.pwmOutput * 100UL + 127UL) / 255UL);
    display.print(F("%"));
    display.setCursor(0, 40);
    display.print(F("Bag: "));
    if (stage == Stage::THAWING) printDisplayTemperature(secondaryTemperatureC);
    else display.print(F("OFF"));
    display.setCursor(0, 48);
    display.print(F("Pads target:"));
    display.print(padTargetC(), 1);
    display.print(F("C"));
    display.setCursor(0, 56);
    if (savedNotice && (uint32_t)(millis() - savedNoticeMs) < 1500UL) display.print(F("Settings saved"));
    else if (stage == Stage::READY) display.print(F("Press START"));
    else if (stage == Stage::THAWING) display.print(F("Mode locked"));
    else display.print(F("Wait for pads"));
  }
  display.display();
  bool displayIsolated = setMuxMask(0); // Also attempted after a display timeout
  if (!displayIsolated || Wire.getWireTimeoutFlag()) {
    displayFault = true;
    tripFault(Fault::OLED);
  }
}

void printDebug() {
  Serial.print(statusText());
  Serial.print(F(", Mode="));
  Serial.print(modeText());
  Serial.print(F(", Goal="));
  Serial.print(goalTemperatureC(), 1);
  Serial.print(F(" C, PadTarget="));
  Serial.print(padTargetC(), 1);
  Serial.print(F(" C, Pad1="));
  Serial.print(temperatureC, 2);
  Serial.print(F(" C, Pad2="));
  Serial.print(secondPlateTemperatureC, 2);
  Serial.print(F(" C, Bag="));
  if (stage == Stage::PREHEATING || stage == Stage::READY) {
    Serial.print(F("OFF"));
  } else {
    Serial.print(secondaryTemperatureC, 2);
    Serial.print(F(" C"));
  }
  Serial.print(F(", PWM1="));
  Serial.print(plate1PID.pwmOutput);
  Serial.print(F("/255, PWM2="));
  Serial.print(plate2PID.pwmOutput);
  Serial.print(F("/255"));
  if (stage == Stage::FAULT) {
    Serial.print(F(", "));
    Serial.print(faultText());
    Serial.print(F(" (readings at fault)"));
  }
  Serial.println();
}
