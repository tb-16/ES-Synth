#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

const uint32_t interval = 100;
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

const uint32_t FS = 22000;
constexpr float BASE_FREQ = 440.0;
volatile uint32_t currentStepSize = 0;

constexpr uint8_t NUM_NOTES = 12;
volatile bool notePressed[NUM_NOTES] = {false};
volatile uint32_t noteStepSizes[NUM_NOTES] = {0};
volatile uint32_t phaseAccumulators[NUM_NOTES] = {0};

const char* noteNames[12] = {
  "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

struct {
  std::bitset<32> inputs;
  bool anyKeyPressed;
  uint8_t lastKeyIndex;
  SemaphoreHandle_t mutex;
} sysState;

class Knob {
  public:
    Knob(uint8_t lowerLimit = 0, uint8_t upperLimit = 8)
      : _rotation(0), _lowerLimit(lowerLimit), _upperLimit(upperLimit), _lastDelta(0), _lastState(0)
    {
      _mutex = xSemaphoreCreateMutex();
    }
    
    ~Knob() {
      if (_mutex != NULL) {
        vSemaphoreDelete(_mutex);
      }
    }
    
    void update(uint8_t BA) {
      xSemaphoreTake(_mutex, portMAX_DELAY);
      uint8_t transition = (_lastState << 2) | BA;
      int8_t knobDelta = transitionTable[transition];
      if (knobDelta == 2) {
        knobDelta = _lastDelta;
      }
      _lastDelta = knobDelta;
      _lastState = BA;
      int8_t expected = __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
      int8_t desired;
      do {
          desired = expected + knobDelta;
          if (desired < _lowerLimit) desired = _lowerLimit;
          if (desired > _upperLimit) desired = _upperLimit;
      } while (!__atomic_compare_exchange_n(&_rotation, &expected, desired, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED));
      xSemaphoreGive(_mutex);
    }
    
    void setLimits(int8_t lower, int8_t upper) {
      xSemaphoreTake(_mutex, portMAX_DELAY);
      _lowerLimit = lower;
      _upperLimit = upper;
      int8_t current = __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
      if (current < _lowerLimit) {
          __atomic_store_n(&_rotation, _lowerLimit, __ATOMIC_RELAXED);
      } else if (current > _upperLimit) {
          __atomic_store_n(&_rotation, _upperLimit, __ATOMIC_RELAXED);
      }
      xSemaphoreGive(_mutex);
    }
    
    int8_t read() {
      return __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
    }
  
  private:
    int8_t _rotation;
    int8_t _lowerLimit;
    int8_t _upperLimit;
    int8_t _lastDelta;
    int8_t _lastState;
    SemaphoreHandle_t _mutex;
  
    static const int8_t transitionTable[16];
};

const int8_t Knob::transitionTable[16] = {
    0,  1,  2,  2,
   -1,  0,  2,  2,
    2,  2,  0, -1,
    2,  2,  1,  0
};

Knob knob2(-3, 3);
Knob knob3(0, 8);

U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);
HardwareTimer sampleTimer(TIM1);

constexpr uint32_t computeStepSize(float freq) {
  return static_cast<uint32_t>((pow(2, 32) * freq) / FS);
}

uint32_t stepSizes[12] = {
  computeStepSize(BASE_FREQ * pow(2.0, -9.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -8.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -7.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -6.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -5.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -4.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -3.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -2.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, -1.0/12.0)),
  computeStepSize(BASE_FREQ),
  computeStepSize(BASE_FREQ * pow(2.0, 1.0/12.0)),
  computeStepSize(BASE_FREQ * pow(2.0, 2.0/12.0))
};

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, (rowIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (rowIdx >> 2) & 0x01);
  digitalWrite(REN_PIN, HIGH);
}

std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = !digitalRead(C0_PIN);
  result[1] = !digitalRead(C1_PIN);
  result[2] = !digitalRead(C2_PIN);
  result[3] = !digitalRead(C3_PIN);
  return result;
}

void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void sampleISR() {
  static uint32_t phaseAccumulators[12] = {0};
  uint8_t localVolume = knob3.read();
  int32_t mix = 0;

  for (uint8_t i = 0; i < 12; i++) {
    if (notePressed[i]) {
      phaseAccumulators[i] += noteStepSizes[i];
      int32_t s = (phaseAccumulators[i] >> 24) - 128;
      mix += s;
    }
  }

  mix >>= (8 - localVolume);
  mix += 128;
  if (mix < 0)   mix = 0;
  if (mix > 255) mix = 255;

  analogWrite(OUTR_PIN, (uint8_t)mix);
}

void scanKeysTask(void *pvParameters) {
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> localInputs;
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> rowInputs = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        localInputs[row * 4 + col] = rowInputs[col];
      }
    }
    bool anyKeyPressed = false;
    uint8_t lastKeyIndex = 0;
    uint32_t localCurrentStepSize = 0;
    for (uint8_t i = 0; i < 12; i++) {
      bool pressed = localInputs[i];
      notePressed[i] = pressed;
      if (pressed) {
        anyKeyPressed = true;
        lastKeyIndex = i;
        localCurrentStepSize = stepSizes[i];
      }
    }
    uint8_t BA_Knob2 = (localInputs[15] << 1) | (localInputs[14]);
    knob2.update(BA_Knob2);
    int8_t octave = knob2.read();
    if (anyKeyPressed) {
      if (octave >= 0) {
        localCurrentStepSize <<= octave;
      } else {
        localCurrentStepSize >>= -octave;
      }
    }
    for (uint8_t i = 0; i < 12; i++) {
      if (notePressed[i]) {
        uint32_t baseStep = stepSizes[i];
        if (octave >= 0) {
          baseStep <<= octave;
        } else {
          baseStep >>= -octave;
        }
        noteStepSizes[i] = baseStep;
      } else {
        noteStepSizes[i] = 0;
      }
    }
    uint8_t BA_Knob3 = (localInputs[13] << 1) | (localInputs[12]);
    knob3.update(BA_Knob3);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    sysState.anyKeyPressed = anyKeyPressed;
    sysState.lastKeyIndex = lastKeyIndex;
    xSemaphoreGive(sysState.mutex);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    std::bitset<32> localInputs = sysState.inputs;
    uint8_t localLastKey = sysState.lastKeyIndex;
    bool localAnyKeyPressed = sysState.anyKeyPressed;
    xSemaphoreGive(sysState.mutex);
    int8_t localKnob3Rotation = knob3.read();
    int8_t localKnob2Rotation = knob2.read(); 
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(2, 10);
    u8g2.print(localInputs.to_ulong(), HEX);
    u8g2.setCursor(2, 20);
    if (localAnyKeyPressed) {
      u8g2.print("Note: ");
      u8g2.print(noteNames[localLastKey]);
    } else {
      u8g2.print("Note: None");
    }
    u8g2.setCursor(2, 30);
    u8g2.print(localKnob3Rotation);
    u8g2.print(localKnob2Rotation);
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

void setup() {
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  Serial.begin(9600);
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  sysState.mutex = xSemaphoreCreateMutex();
  TaskHandle_t scanKeysHandle = NULL;
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 92, NULL, 2, &scanKeysHandle);
  xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle);
  vTaskStartScheduler();
}

void loop() {
}