#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

struct {
  std::bitset<32> inputs;
  bool anyKeyPressed;
  uint8_t lastKeyIndex;
} sysState;

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

const char* noteNames[12] = {
  "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

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
  computeStepSize(BASE_FREQ * pow(2.0, 2.0/12.0)),
};

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, (rowIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (rowIdx >> 2) & 0x01);
  digitalWrite(REN_PIN, HIGH);
}

std::bitset<4> readCols(){
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
  static uint32_t phaseAcc = 0;
  uint32_t localStep = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localStep;
  int32_t Vout = (phaseAcc >> 24) - 128;
  // Factor 8 scaling to be less loud
  analogWrite(OUTR_PIN, Vout/8 + 128);
}

void scanKeysTask(void *pvParameters) {
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> localInputs;
    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> rowInputs = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        localInputs[row * 4 + col] = rowInputs[col];
      }
    }
    sysState.inputs = localInputs;
    bool anyKeyPressed = false;
    uint8_t lastKeyIndex = 0;
    uint32_t localCurrentStepSize = 0;
    for (uint8_t i = 0; i < 12; i++) {
      if (localInputs[i]) {
        localCurrentStepSize = stepSizes[i];
        anyKeyPressed = true;
        lastKeyIndex = i;
      }
    }
    sysState.anyKeyPressed = anyKeyPressed;
    sysState.lastKeyIndex = lastKeyIndex;
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);
    u8g2.setCursor(2, 30);
    if (sysState.anyKeyPressed) {
      u8g2.print("Note: ");
      u8g2.print(noteNames[sysState.lastKeyIndex]);
    } else {
      u8g2.print("Note: None");
    }
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
  TaskHandle_t scanKeysHandle = NULL;
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(scanKeysTask,"scanKeys",64,NULL,2,&scanKeysHandle);
  xTaskCreate(displayUpdateTask,"displayUpdate",256,NULL,1,&displayUpdateHandle);
  vTaskStartScheduler();
}

void loop() {
}