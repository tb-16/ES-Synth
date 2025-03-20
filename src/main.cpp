#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

#define JOYY_Pin GPIO_PIN_0
#define JOYY_GPIO_Port GPIOA
#define JOYX_Pin GPIO_PIN_1
#define JOYX_GPIO_Port GPIOA
#define C0_Pin GPIO_PIN_3
#define C0_GPIO_Port GPIOA
#define REN_Pin GPIO_PIN_6
#define REN_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_7
#define C2_GPIO_Port GPIOA
#define RA0_Pin GPIO_PIN_0
#define RA0_GPIO_Port GPIOB
#define RA1_Pin GPIO_PIN_1
#define RA1_GPIO_Port GPIOB
#define C1_Pin GPIO_PIN_8
#define C1_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_9
#define C3_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define RA2_Pin GPIO_PIN_4
#define RA2_GPIO_Port GPIOB
#define D_Pin GPIO_PIN_5
#define D_GPIO_Port GPIOB

ADC_HandleTypeDef hadc1;

uint32_t adcValue1;
uint32_t adcValue2;

const int RA0_PIN  = D3;
const int RA1_PIN  = D6;
const int RA2_PIN  = D12;
const int REN_PIN  = A5;
const int C0_PIN   = A2;
const int C1_PIN   = D9;
const int C2_PIN   = A6;
const int C3_PIN   = D1;
const int OUT_PIN  = D11;
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;
const int DEN_BIT  = 3;
const int DRST_BIT = 4;
constexpr int FS = 22000;
constexpr float BASE_FREQ = 440.0;

volatile uint16_t pressedMask = 0;
volatile uint32_t stepSizesISR[12] = {0};
volatile bool anyKeyPressed = false;
volatile uint8_t lastKeyIndex = 0;

static void MX_ADC1_Init(void);

class Knob {
public:
    Knob(int8_t lowerLimit = 0, int8_t upperLimit = 8, int8_t defaultValue = 4)
      : _rotation(defaultValue),
        _lowerLimit(lowerLimit),
        _upperLimit(upperLimit),
        _lastDelta(0),
        _lastState(0) {}

    void update(uint8_t BA) {
        uint8_t transition = (_lastState << 2) | BA;
        int8_t knobDelta = transitionTable[transition];
        if (knobDelta == 2) {
            knobDelta = _lastDelta;
        }
        _lastDelta = knobDelta;
        _lastState = BA;
        int8_t current = __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
        int8_t desired = current + knobDelta;
        if (desired < _lowerLimit) desired = _lowerLimit;
        if (desired > _upperLimit) desired = _upperLimit;
        __atomic_store_n(&_rotation, desired, __ATOMIC_RELAXED);
    }

    int8_t read() const {
        return __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
    }

    void setLimits(int8_t lower, int8_t upper) {
        _lowerLimit = lower;
        _upperLimit = upper;
        int8_t val = __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
        if (val < _lowerLimit) val = _lowerLimit;
        if (val > _upperLimit) val = _upperLimit;
        __atomic_store_n(&_rotation, val, __ATOMIC_RELAXED);
    }

private:
    static const int8_t transitionTable[16];
    volatile int8_t _rotation;
    int8_t _lowerLimit;
    int8_t _upperLimit;
    int8_t _lastDelta;
    int8_t _lastState;
};

const int8_t Knob::transitionTable[16] = {
    0,   1,   2,   2,
   -1,   0,   2,   2,
    2,   2,   0,  -1,
    2,   2,   1,   0
};

Knob knob2(-3, 3, 0);
Knob knob3(0, 8, 4);
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);
HardwareTimer sampleTimer(TIM1);

constexpr uint32_t computeStepSize(float freq) {
    return static_cast<uint32_t>((pow(2.0, 32) * freq) / FS);
}

uint32_t baseStepSizes[12] = {
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

const char* noteNames[12] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
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
    digitalWrite(RA1_PIN, (bitIdx >> 1) & 0x01);
    digitalWrite(RA2_PIN, (bitIdx >> 2) & 0x01);
    digitalWrite(OUT_PIN, value);
    digitalWrite(REN_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN, LOW);
}

void sampleISR() {
    static uint32_t phaseAccumulators[12] = {0};
    const uint16_t localMask = __atomic_load_n(&pressedMask, __ATOMIC_RELAXED);
    const int8_t localVol = knob3.read();
    int32_t mix = 0;
    for (uint8_t i = 0; i < 12; i++) {
        if (localMask & (1 << i)) {
            uint32_t step = __atomic_load_n(&stepSizesISR[i], __ATOMIC_RELAXED);
            phaseAccumulators[i] += step;
            int32_t s = (phaseAccumulators[i] >> 24) - 128;
            mix += s;
        }
    }
    mix >>= (8 - localVol);
    mix += 128;
    if (mix < 0)   mix = 0;
    if (mix > 255) mix = 255;
    analogWrite(OUTR_PIN, (uint8_t)mix);
}

void scanKeysTask(void* pvParameters) {
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        std::bitset<16> localInputs;
        for (uint8_t row = 0; row < 4; row++) {
            setRow(row);
            delayMicroseconds(3);
            std::bitset<4> cols = readCols();
            for (uint8_t col = 0; col < 4; col++) {
                localInputs[row * 4 + col] = cols[col];
            }
        }
        bool foundAnyKey = false;
        uint8_t foundLastKey = 0;
        uint16_t localMask = 0;
        for (uint8_t i = 0; i < 12; i++) {
            bool pressed = localInputs[i];
            if (pressed) {
                foundAnyKey = true;
                foundLastKey = i;
                localMask |= (1 << i);
            }
        }
        __atomic_store_n(&pressedMask, localMask, __ATOMIC_RELAXED);
        uint8_t BA_Knob2 = (localInputs[15] << 1) | (localInputs[14]);
        uint8_t BA_Knob3 = (localInputs[13] << 1) | (localInputs[12]);
        knob2.update(BA_Knob2);
        knob3.update(BA_Knob3);
        int8_t octave = knob2.read();
        for (uint8_t i = 0; i < 12; i++) {
            if (localMask & (1 << i)) {
                uint32_t baseStep = baseStepSizes[i];
                if (octave >= 0) {
                    baseStep <<= octave;
                } else {
                    baseStep >>= -octave;
                }
                __atomic_store_n(&stepSizesISR[i], baseStep, __ATOMIC_RELAXED);
            } else {
                __atomic_store_n(&stepSizesISR[i], 0, __ATOMIC_RELAXED);
            }
        }
        __atomic_store_n(&anyKeyPressed, foundAnyKey, __ATOMIC_RELAXED);
        __atomic_store_n(&lastKeyIndex, foundLastKey, __ATOMIC_RELAXED);
    }
}

void displayUpdateTask(void* pvParameters) {
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if(HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }
        adcValue1 = HAL_ADC_GetValue(&hadc1);
        adcValue2 = HAL_ADC_GetValue(&hadc1);
        if(HAL_ADC_Stop(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }
        bool localAnyKeyPressed = __atomic_load_n(&anyKeyPressed, __ATOMIC_RELAXED);
        uint8_t localLastKey = __atomic_load_n(&lastKeyIndex, __ATOMIC_RELAXED);
        uint16_t localPressedMask = __atomic_load_n(&pressedMask, __ATOMIC_RELAXED);
        int8_t localVol = knob3.read();
        int8_t localOct = knob2.read();
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(2, 10);
        u8g2.print("Pressed: 0x");
        u8g2.print(localPressedMask, HEX);
        u8g2.setCursor(2, 20);
        if (localAnyKeyPressed) {
            u8g2.print("Note: ");
            u8g2.print(noteNames[localLastKey]);
        } else {
            u8g2.print("Note: None");
        }
        u8g2.setCursor(2, 30);
        u8g2.print("Vol=");
        u8g2.print(adcValue1);
        u8g2.print("  Oct=");
        u8g2.print(adcValue2);
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
    setOutMuxBit(DRST_BIT, LOW);
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);
    MX_ADC1_Init();
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);
    sampleTimer.setOverflow(FS, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();
    xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 2, NULL);
    xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, NULL);
    vTaskStartScheduler();
}

void loop() {
}

static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}