#ifndef DUAL_ENCODER_H
#define DUAL_ENCODER_H

#include <Arduino.h>

class DualEncoder {
  public:
    DualEncoder(uint8_t pinAR, uint8_t pinBR, uint8_t pinAL, uint8_t pinBL);

    void begin();

    int32_t getCountR();
    int32_t getCountL();

    void resetCountR();
    void resetCountL();

  private:
    uint8_t pinAR, pinBR;
    uint8_t pinAL, pinBL;

    volatile int32_t encoderCountR = 0;
    volatile int32_t encoderCountL = 0;

    void updateFromISR(bool isRight, bool isA);

    static void IRAM_ATTR handleInterruptAR();
    static void IRAM_ATTR handleInterruptBR();
    static void IRAM_ATTR handleInterruptAL();
    static void IRAM_ATTR handleInterruptBL();

    static DualEncoder* instance;
};

#endif
