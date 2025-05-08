#include "DualEncoder.h"

DualEncoder* DualEncoder::instance = nullptr;

DualEncoder::DualEncoder(uint8_t pinAR, uint8_t pinBR, uint8_t pinAL, uint8_t pinBL)
  : pinAR(pinAR), pinBR(pinBR), pinAL(pinAL), pinBL(pinBL) {
  instance = this;
}

void DualEncoder::begin() {
  // Right
  pinMode(pinAR, INPUT);
  pinMode(pinBR, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinAR), handleInterruptAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinBR), handleInterruptBR, CHANGE);

  // Left
  pinMode(pinAL, INPUT);
  pinMode(pinBL, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinAL), handleInterruptAL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinBL), handleInterruptBL, CHANGE);
}

int32_t DualEncoder::getCountR() {
  noInterrupts();
  int32_t count = encoderCountR;
  interrupts();
  return count;
}

int32_t DualEncoder::getCountL() {
  noInterrupts();
  int32_t count = encoderCountL;
  interrupts();
  return count;
}

void DualEncoder::resetCountR() {
  noInterrupts();
  encoderCountR = 0;
  interrupts();
}

void DualEncoder::resetCountL() {
  noInterrupts();
  encoderCountL = 0;
  interrupts();
}

void DualEncoder::updateFromISR(bool isRight, bool isA) {
  if (isRight) {
    bool A = digitalRead(pinAR);
    bool B = digitalRead(pinBR);
    if (isA) {
      if (A != B) encoderCountR++;
      else        encoderCountR--;
    } else {
      if (A == B) encoderCountR++;
      else        encoderCountR--;
    }
  } else {
    bool A = digitalRead(pinAL);
    bool B = digitalRead(pinBL);
    if (isA) {
      if (A != B) encoderCountL++;
      else        encoderCountL--;
    } else {
      if (A == B) encoderCountL++;
      else        encoderCountL--;
    }
  }
}

// ==== Static ISR wrappers ====

void IRAM_ATTR DualEncoder::handleInterruptAR() {
  if (instance) instance->updateFromISR(true, true);
}

void IRAM_ATTR DualEncoder::handleInterruptBR() {
  if (instance) instance->updateFromISR(true, false);
}

void IRAM_ATTR DualEncoder::handleInterruptAL() {
  if (instance) instance->updateFromISR(false, true);
}

void IRAM_ATTR DualEncoder::handleInterruptBL() {
  if (instance) instance->updateFromISR(false, false);
}
