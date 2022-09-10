#ifndef __DEVICE_STM32_ENCODER_H__
#define __DEVICE_STM32_ENCODER_H__

void Tim2Init(void);
void Tim3Init(void);

void EncoderStart(uint8_t id);
void EncoderStop(uint8_t id);

int16_t EncoderReadDirection(uint8_t id);
int16_t EncoderReadCount(uint8_t id);



#endif // ! __DEVICE_STM32_ENCODER_H__
