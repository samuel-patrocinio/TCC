#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder
{
public:
    Encoder(gpio_num_t pinA, gpio_num_t pinB);

    void begin();
    float getAngle();
    void reset();

private:
    gpio_num_t pinA, pinB;
    ESP32Encoder encoder;

    static constexpr int CPR = 400;
};

#endif
