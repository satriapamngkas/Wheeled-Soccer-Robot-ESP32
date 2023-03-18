#ifndef Encoder_Fukuro_H
#define Encoder_Fukuro_H

#include <Arduino.h>

#define PREV_MASK 0x1 // Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 // Mask for the current state in determining direction of rotation.
#define INVALID 0x3   // XORing two states where both bits have changed.

class Encoder
{
public:
    typedef enum Encoding
    {
        X2_ENCODING,
        X4_ENCODING
    } Encoding;

    Encoder(){}
    Encoder(uint8_t channelA, uint8_t channelB, int pulses_per_rev, Encoding encoding = X4_ENCODING);
    void reset();
    int getPulses();
    int getRevolutions();
    void encode();
    static void decode()
    {
        // encode();
    }

private:
    Encoding encoding_;
    uint8_t channelA_;
    uint8_t channelB_;
    int pulses_per_rev_;
    int prev_state_;
    int cur_state_;
    volatile int pulses_;
    volatile int revolutions_;
};
#endif