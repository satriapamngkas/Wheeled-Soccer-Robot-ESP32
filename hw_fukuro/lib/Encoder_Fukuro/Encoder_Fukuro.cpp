#include <Encoder_Fukuro.h>

Encoder::Encoder(uint8_t channelA,
                 uint8_t channelB,
                 int pulses_per_rev,
                 Encoding encoding) : channelA_(channelA), channelB_(channelB)
{
    pulses_ = 0;
    revolutions_ = 0;
    pulses_per_rev_ = pulses_per_rev;
    encoding_ = encoding;

    int chanA = digitalRead(channelA_);
    int chanB = digitalRead(channelB_);
    // 2-bit state.
    cur_state_ = (chanA << 1) | (chanB);
    prev_state_ = cur_state_;
    // attachInterrupt(digitalPinToInterrupt(channelA_), decode, RISING);
    // attachInterrupt(digitalPinToInterrupt(channelB_), encode, RISING);
}

// void isr1()
// {
//     encoder.encode()
// }

// void isr2()
// {
//     encoder.encode()
// }

void Encoder::reset()
{
    pulses_ = 0;
    revolutions_ = 0;
}

int Encoder::getPulses()
{
    return pulses_;
}

int Encoder::getRevolutions()
{
    revolutions_ = pulses_ / pulses_per_rev_;
    return revolutions_;
}

void Encoder::encode()
{
    int changes = 0;
    int s;
    int chanA = digitalRead(channelA_);
    int chanB = digitalRead(channelB_);

    if (chanA != cur_state_)
    {
        if (chanA != chanB)
        {
            pulses_++;
        }
        else
        {
            pulses_--;
        }
    }
    cur_state_ = chanA;
    // 2-bit state.
    // cur_state_ = (chanA << 1) | (chanB);
    // if (((cur_state_ ^ prev_state_) != INVALID) && (cur_state_ != prev_state_))
    // {
    //     // 2 bit state. Right hand bit of prev XOR left hand bit of current
    //     // gives 0 if clockwise rotation and 1 if counter clockwise rotation.
    //     changes = (prev_state_ & PREV_MASK) ^ ((cur_state_ & CURR_MASK) >> 1);

    //     if (changes == 0)
    //     {
    //         changes = -1;
    //     }

    //     pulses_ -= changes;
    // }
}
