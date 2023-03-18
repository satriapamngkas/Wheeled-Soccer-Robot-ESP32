#ifndef ENCODERKRTMI_H
#define ENCODERKRTMI_H
 
/**
 * Includes
 */
#include <Arduino.h>
 
/**
 * Defines
 */
#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.
 
/**
 * Quadrature Encoder Interface.
 */
class encoderstm {
 
public:
 
    typedef enum Encoding {
 
        X2_ENCODING,
        X4_ENCODING
 
    } Encoding;
    
    /** Membuat interface dari encoder    
     *
     * @param inA DigitalIn, out A dari encoder
     * @param inB DigitalIn, out B dari encoder
     */
    encoderstm(uint8_t channelA, uint8_t channelB, int pulsesPerRev, Encoding encoding = X2_ENCODING);
    
    /**
     * Reset encoder.
     *
     * Menset pulse dan revolusi/putaran menjadi 0
     */
    void reset(void);
    
    /**
     * Membaca pulse yang didapat oleh encoder
     *
     * @return Nilai pulse yang telah dilalui.
     */
    int getPulses(void);
    
    /**
     * Membaca putaran yang didapat oleh encoder
     *
     * @return Nilai revolusi/putaran yang telah dilalui.
     */
    int getRevolutions(void);
    
    /**
     * Membaca pulse yang didapat oleh encoder
     *
     * @return Nilai pulse yang telah dilalui.
     */
    //int readDeltaPulses(void);
    
    /**
     * Membaca putaran yang didapat oleh encoder
     *
     * @return Nilai revolusi/putaran yang telah dilalui.
     */
    //int readDeltaRevolutions(void);
 
private:
 
    /**
     * Menghitung pulse
     *
     * Digunakan setiap rising/falling edge baik channel A atau B
     * Membaca putaran CW atau CCW => mengakibatkan pertambahan/pengurangan pulse
     */
    void encode(void);
 
    /**
     * Indeks setiap rising edge untuk menghitung putaran
     * Nilai bertambah 1
     */
    //void index(void);
 
    Encoding encoding_;
 
    uint8_t channelA_;
    uint8_t channelB_;
    //InterruptIn index_;
 
    int          pulsesPerRev_;
    int          prevState_;
    int          currState_;
 
    volatile int pulses_;
    volatile int revolutions_;
 
 
};
 
#endif /* ENCODERKRTMI_H */
