/* ========================================================================== */
/*                                                                            */
/*  Sensirion.h - Library for Sensirion SHT1x & SHT7x temperature and         */
/*  humidity sensor.                                                          */
/*  Created by Markus Schatzl, November 28, 2008.                             */
/*  Released into the public domain.                                          */
/*                                                                            */
/*  Modified by Carl Jackson, August 4, 2010.                                 */
/*  - Added separate functions for temperature and humidity measurements,     */
/*    each with blocking vs. non-blocking flag                                */
/*  - Added function to write sensor status register to enable setting        */
/*    sensor precision (14-bit/12-bit Temp/RH vs. 12-bit/8-bit Temp/RH)       */
/*  - Dewpoint calculation is now public (and renamed to match new calls)     */
/*  - Updated equations with latest recommendations from Sensirion (V4)       */
/* ========================================================================== */


#ifndef Sensirion_h
#define Sensirion_h

#include <inttypes.h>

#define HIGH_RES     0x00  // 14-bit Temp, 12-bit RH
#define LOW_RES      0x01  // 12-bit Temp,  8-bit RH
#define TEMP            0
#define HUMI            1
#define BLOCK        true
#define NONBLOCK    false

#define measTemp(result)  meas(TEMP, result, BLOCK)
#define measHumi(result)  meas(HUMI, result, BLOCK)

class Sensirion
{
  private:
    uint8_t _pinData;
    uint8_t _pinClock;
    uint16_t *p_result;
    uint8_t _resolution;

    uint8_t softReset(void);
    uint8_t putByte(uint8_t);
    uint8_t readByte(uint8_t);
    uint8_t readSensorRaw(unsigned int*, uint8_t);
 
    void startTransmission(void);
    void resetConnection(void);
    void extractValues(float* ,float*);

  public:
    Sensirion(uint8_t, uint8_t);
    uint8_t measure(float*, float*, float*);   
    uint8_t meas(uint8_t, uint16_t*, uint8_t);
    bool measRdy(void);
    float calcTemp(uint16_t);
    float calcHumi(uint16_t, float);
    float calcDewpoint(float, float);
    uint8_t writeSR(uint8_t);
    uint8_t reset(void);
};

#endif
