
#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H

#include "board.h"

class PositionSensor {
public:
    virtual void Sample(float dt) = 0;
    virtual float GetMechPosition() {return 0.0f;}
    virtual float GetMechPositionFixed() {return 0.0f;}
    virtual float GetElecPosition() {return 0.0f;}
    virtual float GetMechVelocity() {return 0.0f;}
    virtual float GetElecVelocity() {return 0.0f;}
    virtual void ZeroPosition(void) = 0;
    virtual long int GetRawPosition(void) = 0;
    virtual void SetElecOffset(float offset) = 0;
    virtual long int GetCPR(void) = 0;
    virtual  void WriteLUT(int new_lut[128]) = 0;
private:
    uint16_t spi_write(uint16_t val);
};
  
  
class PositionSensorEncoder: public PositionSensor {
public:
    PositionSensorEncoder(long int CPR, float offset, int ppairs);
    virtual void Sample(float dt);
    virtual float GetMechPosition();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual float GetElecVelocity();
    virtual void ZeroPosition(void);
    virtual void SetElecOffset(float offset);
    virtual long int GetRawPosition(void);
    virtual long int GetCPR(void);
    virtual  void WriteLUT(int new_lut[128]); 
private:
    //virtual void ZeroEncoderCount(void);
    //virtual void ZeroEncoderCountDown(void);
    long int _CPR, flag, rotations, _ppairs, raw, first_sample;
    //int state;
    float _offset, MechPosition, MechOffset, dir, test_pos, oldVel, out_old, velVec[40];
    int offset_lut[128];
    uint16_t spi_write(uint16_t val);
};

class PositionSensorAM5147: public PositionSensor{
public:
    PositionSensorAM5147(long int CPR, float offset, int ppairs);
    virtual void Sample(float dt);
    virtual float GetMechPosition();
    virtual float GetMechPositionFixed();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual float GetElecVelocity();
    virtual long int GetRawPosition();
    virtual float GetMechOffset();
    virtual void ZeroPosition();
    virtual void SetElecOffset(float offset);
    virtual void SetMechOffset(float offset);
    virtual long int GetCPR(void);
    virtual void WriteLUT(int new_lut[128]);
    virtual void offsetinit();
private:
    float position, ElecPosition, ElecOffset, MechPosition, MechOffset, modPosition, oldModPosition, oldVel, velVec[40], MechVelocity, ElecVelocity, ElecVelocityFilt;
    long int raw, _CPR, rotations, old_counts, _ppairs, first_sample;
    int readAngleCmd;
    int offset_lut[128];
    uint16_t spi_write(uint16_t val);

};
#endif
