#include "..\HeaderFile\PositionSensor.h"
#include "..\HeaderFile\math_ops.h"
#include "..\HeaderFile\user_config.h"
#include "..\HeaderFile\foc.h"


//Modified for c2000 board SPI communication
//AS5147 Data sheet said it works in mode1 (Transfer Protocol), but it does not work in mode1. It works in mode0. I don't know why
//val 0xFFFF is the measured angle with dynamic angle error compensation

uint16_t PositionSensor::spi_write(uint16_t val){
    uint16_t rData = 0;

    GPIO_writePin(GPIO_PIN_SPISTEB,0);
    // Transmit data
    SPI_writeDataNonBlocking(mySPI1_enc_BASE, val);
    // Block until data is received and then return it
    rData = SPI_readDataBlockingNonFIFO(mySPI1_enc_BASE);
    //chip select; When nSCS logic is high, it disables serial interface communication
    //The SCLK pin should be low when the nSCS pin transitions from high to low and from low to high.
    GPIO_writePin(GPIO_PIN_SPISTEB,1);
    //SPI end------------------------------------------------------------------------------------------------------------------------------------------
    rData = rData&(0x3FFF); //Extract last 14 bits.
    return rData;
}

uint16_t PositionSensorAM5147::spi_write(uint16_t val){
    uint16_t rData = 0;

    GPIO_writePin(GPIO_PIN_SPISTEB,0);
    // Transmit data
    SPI_writeDataNonBlocking(mySPI1_enc_BASE, val);
    // Block until data is received and then return it
    rData = SPI_readDataBlockingNonFIFO(mySPI1_enc_BASE);
    //chip select; When nSCS logic is high, it disables serial interface communication
    //The SCLK pin should be low when the nSCS pin transitions from high to low and from low to high.
    GPIO_writePin(GPIO_PIN_SPISTEB,1);
    //SPI end------------------------------------------------------------------------------------------------------------------------------------------
    rData = rData&(0x3FFF); //Extract last 14 bits.
    return rData;
}

uint16_t PositionSensorEncoder::spi_write(uint16_t val){
    uint16_t rData = 0;

    GPIO_writePin(GPIO_PIN_SPISTEB,0);
    // Transmit data
    SPI_writeDataNonBlocking(mySPI1_enc_BASE, val);
    // Block until data is received and then return it
    rData = SPI_readDataBlockingNonFIFO(mySPI1_enc_BASE);
    //chip select; When nSCS logic is high, it disables serial interface communication
    //The SCLK pin should be low when the nSCS pin transitions from high to low and from low to high.
    GPIO_writePin(GPIO_PIN_SPISTEB,1);
    //SPI end------------------------------------------------------------------------------------------------------------------------------------------
    rData = rData&(0x3FFF); //Extract last 14 bits.
    return rData;
}

PositionSensorAM5147::PositionSensorAM5147(long int CPR, float offset, int ppairs){
    //_CPR = CPR;
    _CPR = CPR;
    _ppairs = ppairs;
    ElecOffset = offset;
    rotations = 0;
    
    GPIO_writePin(GPIO_PIN_SPISTEB,1);
    readAngleCmd = 0x3FFF;
    MechOffset = offset;
    modPosition = 0;
    oldModPosition = 0;
    oldVel = 0;
    raw = 0;
    first_sample = 0;
    /*for(int i = 0; i<100; i++)              // Initial measurement is really noisy
    {
        spi_write(0);
        DEVICE_DELAY_US(100);
    }
    */

    }
    
void PositionSensorAM5147::Sample(float dt){
    //GPIO_writePin(GPIO_PIN_SPISTEB,0);
    //raw = spi->write(readAngleCmd);
    //raw &= 0x3FFF;
    uint16_t raw1 = spi_write(0x3FFF);
    raw = (int)(raw1);
/*
    int off_1 = offset_lut[raw>>7];
    int off_2 = offset_lut[((raw>>7)+1)%128];
    int off_interp = off_1 + ((off_2 - off_1)*(raw - ((raw>>7)<<7))>>7);        // Interpolate between lookup table entries
    */
    int angle = raw;// + off_interp;                                               // Correct for nonlinearity with lookup table from calibration

    if(first_sample){
        if(angle - old_counts > _CPR/2){
            rotations -= 1;
            }
        else if (angle - old_counts < -_CPR/2){
            rotations += 1;
            }
    }
    if(!first_sample){first_sample = 1;}
    
    old_counts = angle;
    oldModPosition = modPosition;
    modPosition = ((2.0f*PI * ((float) angle))/ (float)_CPR);
    position = (2.0f*PI * ((float) angle+(_CPR*rotations)))/ (float)_CPR;
    MechPosition = position - MechOffset;
    float elec = ((2.0f*PI/(float)_CPR) * (float) ((_ppairs*angle)%_CPR)) + ElecOffset;
    if(elec < 0) elec += 2.0f*PI;
    else if(elec > 2.0f*PI) elec -= 2.0f*PI;
    ElecPosition = elec;
    
    float vel;

    if((modPosition-oldModPosition) < -3.0f){
        vel = (modPosition - oldModPosition + 2.0f*PI)/dt;
        }
    else if((modPosition - oldModPosition) > 3.0f){
        vel = (modPosition - oldModPosition - 2.0f*PI)/dt;
        }
    else{
        vel = (modPosition-oldModPosition)/dt;
    }


    
    int n = 40;
    float sum = vel;
    for (int i = 1; i < (n); i++){

        velVec[n - i] = velVec[n-i-1];
        sum += velVec[n-i];
        }
    velVec[0] = vel;
    MechVelocity =  sum/((float)n);
    ElecVelocity = MechVelocity*_ppairs;
    ElecVelocityFilt = 0.995f*ElecVelocityFilt + 0.005f*ElecVelocity;
    }



long int PositionSensorAM5147::GetRawPosition(){
    return raw;
    }

float PositionSensorAM5147::GetMechPositionFixed(){
    return MechPosition+MechOffset;
    }
    
float PositionSensorAM5147::GetMechPosition(){
    return MechPosition;
    }

float PositionSensorAM5147::GetElecPosition(){
    return ElecPosition;
    }

float PositionSensorAM5147::GetElecVelocity(){
    return ElecVelocity;
    }

float PositionSensorAM5147::GetMechVelocity(){
    return MechVelocity;
    }
float PositionSensorAM5147::GetMechOffset(){
    return MechOffset;
}

void PositionSensorAM5147::ZeroPosition(){
    rotations = 0;
    MechOffset = 0;
    Sample(.000025f);
    MechOffset = GetMechPosition();
    }
    
void PositionSensorAM5147::SetElecOffset(float offset){
    ElecOffset = offset;
    }
void PositionSensorAM5147::SetMechOffset(float offset){
    MechOffset = offset;
    first_sample = 0;
    }

long int PositionSensorAM5147::GetCPR(){
    return _CPR;
    }


void PositionSensorAM5147::WriteLUT(int new_lut[128]){
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
    }

void PositionSensorAM5147::offsetinit(){

    float theta_ref = 0;
    float v_d = 0.15f;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;

    int newcomparevalue333;
    int newcomparevalue222;
    int newcomparevalue111;

    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
    for(long long i = 0; i<40000; i++){
        newcomparevalue333 = (TBP_ePWM)*(1.0f-dtc_u); //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
                                               // Set duty cycles
        if(PHASE_ORDER){
            newcomparevalue222 = (TBP_ePWM)*(1.0f-dtc_v);
            newcomparevalue111 = (TBP_ePWM)*(1.0f-dtc_w);
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue333);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue222);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue111);
            }
        else{
            newcomparevalue222 = (TBP_ePWM)*(1.0f-dtc_v);
            newcomparevalue111 = (TBP_ePWM)*(1.0f-dtc_w);
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue333);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue111);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue222);
            }
        DEVICE_DELAY_US(10);
        }

    Sample(0.000025f);
    SetMechOffset(MechPosition);
    M_OFFSET = MechPosition;

    SetElecOffset(ElecPosition);                                              // Set position sensor offset
    E_OFFSET = ElecPosition;


/*
    if (!prefs.ready()) prefs.open();
        prefs.flush();                                                  // Write new prefs to flash
        prefs.close();
        prefs.load();

    ps->SetMechOffset(M_OFFSET);
    E_OFFSET = ps->GetElecPosition();
    ps->SetElecOffset(E_OFFSET);
    */


    newcomparevalue333 = (TBP_ePWM)*(1.0f); //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
    newcomparevalue222 = (TBP_ePWM)*(1.0f);
    newcomparevalue111 = (TBP_ePWM)*(1.0f);

    EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue333);
    EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue222);
    EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue111);

}
    


PositionSensorEncoder::PositionSensorEncoder(long int CPR, float offset, int ppairs) {
    _ppairs = ppairs;
    _CPR = CPR;
    _offset = offset;
    MechPosition = 0;
    out_old = 0;
    oldVel = 0;
    raw = 0;
    
    // Extra Timer for velocity measurement
    /*
    __TIM2_CLK_ENABLE();
    
    TIM2->PSC = 0x03;
    //TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->SMCR = 0x24;                                                      //TS = 010 for ITR2, SMS = 100 (reset counter at edge)
    TIM2->CCMR1 = 0x3;                                                      // CC1S = 11, IC1 mapped on TRC
    
    //TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->CCER |= TIM_CCER_CC1P;
    //TIM2->CCER |= TIM_CCER_CC1NP;
    TIM2->CCER |= TIM_CCER_CC1E;
    
    
    TIM2->CR1 = 0x01;                                                       //CEN,  enable timer
    */
    /*
    ZPulse = new InterruptIn(PC_4);
    ZSense = new DigitalIn(PC_4);
    //ZPulse = new InterruptIn(PB_0);
    //ZSense = new DigitalIn(PB_0);
    ZPulse->enable_irq();
    ZPulse->rise(this, &PositionSensorEncoder::ZeroEncoderCount);
    //ZPulse->fall(this, &PositionSensorEncoder::ZeroEncoderCountDown);
    ZPulse->mode(PullDown);
    */
    flag = 0;

    
    //ZTest = new DigitalOut(PC_2);
    //ZTest->write(1);
    }
    
void PositionSensorEncoder::Sample(float dt){
    
    }

 
float PositionSensorEncoder::GetMechPosition() {                            //returns rotor angle in radians.
    uint16_t raw = (spi_write(0x3FFF));
    long int raw1 = (long int)(raw);
    float unsigned_mech = (6.28318530718f/(float)_CPR) * (float) ((raw1)%_CPR);
    return (float) unsigned_mech;// + 6.28318530718f* (float) rotation
}

float PositionSensorEncoder::GetElecPosition() {                            //returns rotor electrical angle in radians.
    uint16_t raw = (spi_write(0x3FFF));
    long int raw1 = (long int)(raw);
    float elec = ((6.28318530718f/(float)_CPR) * (float) ((_ppairs*raw1)%_CPR)) - _offset;
    if(elec < 0) elec += 6.28318530718f;
    return elec;
}


    
float PositionSensorEncoder::GetMechVelocity(){


    float out = 0;
    /*
    float rawPeriod = TIM2->CCR1; //Clock Ticks
    int currentTime = TIM2->CNT;
    if(currentTime > 2000000){rawPeriod = currentTime;}
    float  dir = -2.0f*(float)(((TIM3->CR1)>>4)&1)+1.0f;    // +/- 1
    float meas = dir*180000000.0f*(6.28318530718f/(float)_CPR)/rawPeriod; 
    if(isinf(meas)){ meas = 1;}
    out = meas;
    //if(meas == oldVel){
     //   out = .9f*out_old;
     //   }
    

    oldVel = meas;
    out_old = out;
    */
    int n = 120;
    float sum = out;
    for (int i = 1; i < (n); i++){
        velVec[n - i] = velVec[n-i-1];
        sum += velVec[n-i];
        }
    velVec[0] = out;
    return sum/(float)n;
    }
    
float PositionSensorEncoder::GetElecVelocity(){
    return _ppairs*GetMechVelocity();
    }

/*
void PositionSensorEncoder::ZeroEncoderCount(void){
    if (ZSense->read() == 1 & flag == 0){
        if (ZSense->read() == 1){
            GPIOC->ODR ^= (1 << 4);   
            TIM3->CNT = 0x000;
            //state = !state;
            //ZTest->write(state);
            GPIOC->ODR ^= (1 << 4);
            //flag = 1;
        }
        }
    }
*/
void PositionSensorEncoder::ZeroPosition(void){
    
    }
/*
void PositionSensorEncoder::ZeroEncoderCountDown(void){
    if (ZSense->read() == 0){
        if (ZSense->read() == 0){
            GPIOC->ODR ^= (1 << 4);
            flag = 0;
            float dir = -2.0f*(float)(((TIM3->CR1)>>4)&1)+1.0f;
            if(dir != dir){
                dir = dir;
                rotations +=  dir;
                }

            GPIOC->ODR ^= (1 << 4);

        }
        }
    }
    */
void PositionSensorEncoder::SetElecOffset(float offset){
    
    }
    
long int PositionSensorEncoder::GetRawPosition(void){
    return 0;
    }
    
long int PositionSensorEncoder::GetCPR(){
    return _CPR;
    }
    

void PositionSensorEncoder::WriteLUT(int new_lut[128]){
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
    }
