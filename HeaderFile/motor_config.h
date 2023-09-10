#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

//For U8IIkv100 Tmotor
#define R_PHASE 0.06618f           //Ohms
#define L_D 0.000012f            //Henries
#define L_Q 0.000012f            //Henries
#define KT .0625275f                 //N-m per peak phase amp, = WB*NPP*3/2
#define NPP 21                  //Number of pole pairs
#define GR 1.0f                 //Gear ratio
#define KT_OUT 0.0625275f            //KT*GR
#define WB 0.002382f              //Flux linkage, Webers.
#define R_TH 1.25f              //Kelvin per watt
#define INV_M_TH 0.02825f       //Kelvin per joule
#define T_AMBIENT 25.0f        // ambient temperature during temp calibration



#endif
