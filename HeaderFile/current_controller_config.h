/*
 * current_controller_config.h
 *
 *  Created on: 2023. 1. 9.
 *      Author: RIMLAB
 */

#ifndef CURRENT_CONTROLLER_CONFIG_H_
#define CURRENT_CONTROLLER_CONFIG_H_

////GAIN �ٲ�ߵ�.

// Current controller///
#define K_D .05f                    // Loop gain,  Volts/Amp
#define K_Q .05f                    // Loop gain,  Volts/Amp

#define K_SCALE 0.0000119366f             // K_loop/Loop BW (rad/s) 0.0042
#define KI_D 0.137875f//0.000455f                // PI zero, in radians per sample
#define KI_Q 0.137875f//0.000455f                // PI zero, in radians per sample
#define V_BUS 24.0f                 // Volts
#define OVERMODULATION 1.0f        // 1.0 = no overmodulation

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples

//Observer//
#define DT 0.000050f
#define K_O 0.02f





#endif /* CURRENT_CONTROLLER_CONFIG_H_ */
