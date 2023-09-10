/*
 * hw_config.h
 *
 *  Created on: 2023. 1. 9.
 *      Author: RIMLAB
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_


#define I_SCALE 0.0044f  // Amps per A/D Count  ///�ٲ����0.0000000347826087
#define V_SCALE 0.0008056641f     // Bus volts per A/D Count   ///�ٲ����
#define DTC_MAX 1.0f          // Max phase duty cycle
#define DTC_MIN 0.0f          // Min phase duty cycle
#define DTC_COMP .000f          /// deadtime compensation (100 ns / 25 us)  ///�ٲ����



#endif /* HW_CONFIG_H_ */
