#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "foc.h"
#include "PositionSensor.h"
#include "PreferenceWriter.h"
#include "user_config.h"

#define V_CAL 0.30f;

void order_phases(PositionSensorAM5147 *ps, ControllerStruct *controller);//, PreferenceWriter *prefs);
void calibrate(PositionSensorAM5147 *ps, ControllerStruct *controller);//, PreferenceWriter *prefs);
#endif
