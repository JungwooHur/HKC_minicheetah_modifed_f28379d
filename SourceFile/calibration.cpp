/// Calibration procedures for determining position sensor offset, 
/// phase ordering, and position sensor linearization
/// 

#include "..\HeaderFile\calibration.h"
#include "..\HeaderFile\foc.h"
#include "..\HeaderFile\user_config.h"
#include "..\HeaderFile\motor_config.h"
#include "..\HeaderFile\current_controller_config.h"
#include "..\HeaderFile\PreferenceWriter.h"
#include "driverlib.h" //CCS functions library
#include "device.h"
#include "board.h" //For System configuration, generated codes.
#include "math.h"

char* msg1;
char* vmsg1;
long aval1;

volatile int newcomparevalue11;
volatile int newcomparevalue22;
volatile int newcomparevalue33;

void order_phases(PositionSensorAM5147* ps, ControllerStruct *controller){
    
    ///Checks phase order, to ensure that positive Q current produces
    ///torque in the positive direction wrt the position sensor.
    msg1 = "\n\r Checking phase ordering\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 28);
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL;                                                             //Put all volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    int sample_counter = 0;
    
    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 //inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            //space vector modulation
    for(long long int i = 0; i<65536; i++){

        // Set duty cycles
        newcomparevalue33 = (TBP_ePWM)*(1.0f-dtc_u); //TBP_ePWM is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
        newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
        newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);

        EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
        EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
        EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);

        DEVICE_DELAY_US(10);
        }

    ps->ZeroPosition();//임시코드
    float offset = (float)(ps->GetMechOffset());
    float eoff = fmodf((float)(offset*NPP), (float)(2*PI));                                        // convert mechanical angle to electrical angle

    ps->SetElecOffset(eoff);                                              // Set position sensor offset
    __float_reg[1] = ps->GetMechOffset();
    __float_reg[0] = eoff;
    E_OFFSET = eoff;

    /*
    ps->Sample(DT);
    //float theta_start = ps->GetMechPositionFixed();                                  //get initial rotor position
    DEVICE_DELAY_US(100);
    float theta_start;

    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;

    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));

    msg1 = "\n\rCurrent\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 11);
    aval1 = (long)(controller->i_d);
    ltoa(aval1,vmsg1,10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
    msg1 = "    ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 4);
    aval1 = (long)(controller->i_q);
    ltoa(aval1,vmsg1,10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
    msg1 = "    ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 4);
    aval1 = (long)(current);
    ltoa(aval1,vmsg1,10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
    msg1 = "\n\r\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 4);

    /// Rotate voltage angle
    while(theta_ref < 4*PI){                                                    //rotate for 2 electrical cycles
       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //inverse dq0 transform on voltages
       svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                        //space vector modulation
       DEVICE_DELAY_US(100);
       // Set duty cycles
       newcomparevalue33 = (TBP_ePWM)*(1.0f-dtc_u); //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
       newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
       newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);

       EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
       EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
       EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
       ps->Sample(DT);                                                            //sample position sensor
       theta_actual = ps->GetMechPositionFixed();

       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter > 200){
           sample_counter = 0 ;
           aval1 = (long)(theta_ref/(NPP));
           ltoa(aval1,vmsg1,10);
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 4);
           msg1 = "   ";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
           aval1 = (long)(theta_actual);
           ltoa(aval1,vmsg1,10);
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 4);
           msg1 = "\n\r";
           SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 2);
        }
       sample_counter++;
       theta_ref += 0.001f;
        }

    float theta_end = ps->GetMechPositionFixed();
    int direction = (theta_end - theta_start)>0;
    msg1 = "Theta Start:   ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 15);
    aval1 = (long)(theta_start);
    ltoa(aval1,vmsg1,10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
    msg1 = "    ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 4);
    msg1 = "Theta End:   ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 12);
    aval1 = (long)(theta_end);
    ltoa(aval1,vmsg1,10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
    msg1 = "\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 2);

    msg1 = "Direction:  ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 12);
    aval1 = (long)(direction);
    ltoa(aval1,vmsg1,2);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 2);
    msg1 = "\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 2);

    if(direction){
        msg1 = "Phasing correct\n\r";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 17);
    }
    else if(!direction){
        msg1 = "Phasing incorrect.  Swapping phases V and W\n\r";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 45);
    }
    PHASE_ORDER = direction;
    */
}
    
    
void calibrate(PositionSensorAM5147 *ps, ControllerStruct *controller){
    /// Measures the electrical angle offset of the position sensor
    /// and (in the future) corrects nonlinearity due to position sensor eccentricity
    msg1 = "Starting calibration procedure\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 32);
    float * error_f;
    float * error_b;
    int * lut;
    int * raw_f;
    int * raw_b;
    float * error;
    float * error_filt;

    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
    float delta = 2*PI/(128*n2);     //2*PI*NPP/(n*n2);                                              // change in angle between samples
    error_f = new float[n]();                                                     // error vector rotating forwards
    error_b = new float[n]();                                                     // error vector rotating backwards
    const int  n_lut = 128;
    lut = new int[n_lut]();                                                        // clear any old lookup table before starting.

    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    float cogging_current[window] = {0};
    
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    
        
    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
    for(long long i = 0; i<40000; i++){
        newcomparevalue33 = (TBP_ePWM)*(1.0f-dtc_u); //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
                                               // Set duty cycles
        if(PHASE_ORDER){                                   
            newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
            newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
            }
        else{
            newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
            newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
            }
        DEVICE_DELAY_US(100);
        }
    ps->Sample(DT);

    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;

    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));
    msg1 = " Current Angle : Rotor Angle : Raw Encoder \n\r\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 48);

    for(long long i = 0; i<n; i++){                                                   // rotate forwards
       for(long long j = 0; j<n2; j++){
           theta_ref += delta;
           abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
           svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
           newcomparevalue33 = (TBP_ePWM)*(1.0f-dtc_u); //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)

           if(PHASE_ORDER){
               // Check phase ordering
               newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
               newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);
               EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
               EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
               EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
           }
           else{
               newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
               newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);
               EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
               EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
               EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
           }
           DEVICE_DELAY_US(100);
           ps->Sample(DT);
       }
       ps->Sample(DT);
       theta_actual = ps->GetMechPositionFixed();
       error_f[i] = theta_ref/NPP - theta_actual;
       raw_f[i] = ps->GetRawPosition();
       aval1 = (long)(theta_ref/(NPP));
       ltoa(aval1,vmsg1,4);
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 4);
       msg1 = "   ";
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
       aval1 = (long)(theta_actual);
       ltoa(aval1,vmsg1,4);
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 4);
       msg1 = "   ";
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
       aval1 = (long)(raw_f[i]);
       ltoa(aval1,vmsg1,3);
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
       msg1 = "\n\r";
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 2);
       //theta_ref += delta;
    }
    

    for(long long i = 0; i<n; i++){                                                   // rotate backwards
       for(long long j = 0; j<n2; j++){
        theta_ref -= delta;
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
        svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
        newcomparevalue33 = (TBP_ePWM)*(1.0f-dtc_u); //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)

        if(PHASE_ORDER){
            newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
            newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
        }
        else{
            newcomparevalue22 = (TBP_ePWM)*(1.0f-dtc_v);
            newcomparevalue11 = (TBP_ePWM)*(1.0f-dtc_w);
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue33);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue11);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue22);
        }
        DEVICE_DELAY_US(100);
        ps->Sample(DT);
        }
       ps->Sample(DT);                                                            // sample position sensor
       theta_actual = ps->GetMechPositionFixed();                                    // get mechanical position
       error_b[i] = theta_ref/NPP - theta_actual;
       raw_b[i] = ps->GetRawPosition();

       aval1 = (long)(theta_ref/(NPP));
       ltoa(aval1,vmsg1,4);
       int sizeofval = sizeof(vmsg1);
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, sizeofval);
       msg1 = "   ";
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
       aval1 = (long)(theta_actual);
       ltoa(aval1,vmsg1,4);
       sizeofval = sizeof(vmsg1);
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, sizeofval);
       msg1 = "   ";
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
       aval1 = (long)(raw_b[i]);
       ltoa(aval1,vmsg1,3);
       sizeofval = sizeof(vmsg1);
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, sizeofval);
       msg1 = "\n\r";
       SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 2);
       //theta_ref -= delta;
        }
        
        float offset = 0;                                  
        for(int i = 0; i<n; i++){
            offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
            }
        offset = fmodf((float)(offset*NPP), (float)(2*PI));                                        // convert mechanical angle to electrical angle

            
        ps->SetElecOffset(offset);                                              // Set position sensor offset
        __float_reg[0] = offset;
        E_OFFSET = offset;
        
        /// Perform filtering to linearize position sensor eccentricity
        /// FIR n-sample average, where n = number of samples in one electrical cycle
        /// This filter has zero gain at electrical frequency and all integer multiples
        /// So cogging effects should be completely filtered out.
        

        float mean = 0;
        for (long long i = 0; i<n; i++){                                              //Average the forward and back directions
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
        }
        for (long long i = 0; i<n; i++){
            for(long long j = 0; j<window; j++){
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0){
                    ind += n;}                                                  // Moving average wraps around
                else if(ind > n-1) {
                    ind -= n;}
                error_filt[i] += error[ind]/(float)window;
            }
            if(i<window){
                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
            }
            //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
            mean += error_filt[i]/n;
        }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty

        msg1 = "\n\r Encoder non-linearity compensation table\n\r";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 45);
        msg1 = " Sample Number : Lookup Index : Lookup Value\n\r\n\r";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 48);
        for (int i = 0; i<n_lut; i++){                                          // build lookup table
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            aval1 = (long)(i);
            ltoa(aval1,vmsg1,3);
            SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
            msg1 = "   ";
            SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
            aval1 = (long)(ind);
            ltoa(aval1,vmsg1,3);
            SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
            msg1 = "   ";
            SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
            aval1 = (long)(lut[ind]);
            ltoa(aval1,vmsg1,3);
            SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
            msg1 = " \n\r";
            SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 3);
            DEVICE_DELAY_US(50);
            }
            
        ps->WriteLUT(lut);                                                      // write lookup table to position sensor object
        //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
        
        memcpy(&ENCODER_LUT, lut, 128*4);                                // copy the lookup table to the flash array
        msg1 = "\n\rEncoder Electrical Offset (rad) ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 34);
        aval1 = (long)(offset);
        ltoa(aval1,vmsg1,3);
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg1, 3);
        msg1 = "\n\r";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg1, 2);

        //prefs->eraseAndWriteArray();

        delete[] error_f;       //gotta free up that ram
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;


    }
