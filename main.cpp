/*
 * main.c
 *
 *  Created on: 2023. 4. 3.
 *      Author: Jungwoo Hur (RIMLAB)
 */

///3-phase motor Torque control (+ Position control(+ Impedance control))
///Modified by Jungwoo Hur, written by Ben Katz (Thank you very much for sharing this amazing project!)
///This code is modified for Launchxl-f28379d (Texas Instruments) + Boostxl-DRV8323RS (also TI) + AS5147P-TS_EK_AB(AMS)
///I used Code Composer Studio (TI's IDE)
///Look at c2000.syscfg(Generated Source\SysConfig\board.h) for pin mapping and all configurations
//

//
//
// Included Files
//
#include "driverlib.h" //CCS functions library
#include "device.h"
#include "board.h" //For System configuration, generated codes.
#include "math.h"

#include "HeaderFile\PreferenceWriter.h"
#include "HeaderFile\current_controller_config.h" //Controller Gains(PI Gain, PD gain, coefficients)
#include "HeaderFile\structs.h"
#include "HeaderFile\math_ops.h"

#include "HeaderFile\DRVjw.h" //Class for Motor Driver settings(SPI)
#include "HeaderFile\foc.h"
#include "HeaderFile\CAN_com.h"
#include "HeaderFile\calibration.h"




//#include "..\HeaderFile\FlashWriter.h"

#define CPUCLK_FREQUENCY    200 /*200 MHz System frequency*/

#define REST_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6

#define VERSION_NUM "1.10"


#define MSG_DATA_LENGTH_R    0   // "Don't care" for a Receive mailbox
#define MSG_DATA_LENGTH_T    6   // MSG_LENGTH for a Transmit mailbox

#define RX_MSG_OBJ_ID      1   // Use mailbox 1
#define TX_MSG_OBJ_ID      2   // Use mailbox 2

float __float_reg[64];                                                          // Floats stored in flash
int __int_reg[256];                                                             // Ints stored in flash.  Includes position sensor calibration lookup table



//
// Globals
//
uint8_t rxMsgData[8];
uint8_t txMsgData[6];

//PreferenceWriter prefs;
ControllerStruct controller;
ObserverStruct observer;

DRV832x drv;

PositionSensorAM5147 spi(16384, 0.0, NPP);




volatile int count = 0;
volatile int state = REST_MODE;
volatile int state_change = 1;

char* msg;
char* vmsg;
char receivedChar;
long aval;

char* msg11;
char* vmsg11;
long aval11;

volatile int newcomparevalue111;
volatile int newcomparevalue222;
volatile int newcomparevalue333;

volatile uint32_t status;



char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

//FOR ISENSE TEST
float adc_B_average = 0;
float adc_C_average = 0;
float adc_B_avg_save = 0;
float adc_C_avg_save = 0;
float adc_B_value = 0;
float adc_C_value = 0;
int countofloop = 1;

float offset = 0;

void initc2000(void);
void onMsgReceived(void);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_torque_mode(void);
void print_encoder(void);
void calibrate(void);
void offset_init(PositionSensorAM5147* ps);


__interrupt void INT_mySCI0_TX_ISR(void){
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
__interrupt void INT_mySCI0_RX_ISR(void){
    char c = SCI_readCharBlockingFIFO(SCIA_BASE);
    if(c == 27){
            state = REST_MODE;
            state_change = 1;
            char_count = 0;
            cmd_id = 0;
            //gpio.led->write(0);;
            for(int i = 0; i<8; i++){cmd_val[i] = 0;}
            }
    if(state == REST_MODE){
        switch (c){
            case 'c':
                state = CALIBRATION_MODE;
                state_change = 1;
                break;
            case 'm':
                state = MOTOR_MODE;
                state_change = 1;
                break;
            case 'e':
                state = ENCODER_MODE;
                state_change = 1;
                break;
            case 's':
                state = SETUP_MODE;
                state_change = 1;
                break;
            case 'z':
                spi.SetMechOffset(0);
                spi.Sample(DT);
                DEVICE_DELAY_US(20);
                M_OFFSET = spi.GetMechPosition();
                /*
                if (!prefs.ready()) prefs.open();
                    prefs.flush();                                                  // Write new prefs to flash
                    prefs.close();
                    prefs.load();
                    */
                spi.SetMechOffset(M_OFFSET);
                msg = "\n\r  Saved new zero position:  ";
                SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 30);
                DEVICE_DELAY_US(10);
                aval = (long)(M_OFFSET);
                ltoa(aval, vmsg, 10);
                SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg, 3);
                msg = "\n\r\n\r";
                SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 4);
                break;
            }

            }
    else if(state == SETUP_MODE){
        if(c == 13){
            switch (cmd_id){
                case 'b':
                    I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                    break;
                case 'i':
                    CAN_ID = atoi(cmd_val);
                    break;
                case 'm':
                    CAN_MASTER = atoi(cmd_val);
                    break;
                case 'l':
                    I_MAX = fmaxf(fminf(atof(cmd_val), 12.0f), 0.0f);
                    break;
                case 'f':
                    I_FW_MAX = fmaxf(fminf(atof(cmd_val), 10.0f), 0.0f);
                    break;
                case 't':
                    CAN_TIMEOUT = atoi(cmd_val);
                    break;
                case 'h':
                    TEMP_MAX = fmaxf(fminf(atof(cmd_val), 150.0f), 0.0f);
                    break;
                case 'c':
                    I_MAX_CONT = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                    break;
                default:
                    msg = "\n\r '";
                    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 4);
                    DEVICE_DELAY_US(10);

                    //SCI_writeCharArray(SCIA_BASE, cmd_id, 1);
                    msg = "' Not a valid command prefix\n\r\n\r";
                    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 32);
                    DEVICE_DELAY_US(10);
                    break;
                }
            /*

            if (!prefs.ready()) prefs.open();
            prefs.flush();                                                  // Write new prefs to flash
            prefs.close();
            prefs.load();

            state_change = 1;
            char_count = 0;
            cmd_id = 0;
            for(int i = 0; i<8; i++){cmd_val[i] = 0;}
            */
            }
        /*
        else{
            if(char_count == 0){cmd_id = c;}
            else{
                cmd_val[char_count-1] = c;

            }
            pc.putc(c);
            char_count++;
            }
            */
        }
    else if (state == ENCODER_MODE){
        switch (c){
            case 27:
                state = REST_MODE;
                state_change = 1;
                break;
                }
        }
    else if (state == MOTOR_MODE){
        switch (c){
            case 'd':
                controller.i_q_ref = 0;
                controller.i_d_ref = 0;
            }
        }

    //
    // Acknowledge this interrupt located in group 9
    //

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}



__interrupt void INT_myCAN0_1_ISR(void){

    CAN_clearInterruptStatus(CANB_BASE, TX_MSG_OBJ_ID);
    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

__interrupt void INT_myCAN0_0_ISR(void){
    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANB_BASE);

    //
    // Check if the cause is the receive message object 1
    //
    if(status == RX_MSG_OBJ_ID)
    {
        onMsgReceived();

        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID);
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}




__interrupt void INT_myCPUTIMER0_ISR(void)
{

    //
    // Toggle GPIO111 in software.  This will cause a trigger to
    // both ADCs via input XBAR, line 5.
    //
    GPIO_writePin(111U, 1U); // Set pin
    GPIO_writePin(111U, 0U); // Clear pin


    //
    // Wait for ADCA to complete, then acknowledge the flag.
    // Since both ADCs are running synchronously, it isn't necessary
    // to wait for completion notification from both ADCs
    //
    while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    spi.Sample(DT);
    //
    // Store results
    //
    controller.adc2_raw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);

    controller.adc1_raw = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);


/*
    adc_B_value = adc_B_value + (float)controller.adc2_raw;
    adc_C_value = adc_C_value + (float)controller.adc1_raw;

    adc_B_average = adc_B_value/countofloop;
    adc_C_average = adc_C_value/countofloop;

    countofloop++;

    if(countofloop > 2000){
        countofloop = 1;
        adc_B_avg_save = adc_B_average;
        adc_C_avg_save = adc_C_average;

        adc_B_value = 0;
        adc_C_value = 0;
        adc_B_average = 0;
        adc_C_average = 0;
    }
*/


    controller.theta_elec = spi.GetElecPosition();
    controller.theta_mech = (1.0f/GR)*spi.GetMechPosition();
    controller.dtheta_mech = (1.0f/GR)*spi.GetMechVelocity();
    controller.dtheta_elec = spi.GetElecVelocity();
    //controller.v_bus = 0.95f*controller.v_bus + 0.05f*((float)controller.adc3_raw)*V_SCALE; //filter the dc link voltage measurement

    /// Check state machine state, and run the appropriate function ///
    switch(state){
    case REST_MODE:
        if(state_change){
            enter_menu_state();
        }
        update_observer(&controller, &observer);
        break;
    case CALIBRATION_MODE:
        if(state_change){
            calibrate();
        }
        break;
    case INIT_TEMP_MODE:
        if(state_change){
            enter_torque_mode();
            count = 0;
            observer.resistance = 0.0f;
        }
        controller.i_d_ref = -2.0f;
        controller.i_q_ref = 0.0f;
        commutate(&controller, &observer, controller.theta_elec);
        if(count > 200)
        {
           float r_meas = controller.v_d*(DTC_MAX-DTC_MIN)/(controller.i_d*SQRT3);
            //testing2[count-100] = controller.i_d;
           observer.resistance += .001f*r_meas;
        }
        if(count > 1200)
        {
            count = 0;
            state = REST_MODE;
            state_change = 1;
            //gpio.led->write(0);
            observer.temperature = (double)(T_AMBIENT + ((observer.resistance/R_NOMINAL) - 1.0f)*254.5f);
            //printf("Winding Resistance:  %f\n\r", observer.resistance);
            //printf("Winding Temperature:  %f\n\r", observer.temperature);

            if(R_NOMINAL==0)
            {
                //printf("Saving winding resistance\n\r");
                R_NOMINAL = observer.resistance;
                //if (!prefs.ready()) prefs.open();
                //prefs.flush();                                                         // write offset and lookup table to flash
                //prefs.close();
            }
            //for(int i = 0; i<1000; i++){printf("%f \n\r", testing[i]);}
        }

        count++;
        break;
    case MOTOR_MODE:
        if(state_change){
            enter_torque_mode();
            count = 0;
            }
        else{
            /*
            if(controller.v_bus>28.0f){         //Turn of gate drive if bus voltage is too high, to prevent FETsplosion if the bus is cut during regen
                gpio.
                ->write(0);
                controller.ovp_flag = 1;
                state = REST_MODE;
                state_change = 1;
                printf("OVP Triggered!\n\r");
                }
                */
/*임시로 주석처리
            if((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0)){
                controller.i_d_ref = 0;
                controller.i_q_ref = 0;
                controller.kp = 0;
                controller.kd = 0;
                controller.t_ff = 0;
                }
*/
            torque_control(&controller);
            //update_observer(&controller, &observer);
            field_weaken(&controller);
            commutate(&controller, &observer, controller.theta_elec);           // Run current loop
            controller.timeout++;

            if(controller.otw_flag)
            {
                state = REST_MODE;
                state_change = 1;
                //gpio.led->write(0);
            }

           count++;
        }


        break;
    case SETUP_MODE:
        if(state_change){
            enter_setup_state();
        }
        break;
    case ENCODER_MODE:
        print_encoder();
        break;


    }

    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}

//
// Main
//
//
void main(void)
{
    //initialize c2000 board
	initc2000();
	//prefs.initFlashAPI();

    CAN_setInterruptMux(myCAN0_BASE, 1);

	CPUTimer_stopTimer(myCPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(myCPUTIMER0_BASE);
	controller.v_bus = V_BUS;
	controller.mode = 0;

	//enable DRV8323rs
	GPIO_writePin(enablePin,1);
	DEVICE_DELAY_US(100);
    GPIO_writePin(GPIO_PIN_SPISTEA,1);
    GPIO_writePin(GPIO_PIN_SPISTEB,1);

	//For Calibration, write current sense amplifier 1,1,1 to CSACR, Just setting the amplifier, not a calibration function.
    drv.calibrate();
    DEVICE_DELAY_US(100);
    drv.write_DCR(0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    DEVICE_DELAY_US(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_5, 0x0, 0x1, 0x1, 0x1, SEN_LVL_1_0);   // calibrate shunt amplifiers
    //DEVICE_DELAY_US(100);
    //zero_current(&controller.adc1_offset, &controller.adc2_offset);   //For real operartion
    DEVICE_DELAY_US(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_5, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    DEVICE_DELAY_US(100);
    zero_current(&controller.adc1_offset, &controller.adc2_offset);   //For test
    DEVICE_DELAY_US(100);
    drv.write_OCPCR(TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_88);
    drv.disable_gd();
    DEVICE_DELAY_US(100);

    //zero_current(&controller.adc1_offset, &controller.adc2_offset);             // Measure current sensor zero-offset


    reset_foc(&controller);                                                     // Reset current controller
    reset_observer(&observer);

    DEVICE_DELAY_US(100);
    //prefs.load(); //바꿔야돼. 플래시 메모리 사용

/*
    if(isnan(E_OFFSET)){E_OFFSET = 0.0f;}
    if(isnan(M_OFFSET)){M_OFFSET = 0.0f;}
    if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
    if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=12;}
    if(isnan(CAN_ID) || CAN_ID==-1){CAN_ID = 1;}
    if(isnan(CAN_MASTER) || CAN_MASTER==-1){CAN_MASTER = 0;}
    if(isnan(CAN_TIMEOUT) || CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
    if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
    if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
    if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
    */
    I_BW = 5026; //20kHz/25 = 800Hz = 5026rad/sec : timerISR 주기 바꾸면 바꿔야해.
    I_MAX=5;
    I_FW_MAX=1;
    R_NOMINAL = 0.0f;
    TEMP_MAX = 125.0f;
    I_MAX_CONT = 5.0f;
    PHASE_ORDER = 1;
    E_OFFSET = 0.0f;
    M_OFFSET = 0.0f;
    CAN_ID = 1;
    CAN_MASTER = 0;
    //For not using Flash API





    DEVICE_DELAY_US(100);

    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));
    spi.WriteLUT(lut);                                                          // Set potision sensor nonlinearity lookup table
    init_controller_params(&controller);



    CPUTimer_startTimer(myCPUTIMER0_BASE);
    DEVICE_DELAY_US(1000);
    calibrate();

    while(1)
    {

    }

}




//initialize c2000 board
void initc2000() {
    //INITIALIZE -------------------------------------------------------------------------------------------------------------------------------------
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();


    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
    //
    // Board initialization
    //
    Board_init();

    //initADCs_f();
    //initADCSOCs_f();

    Interrupt_enableInCPU(INTERRUPT_CPU_INT1 | INTERRUPT_CPU_INT9);

    //-------------------------------------------------------------------------------------------------------------------------------------------------
    //
    // GPIO111 as an output
    //
    GPIO_setPadConfig(111, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(111U, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_111_GPIO111);

    //
    // GPIO0 set as low
    //
    GPIO_writePin(111U, 0U);
    //-------------------------------------------------------------------------------------------------------------------------------------------------
    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;
}


void onMsgReceived() {
    //msgAvailable = true;
    //printf("%d\n\r", rxMsg.id);
    if(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_RXOK)) == CAN_ES_RXOK)
    {
        //
        // Get the received message
        //
        CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, rxMsgData);
    }

    controller.timeout = 0;
    if(((rxMsgData[0]==0xFF) & (rxMsgData[1]==0xFF) & (rxMsgData[2]==0xFF) & (rxMsgData[3]==0xFF) & (rxMsgData[4]==0xFF) & (rxMsgData[5]==0xFF) & (rxMsgData[6]==0xFF) & (rxMsgData[7]==0xFC))){
        state = MOTOR_MODE;
        state_change = 1;
        }
    else if(((rxMsgData[0]==0xFF) & (rxMsgData[1]==0xFF) & (rxMsgData[2]==0xFF) & (rxMsgData[3]==0xFF) * (rxMsgData[4]==0xFF) & (rxMsgData[5]==0xFF) & (rxMsgData[6]==0xFF) & (rxMsgData[7]==0xFD))){
        state = REST_MODE;
        state_change = 1;
            //gpio.led->write(0);;
        }
    else if(((rxMsgData[0]==0xFF) & (rxMsgData[1]==0xFF) & (rxMsgData[2]==0xFF) & (rxMsgData[3]==0xFF) * (rxMsgData[4]==0xFF) & (rxMsgData[5]==0xFF) & (rxMsgData[6]==0xFF) & (rxMsgData[7]==0xFE))){
        spi.ZeroPosition();
        }
    else if(state == MOTOR_MODE){
        unpack_cmd(rxMsgData, &controller);
        }
    pack_reply(txMsgData, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);


}


void enter_menu_state(void){
    drv.disable_gd();
    reset_foc(&controller);
    //gpio.enable->write(0);
    msg = "\n\r\n\r\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 6);
    msg = " Commands:\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 12);
    DEVICE_DELAY_US(10);
    msg = " m - Motor Mode\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 17);
    DEVICE_DELAY_US(10);
    msg = " c - Calibrate Encoder\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 26);
    DEVICE_DELAY_US(10);
    msg = " s - Setup\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 12);
    DEVICE_DELAY_US(10);
    msg = " e - Display Encoder\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 23);
    DEVICE_DELAY_US(10);
    msg = " z - Set Zero Position\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 25);
    DEVICE_DELAY_US(10);
    msg = " esc - Exit to Menu\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 21);
    DEVICE_DELAY_US(10);
    state_change = 0;
    //gpio.led->write(0);
    }

void enter_setup_state(void){
    state_change = 0;
    }

void enter_torque_mode(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    controller.ovp_flag = 0;
    reset_foc(&controller);                                                     // Tesets integrators, and other control loop parameters
    DEVICE_DELAY_US(10);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0;                                                     // Current Setpoints
    //gpio.led->write(1);                                                     // Turn on status LED
    state_change = 0;
    msg = "\n\r Entering Motor Mode \n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 25);
    DEVICE_DELAY_US(10);
    }

/*
void calibrate(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    //gpio.led->write(1);                                                    // Turn on status LED
    order_phases(&spi, &gpio, &controller, &prefs);                             // Check phase ordering
    calibrate(&spi, &gpio, &controller, &prefs);                                // Perform calibration procedure
    //gpio.led->write(0);                                                     // Turn off status LED
    DEVICE_DELAY_US(10);
    R_NOMINAL = 0;
    state = INIT_TEMP_MODE;
    //printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
    //drv.disable_gd();
    //state_change = 0;

    }
    */

void print_encoder(void){
    msg = "    Mechanical Angle:  ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 23);
    DEVICE_DELAY_US(10);
    aval = (long)(spi.GetMechPosition());
    ltoa(aval, vmsg, 10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg, 3);
    msg = "\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);

    msg = "    Electrical Angle:  ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 23);
    DEVICE_DELAY_US(10);
    aval = (long)(spi.GetElecPosition());
    ltoa(aval, vmsg, 10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg, 3);
    msg = "\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);

    msg = "    Raw:  ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 10);
    DEVICE_DELAY_US(10);
    aval = (long)(spi.GetRawPosition());
    ltoa(aval, vmsg, 10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg, 3);
    msg = "\n\r\n\r\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 6);
    DEVICE_DELAY_US(10);

    //printf("%d\n\r", spi.GetRawPosition());
    DEVICE_DELAY_US(10);
    }

void calibrate(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    order_phases(&spi, &controller);                             // Check phase ordering
    //calibrate(&spi, &controller);                               // Perform calibration procedure
    DEVICE_DELAY_US(500);
    R_NOMINAL = 0;
    state = REST_MODE;
    state_change = 1;
    drv.disable_gd();
    //printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
    //drv.disable_gd();
    //state_change = 0;


    }

/*
///--임시함수__To measure Angle Offset
void offset_init(PositionSensorAM5147* ps){

    msg11 = "\n\r Checking Offset value\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg11, 26);

    float theta_ref = 0;
    float v_d = V_CAL;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;


    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
    for(long long i = 0; i<40000; i++){
        newcomparevalue333 = (TBP_ePWM)*(1.0f-dtc_u)-1; //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
                                               // Set duty cycles
        if(PHASE_ORDER){
            newcomparevalue222 = (TBP_ePWM)*(1.0f-dtc_v)-1;
            newcomparevalue111 = (TBP_ePWM)*(1.0f-dtc_w)-1;
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue333);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue222);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue111);
            }
        else{
            newcomparevalue222 = (TBP_ePWM)*(1.0f-dtc_v)-1;
            newcomparevalue111 = (TBP_ePWM)*(1.0f-dtc_w)-1;
            EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue333);
            EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue111);
            EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue222);
            }
        DEVICE_DELAY_US(100);
        }

    ps->Sample(0.000025f);
    offset = ps->GetMechPosition();
    ps->SetMechOffset(offset);
    M_OFFSET = offset;

    offset = offset/NPP;
    ps->SetElecOffset(offset);                                              // Set position sensor offset
    E_OFFSET = offset;


/*
    if (!prefs.ready()) prefs.open();
        prefs.flush();                                                  // Write new prefs to flash
        prefs.close();
        prefs.load();

    ps->SetMechOffset(M_OFFSET);
    E_OFFSET = ps->GetElecPosition();
    ps->SetElecOffset(E_OFFSET);
    */
                                           // Set position sensor offset
/*
    msg11 = "\n\r  Saved new zero position:  ";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg11, 30);
    DEVICE_DELAY_US(10);
    aval11 = (long)(M_OFFSET);
    ltoa(aval11, vmsg11, 10);
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)vmsg11, 3);
    msg11 = "\n\r\n\r";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg11, 4);

    newcomparevalue333 = (TBP_ePWM)*(1.0f)-1; //2499 is the Time Base period value of ePWM. See the details in board.c (in the Generated Source file)
    newcomparevalue222 = (TBP_ePWM)*(1.0f)-1;
    newcomparevalue111 = (TBP_ePWM)*(1.0f)-1;

    EPWM_setCounterCompareValue(pwm_u_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue333);
    EPWM_setCounterCompareValue(pwm_v_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue222);
    EPWM_setCounterCompareValue(pwm_w_BASE, EPWM_COUNTER_COMPARE_A, newcomparevalue111);
}
*/
///---



//
// End of File
//
