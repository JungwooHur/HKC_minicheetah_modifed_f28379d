#include "..\HeaderFile\CAN_com.h"


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -95.0f
#define V_MAX 95.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define MSG_DATA_LENGTH_R    0   // "Don't care" for a Receive mailbox
#define MSG_DATA_LENGTH_T    6   // MSG_LENGTH for a Transmit mailbox

#define RX_MSG_OBJ_ID      1   // Use mailbox 1
#define TX_MSG_OBJ_ID      2   // Use mailbox 2


/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(uint8_t* txMsgData, float p, float v, float t){

    long int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    long int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    long int t_int = float_to_uint(t, -T_MAX, T_MAX, 12);
    //
    // Initialize the transmit message object data buffer to be sent
    //
    txMsgData[0] = CAN_ID;
    txMsgData[1] = p_int>>8;
    txMsgData[2] = p_int&0xFF;
    txMsgData[3] = v_int>>4;
    txMsgData[4] = ((v_int&0xF)<<4) | (t_int>>8);
    txMsgData[5] = t_int&0xFF;
    
    CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH_T, txMsgData);

    //
    // Poll TxOk bit in CAN_ES register to check completion of transmission
    //
    //while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
    //{
    //}
}

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(uint8_t* rxMsgData,ControllerStruct * controller){

    long int p_int = (rxMsgData[0]<<8)|rxMsgData[1];
    long int v_int = (rxMsgData[2]<<4)|(rxMsgData[3]>>4);
    long int kp_int = ((rxMsgData[3]&0xF)<<8)|rxMsgData[4];
    long int kd_int = (rxMsgData[5]<<4)|(rxMsgData[6]>>4);
    long int t_int = ((rxMsgData[6]&0xF)<<8)|rxMsgData[7];
        
    controller->p_des = uint_to_float(p_int, P_MIN, P_MAX, 16);
    controller->v_des = uint_to_float(v_int, V_MIN, V_MAX, 12);
    controller->kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
    controller->kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    controller->t_ff = uint_to_float(t_int, T_MIN, T_MAX, 12);
    //printf("Received   ");
    //printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //printf("\n\r");
    }

