#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
extern "C" {
#endif

// this definition needs to be here bc the robot and the modules need it and the kilogrid imports
// this h file. We take all values so we do not overwrite by accident
typedef enum {
    // Kilobot to Kilobot range
    MSG_T_NORMAL_IR = 0,
    MSG_T_GPS,
    MSG_T_TEST = 0x0A, // reserved for tests
    MSG_T_VIRTUAL_ROBOT_MSG,
    // Module to Kilobot range
    MSG_T_TRACKING = 0x10,

    MSG_T_GO_STRAIGHT = 0x7D,
    MSG_T_TURN_LEFT,
    MSG_T_TURN_RIGHT,

    // Kilobot configuration range
    MSG_T_BOOT = 0x80,
    MSG_T_BOOTPGM_PAGE,
    MSG_T_BOOTPGM_SIZE,
    MSG_T_RESET,
    MSG_T_SLEEP,
    MSG_T_WAKEUP,
    MSG_T_CHARGE,
    MSG_T_VOLTAGE,
    MSG_T_RUN,
    MSG_T_READUID,
    MSG_T_CALIB
} IR_message_type_t;

/*-----------------------------------------------------------------------------------------------*/
/* DEBUGGING INFORMATION                                                                         */
/*                                                                                               */
/* You must define a struct called 'debug_info_t'                                                */
/*                                                                                               */
/* The name 'debug_info_t' is mandatory                                                          */
/* The content of the struct is whatever you want                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef struct {
    // current commitement of the robot - used for debugging
    int commitement;

    // the following values are used for robot to kilogrid communication
    // flag if robot want to send a message
    int broadcast_flag;

    // content of message, type usually MSG_T_VIRTUAL_ROBOT_MSG, 8 byte of payload
    IR_message_type_t type;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
    uint8_t data7;
       int currentopinion; // value of robot gradient
    int estimateR; // value of robot gradient
    int estimateB; // value of robot gradient
    int estimateG; // value of robot gradient
    int estimateY; // value of robot gradient

    int estimateBL; // value of robot gradient
    int estimateN; // value of robot gradient


    int tot_e; // value of robot gradient

    int kilo_uid; // value of robot gradient

} debug_info_t;

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
}
#endif