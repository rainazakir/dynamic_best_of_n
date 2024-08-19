//
// Created by Till Konrad Aust on 17.12.21.
//
#ifndef KILOGRID_H
#define KILOGRID_H

namespace argos {
    class CSpace;
    class CRay3;
    class CFloorEntity;
    class CSimulator;
}

#include <math.h>
#include <random>

#include <behaviours/agent.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>


using namespace argos;

typedef enum {
    CELL_00 = 0,
    CELL_01 = 1,
    CELL_02 = 2,
    CELL_03 = 3
}cell_num_t;
cell_num_t cell_id[4] = {CELL_00, CELL_01, CELL_02, CELL_03};

typedef enum {
    WHITE   = 0,
    RED     = 1,
    GREEN   = 2,
    BLUE    = 3,
    YELLOW  = 4,
    CYAN    = 5,
    MAGENTA = 6,
    LED_OFF = 7
} color_t;

typedef enum {
    HIGH     = 1
} brightness_t;

typedef struct {
    uint8_t data[9]; ///< message payload
    uint8_t type;    ///< message type
    uint16_t crc;    ///< message crc
} IR_message_t;

typedef struct{
    uint16_t id;

    struct {
        int8_t rtr : 1;
        uint8_t length : 4;
    } header;

    uint8_t data[8];

} CAN_message_t;

typedef enum{
    CAN_MODULE_USER = 0x00,
    CAN_FORWARD_IR_MESSAGE,
    CAN_FORWARD_IR_MESSAGE_STOP,
    CAN_KILO_BOOTPAGE,
    CAN_KILO_BOOTPAGE_SIZE,
    CAN_KILO_BOOTPAGE_NUMBER,
    CAN_MODULE_BOOT,
    CAN_MODULE_BOOTPAGE,
    CAN_MODULE_BOOTPAGE_SIZE,
    CAN_MODULE_BOOTPAGE_NUMBER,
    CAN_MODULE_KILOBOT_SETUP,
    CAN_MODULE_IDLE,
    CAN_MODULE_SETUP,
    CAN_MODULE_RUN,
    CAN_MODULE_PAUSE,                 ///< IGNORED!
    CAN_MODULE_CONFIG_SIZE,           ///< Number of 6 byte configuration data segments
    CAN_MODULE_CONFIG_TRANSFER,       ///< Configuration data transmission
    CAN_TRACKING_KILOBOT,
    CAN_KILOBOT_DATA,
    CAN_MODULE_DATA,
    CAN_TRACKING_KILOBOT_START,
    CAN_TRACKING_REQ
} CAN_message_type_t;

typedef enum __attribute__ ((__packed__)) {
    ADDR_BROADCAST = 0x00, ///< broadcast to all modules
    ADDR_ROW, ///< broadcast to all modules in a row
    ADDR_COLUMN, ///< broadcast to all modules in a column
    ADDR_INDIVIDUAL, ///< send to an individual module
    ADDR_DISPATCHER = 0xFF ///< send to dispatcher
} kilogrid_addressing_type_t;

typedef struct __attribute__((__packed__)){

    uint8_t x; ///< columns
    uint8_t y; ///< rows

    kilogrid_addressing_type_t type; ///< type of address in Kilogrid

} kilogrid_address_t;


class CKilogrid : public CLoopFunctions
{

public:
    /** Basic argos Loopfunctions **/
    CKilogrid();
    virtual ~CKilogrid();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    virtual void Terminate();
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);
   
   
   typedef std::map<CKilobotEntity*, std::vector<CVector3> > TWaypointMap;

    /** utility functions for the simulation **/
    // Get a vector of all the Kilobots in the space
    void get_kilobots_entities();

    CVector2 GetKilobotPosition(CKilobotEntity& c_kilobot_entity);
    UInt16 GetKilobotId(CKilobotEntity& c_kilobot_entity);

    // collect data and set data
    void virtual_message_reception();
    // Get debug infromation of all kilobots
    void GetDebugInfo();

    // parser for reading the configuration
    //void read_configuration(TConfigurationNode& t_tree);

    void read_configuration(std::string conf_file);
    //extern TConfigurationNode& for_access;
    // returns the corresponding option to the grid
    UInt16 position2option(CVector2 t_position);
    UInt16 grid_cell2option(CVector2 t_position);
    CVector2 position2cell(CVector2 t_position);
    CVector2 module2cell(CVector2 module_pos, cell_num_t cn);
    CVector2 position2module(CVector2 t_position);



    /** module functions **/
    // setup method for one module
    void setup(int x, int y);

    // loop method of the module
    void loop(int x, int y);

    // callback from module for receiveing messages
    void IR_rx(int x, int y, IR_message_t *m, cell_num_t c, distance_measurement_t *d, uint8_t CRC_error);

    // module method for setting led at certain cell
    void set_LED_with_brightness(int x, int y, cell_num_t cn, color_t color, brightness_t brightness);

    // module method to send a IR message
    void set_IR_message(int x, int y, IR_message_t &m, cell_num_t cn);

    // this methods are used for sending messages between cells of the kilogrid
    void init_CAN_message(CAN_message_t* cell_msg);
    uint8_t CAN_message_tx(CAN_message_t *, kilogrid_address_t);
    // for message reception
    void CAN_rx(int x, int y, CAN_message_t *m);

    /** TODO further functions have to be implemented here **/



private:
    /** utility initializations **/
    // random struct
    CRandom::CRNG* m_pcRNG;
    distance_measurement_t *d;

    /** configuration set up **/
    // for reading the config file
    std::ifstream input;
    std::string config_file_name1;
    std::string config_file_name2;
    std::string config_file_name3;
    std::string config_file_name4;
    std::string config_file_name5;

    //PUT MORE CONF FILES HERE
    int when_switch;
    int switch_num;
    int majop ;
    int record_e;

    // this data structure is used to save the init data
    std::vector<uint8_t> configuration[10][20];

    /** virtual module storage **/
    // struct of memory of one module
    struct module_mem{
        // tmp variables
        uint8_t received_x;
        uint8_t received_y;

        // counters
        uint32_t msg_number_current;
        uint32_t msg_number;
        uint32_t test_counter;

        // cell variables
        uint8_t cell_x[4] = {0, 0, 0, 0};
        uint8_t cell_y[4] = {0, 0, 0, 0};
        uint8_t cell_role[4] = {0, 0, 0, 0};
        color_t cell_colour[4] = {WHITE, WHITE, WHITE, WHITE};

        // ir message received from robots
        std::vector<IR_message_t*> robot_messages;
        // ir message to send
        IR_message_t *ir_message_tx = new IR_message_t;

        // can message used for inter cell communication - receiving
        std::vector<CAN_message_t> received_cell_messages;
        // can message to send
        CAN_message_t cell_message;
        kilogrid_address_t cell_address;  // TODO init as null?

        // tmp mem for saving the data from can message callback
        uint8_t cell_received_op[4] = {0, 0, 0, 0};
        uint32_t reset_timer[4] = {0, 0, 0, 0};

        // communication flags
        bool init_flag = false;

        // TODO add local variables needed in modules here ...
    };

    // this array mimics the storage of each module
    module_mem module_memory[10][20];

    /** communication modalities **/
    // for communication - kilobots
    typedef std::vector<CKilobotEntity*> kilobot_entities_vector;
    kilobot_entities_vector kilobot_entities;
    // the position is in cells
    std::vector<CVector2> robot_positions;

    // DEBUGGING INFORMATION
    //
    // This is an efficient way to store both controllers and debug information.
    std::vector<debug_info_t*> debug_info_kilobots;
       // DEBUGGING INFORMATION
   //
   // This is an efficient way to store both controllers and debug information.
   std::vector< std::pair<CCI_KilobotController*, debug_info_t*> > m_tKBs;
   //
   ////////////////////////////////////////
       // output file for data acquizition
    std::ofstream m_cOutput;
    std::ofstream m_cOutput_es;

    // output file name
    std::string m_strOutputFileName;
    std::string m_strOutputFileName_es;

    
    /* Experiment time in seconds */
    Real m_fTimeInSeconds;
};

#endif