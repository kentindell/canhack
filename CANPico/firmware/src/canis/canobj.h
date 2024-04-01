// Triggers for turning a CANPico into a smart trigger for a scope/LA
typedef struct {
    uint32_t arbitration_id_mask;                       // Mask/match values over ID, DLC and data
    uint32_t arbitration_id_match;
    bool ide_match;
    uint32_t can_data_mask[2];
    uint32_t can_data_match[2];
    uint8_t can_dlc_mask;
    uint8_t can_dlc_match;
    bool on_error;                                      // Set if should trigger on error
    bool on_rx;                                         // Set if should trigger on receiving a matching frame
    bool on_tx;                                         // Set if should trigger on a transmitting a matching frame
    bool enabled;                                       // Set if trigger is enabled
} can_trigger_t;

// The main CAN() class object, holding a CAN controller, a trigger, and a Python callback function
typedef struct _rp2_can_obj_t {
    mp_obj_base_t base;
    can_controller_t controller;
    can_trigger_t triggers[1];                          // TODO allow multiple triggers
    mp_obj_t mp_rx_callback_fn;                         // Python function to call on receive
} rp2_can_obj_t;
