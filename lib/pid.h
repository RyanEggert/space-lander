
# ifndef LIB_PID_H_
# define LIB_PID_H_

// PID Control
typedef struct {
    float Kp;           // Proportional scalar
    float Ki;           // Integral scalar
    float Kd;           // Derivative scalar
    float set_point;    // Controller positional setpoint
    float dt;  // Time interval between calls. Used to calc. derivative.
    float position;     // The current position of the system
    float _prev_position;   // The previous position of the system
    float _integ_state;     // Stores the "drift" of the controller over time.
                            // Used in calculation of the integral term.
    float integ_max, integ_min;  // Min & max bounds for _integ_state. 
                                 // integ_min <= _integ_state <= integ_max
} _PID_FL;

// PID Control
typedef struct {
    uint32_t Kp;
    uint32_t Ki;
    uint32_t Kd;
    uint32_t set_point;
    uint32_t dt;
    uint32_t position;
    uint32_t _prev_position;
    uint32_t _integ_state;
    uint32_t integ_max, integ_min;
} _PID_U32;

float PID_FL_control(_PID_FL *self);

uint32_t PID_U32_control(_PID_U32 *self);
#endif
