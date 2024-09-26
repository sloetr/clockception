#ifndef Clockhand_h
#define Clockhand_h

#include <Arduino.h>

class Clockhand
{
private:
    enum
    {
	    CCW = 0,  // Counter-Clockwise
        CW  = 1,   // Clockwise
        QUICKEST_DIRECTION = 2, // Quickest direction
        ACCELERATE = 0,
        CRUISE = 1,
        DECELERATE = 2,
        DELAY = 3,
        SWITCH_DIRECTION = 4
    };

    byte _step_pin;
    byte _dir_pin;
    bool _inverted;
    int _steps_per_revolution;
    unsigned char _instruction_counter;
    char _instruction_set_types[10]; // Type of movement (constand, accel, decel)
    unsigned int _instruction_set_steps[10]; // Steps to take
    int _instruction_set_speeds[10]; // Speed
    float _instruction_set_step_factors[10];
    char _movement_type;
    int _movement_speed;
    float _acceleration_step_factor;
    unsigned int *_acceleration_curve;
    unsigned long _step_interval;
    unsigned long _default_step_interval;
    unsigned long _minimum_step_interval; // Used for setting time
    
    uint8_t _steps_accelerating_to_max_speed;
    uint8_t _current_instruction;
    unsigned long _last_step_time;
    unsigned int _substeps_to_go;
    unsigned int _substeps_taken;

public:
    Clockhand(int nr, byte step, byte dir, bool inverted, int steps_per_revolution, unsigned int *acceleration_curve);

    void set_direction(bool direction);
    /* Sets direction of a hand, also to the stepper driver */
    
    void clear_instructions();
    /* Clears memory of all variables assocciated with an animation to enable programming new animations. */

    void set_instruction(int type, int steps, int speed);
    /* Set the instructions for a (partial) animation */

    void get_next_instruction();
    /* Get the instructions for a (partial) animation */

    void calculate_step_interval();
    /* Calculate the step interval, depending on the movement type */

    void run_hand();
    /* Take step if step interval is passed */

    bool movement_finished();
    /* Returns if this hand has finished all the steps */

    void force_finished();
    /* Force hand to be finished */

    void update_positions();
    /* Set current position to actual position and normalize between 0 and steps_per_revolution */

    void run_manually(int direction_type);
    /* Step manually untill target is reached. Used when setting time and calibrating. Does not use the instruction functions. */

    void take_manual_step();
    /* Just take 1 step */

    byte step_pin();
    /* Returns step pin */

    byte dir_pin();
    /* Returns dir pin */


    int current_position;
    int virtual_position;
    int target_position;
    bool direction;
    bool virtual_direction;
    int nr;
    
    float _acceleration_speed_factor;
    float _accel_vs_decel_speed_factor;
    int _accel_speed;
    unsigned int steps_to_take;
    bool hand_finished;

};

#endif
