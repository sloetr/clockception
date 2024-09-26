#ifndef Clockception_h
#define Clockception_h


#include <Arduino.h>
#include "Clockhand.h"
#include <RTClib.h>
#include "Button.h"

class Clockception
{
private:

    enum
    {
	    CCW = 0,  // Counter-Clockwise
        CW  = 1,   // Clockwise
        QUICKEST_DIRECTION = 2, // Quickest direction
        // Movement types
        ACCELERATE = 0,
        CRUISE = 1,
        DECELERATE = 2,
        DELAY = 3,
        SWITCH_DIRECTION = 4,
        // Animations
        LONG_1 = 1,
        LONG_2 = 2,
        LONG_3 = 3,
        LONG_4 = 4,
        LONG_5 = 5,
        LONG_6 = 6,
        LONG_7 = 7,
        LONG_8 = 8,
        LONG_9 = 9,
        LONG_10 = 10,
        LONG_11 = 11,
        LONG_12 = 12,
        LONG_13 = 13,
        SHORT_1 = 21,
        SHORT_2 = 22,
        SHORT_3 = 23,
        SHORT_4 = 24,
        SHORT_5 = 25,
        SHORT_6 = 26,
        SHORT_7 = 27,
        SHORT_8 = 28,
        SHORT_9 = 29,
        SHORT_10 = 30,
        SHORT_11 = 31,
        SHORT_12 = 32,
        SHORT_13 = 33,
    };

    Clockhand *hands[18];
    RTC_DS3231 *rtc;
    Button *button_back;
    Button *button_set;
    Button *button_forward;


    float _max_speed; // Step interval in micro seconds at max speed
    unsigned int _default_acceleration_curve[100]; // Step intervals in micro seconds for creating a nice acceleration
    unsigned int _acceleration_curves[18][100]; // Step intervals in micro seconds for creating a nice acceleration
    float _c0;
    float _cn;
    int _default_acceleration_curve_length;
    unsigned int _default_acceleration_curve_end_speed;
    unsigned long _total_acceleration_duration;
    bool _directions[18];
    unsigned int _min_steps_to_take;
    unsigned int _max_steps_to_take;
    int _current_animation;
    int _previous_animation;
    uint8_t _hour;
    uint8_t _minute;
    uint8_t _second;
    uint8_t _last_minute;
    unsigned long _time_start_animation;

public:
    Clockception();

     void init();
    /* Creates hands, RTC and rotary encoder */

    void calculate_default_acceleration_curve();
    /* Calculates the default acceleration curve, which is adapted for each hand prior to executing an animation */
    
    void calculate_corrected_curves();
    /* Calculates an adapted acceleration curve to end at the desired speed for this hand */
    
    void set_direction_of_all_hands(bool direction);
    /* Sets direction of each hand */

    void set_shortest_direction_to_target();
    /* Sets direction for each hand that gives the shortest distance to the target */

    void run_animation();
    /* Sets a loop to run all animations for all hands, untill all hands are finished */

    void run();
    /* The general loop to run de program infinite */

    void wait_for_new_minute();
    /* Wait for next minute, while checking for button presses */

    void disable_drivers();
    /* Set RESET pin low */
    void enable_drivers();
    /* Set RESET pin high */
    void reset_drivers();
    /* Set RESET pin low first, then high */

    int normalize(int value, int max, int min);
    /* Returns positive value as residual from modulo of max value  */

    bool hands_finished();
    /* Returns true if all hands are finished */

    // Higer level animation functions
    void animation_long_1(); // Stretch_and_turn
    void animation_long_2(); // Opposite rotation
    void animation_long_3(); // Bird shapes
    void animation_long_3_to_birds(); // Partial animation
    void animation_long_3_to_bottom(); // Partial animation
    void animation_long_4(); // Opposite rotation with differend hand speeds
    void animation_long_5(); // Opposite rotation oriented to center
    void animation_long_6(); // Frame rotates in a minute
    void animation_long_7(); // Stretched rotation with each clock different speeds
    void animation_long_8(); // Small clocks that turn 12 hours
    void animation_long_9(); // Opposite rotation oriented to center, time that hands come together to hour and minute hands -> causes drift of hour hand
    void animation_long_10(); // Stretch_and_turn variation
    void animation_long_11(); // Stretch and each column opposite rotation
    void animation_long_12(); // Subsequent rotation downwards and back
    void animation_long_13(); // Splash animation

    void animation_short_1(); // One revolution all hands same time
    void animation_short_2(); // One revolution different start and end times
    void animation_short_3(); // All hands out, then back
    void animation_short_4(); // Create a wave through the frame
    void animation_short_5(); // Rotation with random delay and speed
    void animation_short_6(); // All hands inwards, then back
    void animation_short_7(); // Turn corners first
    void animation_short_8(); // Corners out, straights in
    void animation_short_9(); // Corners in, straights out
    void animation_short_10(); // All hands horizontal
    void animation_short_11(); // All hands vertical
    void animation_short_12(); // One rotation after each other
    void animation_short_13(); // One rotation, each row after another
    void animation_to_zero(); // All hands to zero
    void animation_to_bottom(); // All hands to bottom

    void get_time();
    /* Sets time to public variables */

    void set_time_and_frame_positions();
    /* Set target of hands to clock frame with current time */

    void set_clock_frame_positions();
    /* Set target of hands to clock frame */

    void set_time_positions();
    /* Set target of clock hands to current time */

    void show_time_equal_duration(uint8_t predefined_hour, uint8_t predefined_minute, int extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction);
    /* Creates an animation to show time in which all hands arrive at the same time. Start speed can differ. */
    
    void show_time_with_delays(uint8_t predefined_hour, uint8_t predefined_minute, int extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction);
    /* Creates an animation to show time in which all hands start at the same speed. Hands could arrive at different time at target. */

    // Animation utility functions
    void calculate_steps_to_positions(char extra_rotations);
    /* Calculates for each hand the steps needed to reach the target, taking into account desired direction and optional extra rotations */

    void calculate_animation_equal_duration(char extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction);
    /* Calculates and sets instructions of an animation in which each hand starts at the same time and accelerates to the target position. 
    Hands will arrive at the same time, but possibly on different speeds. 
    This type should be used when all hands need to start and end equally but a end speed (or start speed when decelerating) difference isn't a problem.
    */

    void calculate_run_with_same_speed(unsigned int steps, unsigned int speed);
    /* Runs hands with fixed speed, all hands same steps and speed */

    void calculate_run_with_speed(int *types, unsigned int *steps, unsigned int *speeds);
    /* Runs hands with fixed speed, each hand different */

    void calculate_animation_with_delays(char extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction, bool delay_at_start);
    /* Calculates and sets instructions of an animation in which each hand will arrive (when accelerating) or start (when decelerating) at max_speed. 
    The speed at start or arrival is equal, but hands will start or and with an delay (depending on delay_at_start).
    This type should be used when end and arrival speed need to be equal but hands can start or end at different times.
    */ 

    // Settings functions
    void set_settings();
    /* Program when rotary encoder button is pushed. Runs for example set_time(). */

    void set_time_hour_forward();
    /* Set time hour forward */

    void set_time_hour_back();
    /* Set time hour back */

    void set_time();
    /* Function to set the time of the clock. */

    void test_hand_order();
    /* Run all hands one by one */

};

#endif 