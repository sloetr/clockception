#ifndef settings_h
#define settings_h
#include <Arduino.h>

const unsigned int steps_per_revolution = 4320;
const int nr_of_hands = 18;
const int hour_hand = nr_of_hands-2;
const int minute_hand = nr_of_hands-1;

// Button pins
const byte button_back_pin = 52;
const byte button_set_pin = 50;
const byte button_forward_pin = 48;

const byte stepper_driver_reset = 53;
const byte unused_pin = A12; // For setting random seed

const int motors[18][2] = {
  {11,13}, // Step, direction
  {15,17},
  {3,5},
  {7,9},
  {16,14},  
  {12,10},
  {8,6},
  {4,2},
  {46,44},
  {42,40},
  {38,36},
  {34,32},
  {41,43},
  {45,47},
  {33,35},
  {37,39},
  {25,27},
  {29,31}
};

const bool motor_inverted[18] = {false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true};

// //Voor klok nr. 2
// // Button pins
// const byte button_back_pin = 52;
// const byte button_set_pin = 50;
// const byte button_forward_pin = 48;

// const byte stepper_driver_reset = 53;
// const byte unused_pin = A12; // For setting random seed

// const int motors[18][2] = {
//   {11,13}, // Step, direction
//   {15,17},
//   {3,5},
//   {7,9},
//   {16,14},  
//   {12,10},
//   {8,6},
//   {4,2},
//   {42,40},
//   {46,44},
//   {34,32},
//   {38,36},
//   {41,43},
//   {45,47},
//   {33,35},
//   {37,39},
//   {25,27},
//   {29,31}
// };

// //Voor klok nr. 1
// // Button pins
// const byte button_back_pin = 52;
// const byte button_forward_pin = 50;
// const byte button_set_pin = 53;

// const byte stepper_driver_reset = 31;
// const byte unused_pin = A12; // For setting random seed

// const int motors[18][2] = {
//   {3,2}, // Step, direction
//   {5,4},
//   {25,30},
//   {29,28},
//   {23,24},
//   {27,26},
//   {37,36},
//   {39,38},
//   {33,32},
//   {35,34},
//   {11,10},
//   {17,16},
//   {13,12},
//   {15,14},
//   {41,40},
//   {43,42},
//   {47,44},
//   {19,18}
// };

//const bool motor_inverted[18] = {false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true};

/*
Clock position numbers in program (to create patterns in the movements), seen from the front
Hour hand/minute hand
        0/1
  14/15     2/3
12/13  16/17   4/5
   10/11    6/7
        8/9
*/

int clock_frame_positions[nr_of_hands] = {
    // Hour minute
    int(steps_per_revolution*0.375), int(steps_per_revolution*0.625), 
    int(steps_per_revolution*0.375), int(steps_per_revolution*0.875), 
    int(steps_per_revolution*0.625), int(steps_per_revolution*0.875), 
    int(steps_per_revolution*0.625), int(steps_per_revolution*0.125), 
    int(steps_per_revolution*0.875), int(steps_per_revolution*0.125), 
    int(steps_per_revolution*0.875), int(steps_per_revolution*0.375), 
    int(steps_per_revolution*0.125), int(steps_per_revolution*0.375), 
    int(steps_per_revolution*0.125), int(steps_per_revolution*0.625),
    0, 0
  };

#endif