#include "Clockhand.h"

Clockhand::Clockhand(int nr_of_hand, byte step, byte dir, bool inverted, int steps_per_revolution, unsigned int* acceleration_curve) {

    _step_pin = step;
    _dir_pin = dir;
    _inverted = inverted;
    current_position = 0;
    _acceleration_curve = acceleration_curve;
    _instruction_counter = 0;
    _steps_per_revolution = steps_per_revolution;
    virtual_position = 0;
    _last_step_time = 0;
    _step_interval = 0;
    _default_step_interval = 500; // Speed used for setting time
    _minimum_step_interval = 250;
    
    // Set the pins for step and direction
    pinMode(_step_pin, OUTPUT);
    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_step_pin, LOW);
    digitalWrite(_dir_pin, LOW);
}

void Clockhand::set_direction(bool new_direction) {
  direction = new_direction;
  virtual_direction = direction;
   // Write direction to stepper driver.
  if (_inverted == direction) digitalWrite(_dir_pin, LOW);
  else digitalWrite(_dir_pin, HIGH);
}

void Clockhand::clear_instructions() {
    // Clears all instructions set previously
    memset(_instruction_set_types, 0, sizeof(_instruction_set_types));
    memset(_instruction_set_steps, 0, sizeof(_instruction_set_steps));
    memset(_instruction_set_speeds, 0, sizeof(_instruction_set_speeds));
    memset(_instruction_set_step_factors, 0, sizeof(_instruction_set_step_factors));
    virtual_position = current_position; // To be shure that current position is set as virtual position (when program interrupts an animation)
    hand_finished = true;
    _instruction_counter = 0;
    _current_instruction = 0;
    _substeps_to_go = 0;
    _step_interval = 0;
    _last_step_time = 0;
    _substeps_taken = 0;
}

void Clockhand::set_instruction(int type, int steps, int speed) {
  /* Adds an instruction to the instructions array */
  if(_instruction_counter == 9) Serial.println("Maximum instructions reached!");
  if(_instruction_counter >9) Serial.println("Maximum instructions exceeded!");

  if(steps <= 0) steps = 1; // Prevent division by zero
  if(speed <= 0) speed = 1; // Prevent division by zero

  _instruction_set_types[_instruction_counter] = type; // Constant, accel or decel
  _instruction_set_steps[_instruction_counter] = steps; // Steps to take
  _instruction_set_speeds[_instruction_counter] = speed; // instruction_speed is steptime, inversion of input speed
  _instruction_set_step_factors[_instruction_counter] = 100/float(steps); // Calculate the amount of steps relative to the acceleration curve.

  // Update virtual position of hand since instruction is set
  if(type != DELAY && type != SWITCH_DIRECTION) {
    if(virtual_direction == CW) virtual_position += steps;
    else virtual_position -= steps;
    // Normalize virtual position
    while(virtual_position < 0) virtual_position += _steps_per_revolution;
    while(virtual_position >= _steps_per_revolution) virtual_position -= _steps_per_revolution;
  }

  if(type == SWITCH_DIRECTION) {
    virtual_direction = !virtual_direction;
  }

  hand_finished = false;
  _instruction_counter++;
}

void Clockhand::get_next_instruction() {
  // Get next instruction row
  if(_movement_type == SWITCH_DIRECTION) {
    
    if(direction) set_direction(false);
    else set_direction(true);
    _current_instruction++; // Directly get next instruction
  }

  if(_movement_type != DELAY && _movement_type != SWITCH_DIRECTION) update_positions(); // Update position, but not if instruction was delay (since no actual steps have been taken);
  
  _substeps_taken = 0; // Reset
  
  if(_current_instruction == _instruction_counter) { // Last instrucion was already executed, so this hand is finished.
    hand_finished = true;
    clear_instructions();
    return;
  }
  
  _substeps_to_go = _instruction_set_steps[_current_instruction];
  _movement_type = _instruction_set_types[_current_instruction];
  _movement_speed = _instruction_set_speeds[_current_instruction];
  _acceleration_step_factor = _instruction_set_step_factors[_current_instruction]; // Factor to multiply the step counter with to get the correct acceleration length. Only needed when accelerating and decelerating.

  calculate_step_interval(); // Calculate first step interval
  _current_instruction++;
}

void Clockhand::calculate_step_interval() {
    
  if(_substeps_to_go == 0) { // Get next instruction because all substeps of this instructions have been taken.
    get_next_instruction();
    if(hand_finished) return; // This hand is finished, do not return a step interval.
  }

  if(_movement_type == DELAY) {
    _step_interval = _movement_speed; // Delay the time one step takes
  }
  else if(_movement_type == SWITCH_DIRECTION) {
    _step_interval = 0; // Delay the time one step takes
  }
  else if(_movement_type == CRUISE) {
    _step_interval = _movement_speed+4; // Time moving at given speed + correction for not having to perform calculations
  }
  else if (_movement_type == ACCELERATE) {
    // Correct the acceleration duration by _acceleration_step_factor. Curve has length of 100 steps, but hand will mostly accelerate in different amount of steps.
    int accel_curve_position = _substeps_taken*_acceleration_step_factor;
    if(accel_curve_position > 99) accel_curve_position = 99;
    _step_interval = _acceleration_curve[accel_curve_position];

  }
  else if (_movement_type == DECELERATE) {
    // Correct the acceleration duration by _acceleration_step_factor. Curve has length of 100 steps, but hand will mostly decelerate in different amount of steps.
    int accel_curve_position = _substeps_to_go*_acceleration_step_factor;
    if(accel_curve_position > 99) accel_curve_position = 99; 
    _step_interval = _acceleration_curve[accel_curve_position];
    _step_interval = _step_interval * _accel_vs_decel_speed_factor; // Since acceleration curve was calculated for a different end speed, multiply the step interval with a factor of relative speeds
  }

  if(_step_interval < _minimum_step_interval) _step_interval = _minimum_step_interval; // To be safe
}

void Clockhand::run_hand() {
 
  if((!hand_finished) && (((micros() - _last_step_time) >= _step_interval))) {
    
    _last_step_time = micros();
    
    if(_movement_type != DELAY) {
      // Take the acutal step
      digitalWrite(_step_pin, HIGH);
      delayMicroseconds(1);
      digitalWrite(_step_pin, LOW);
    }

    _substeps_to_go--;
    _substeps_taken++;
    
    calculate_step_interval(); // Calculate step interval to next step
    
  }
}

void Clockhand::run_manually(int direction_type) {
  if(current_position == target_position) return;

  if((micros() - _last_step_time) >= _default_step_interval) {
    // Step is due according to minimum step interval
    
    if(direction_type == QUICKEST_DIRECTION) {
      // Calculate quickest direction
      if((current_position - target_position) > (_steps_per_revolution/2)) direction = true;
      else if((current_position - target_position) < -(_steps_per_revolution/2)) direction = false;
      else if((current_position - target_position) > 0) direction = false;
      else direction = true;
    }

    set_direction(direction); 
    
    take_manual_step();

    _last_step_time = micros();
  }
}

void Clockhand::take_manual_step() {
  if(direction) current_position++;   
  else current_position--;

  // Normalize the current positions between 0 and steps per revolution
  if(current_position == _steps_per_revolution) current_position = 0;
  if(current_position < 0) current_position = _steps_per_revolution;
  
  virtual_position = current_position;

  // Take the actual step
  digitalWrite(_step_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(_step_pin, LOW);
}

bool Clockhand::movement_finished() {
  if(hand_finished) return true;
  else return false;
}

void Clockhand::force_finished() {
  hand_finished = true;
  clear_instructions();
}

void Clockhand::update_positions() {
  // Update the current position and normalize between 0 and steps per revolution.
  if(direction == CW) current_position += _substeps_taken;
  else  current_position -= _substeps_taken;
  current_position = current_position % _steps_per_revolution;
  while(current_position < 0) current_position += _steps_per_revolution;
}

byte Clockhand::step_pin() {
  return(_step_pin);
}

byte Clockhand::dir_pin() {
  return(_dir_pin);
}