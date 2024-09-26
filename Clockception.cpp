#include "Clockception.h"
#include "settings.h"

Clockception::Clockception() {
  _max_speed = 1000.0;
  _default_acceleration_curve_length = 100;
  _current_animation = 0;
}


///////////////////////////////////////////////////////////////////////////////// UTILITY /////////////////////////////////////////////////////////////////////////
void Clockception::init() { 
  reset_drivers();
  calculate_default_acceleration_curve();

  Serial.println(F("Creating clockhands"));
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand] = new Clockhand(hand, motors[hand][0], motors[hand][1], motor_inverted[hand], steps_per_revolution, _acceleration_curves[hand]);
    hands[hand]->clear_instructions();
    hands[hand]->set_direction(true);
  }

  // Initiate RTC
  rtc = new RTC_DS3231;
  if (! rtc->begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    while (1) delay(10);
  }

  // Initiate buttons
  button_back = new Button(button_back_pin);
  button_forward = new Button(button_forward_pin);
  button_set = new Button(button_set_pin);
}

void Clockception::calculate_default_acceleration_curve() {
  // Calculate the standard acceleration curve, of which all the hand-specific curves are derived. Equations from the accelstepper library.
  int acceleration_speed = 4000;
  
  _cn = 0.676 * sqrt(2.0 / acceleration_speed) * 1000000.0; // Equation 15
  _default_acceleration_curve[0] = int(_cn);
  _total_acceleration_duration +=  _default_acceleration_curve[0];
  //stepsToStop = (long)((_max_speed * _max_speed) / (2.0 * _acceleration_speed));

  for(int i=1; i<_default_acceleration_curve_length; i++) { // Calculate the rest of the curve
    _cn = _cn - ((2.0 * _cn) / ((4.0 * i) + 1.0));
    _default_acceleration_curve[i] = int(_cn);
    _total_acceleration_duration += _default_acceleration_curve[i];
  }

  _total_acceleration_duration = int(_total_acceleration_duration / 1000.0); // To ms instead of ns

  _default_acceleration_curve_end_speed = _default_acceleration_curve[_default_acceleration_curve_length-1]; // Set end speed variable to calculate speed factors of hand-specific curves.
}

void Clockception::calculate_corrected_curves() {
  // Multiply the default curve with a factor so each hand will finish at the exact same time, depending on the amount of steps to take. Due to this factor, end speed of each hand will differ.
  // If end speeds would be equal, each hand would finish at a different time.
  for(int i=0; i<_default_acceleration_curve_length; i++) {
      for(int hand=0; hand<nr_of_hands; hand++) {
        _acceleration_curves[hand][i] = int(_default_acceleration_curve[i]*hands[hand]->_acceleration_speed_factor);
      }
    }
}

bool Clockception::hands_finished() {
  // Check if all hands are finished
  bool finished = true;
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(!hands[hand]->movement_finished()) finished = false;
  }
  return finished;
}

void Clockception::disable_drivers() {
  digitalWrite(stepper_driver_reset, LOW);  
}

void Clockception::enable_drivers() {
  digitalWrite(stepper_driver_reset, HIGH);
}

void Clockception::reset_drivers() {
  disable_drivers();  
  delay(2);  // keep reset low min 1ms
  enable_drivers();
}

int Clockception::normalize(int value, int max, int min) {
  value = value % max;
  while(value < min) value += max;
  while(value >= max) value -= max;
  return value;
}

///////////////////////////////////////////////////////////////////////////////// ANIMATION UTILITY /////////////////////////////////////////////////////////////////////////

void Clockception::run_animation() {
  Serial.print("Run animation ");
  Serial.println(_current_animation);

  _time_start_animation = millis();  // Set start time to check for maximal execution time.
  
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->get_next_instruction(); // Get first instruction
  }

  while(!hands_finished()) { // Check if movement is complete

    for(int hand=0; hand<nr_of_hands; hand++) {
      hands[hand]->run_hand(); // Step hand if step is due
    }
    
    if(millis()-_time_start_animation >= 120000) { // Animation is running for more than a minute, force finish
      Serial.println(F("Animation is running longer than a minute, break out of loop"));
      for(int hand=0; hand<nr_of_hands; hand++) {
        hands[hand]->force_finished(); // Get first instruction
      }
    }

  }
  
  // Movement complete, reset instructions
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->clear_instructions(); // Clear instruction memory of all hands.
  }
}

void Clockception::set_direction_of_all_hands(bool direction) {
  for(int hand=0; hand<nr_of_hands; hand++) hands[hand]->set_direction(direction);
}

void Clockception::set_shortest_direction_to_target() {
  for(int hand=0; hand<nr_of_hands; hand++) {
    // Calculate CW steps to target
    int steps_cw = hands[hand]->target_position - hands[hand]->virtual_position;
    while(steps_cw < 0) steps_cw += steps_per_revolution;

    // If smaller than half a revolution, set CW direction
    if(steps_cw <= int(.5*steps_per_revolution)) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
  }
}

void Clockception::calculate_steps_to_positions(char extra_rotations) {
  _max_steps_to_take = 0;
  _min_steps_to_take = 65535; // Full unsigned int

  for(int hand=0; hand<nr_of_hands; hand++) {

    // Determine for each hand how many steps should be taken to reach end position
    if(hands[hand]->virtual_direction == CW) {
      // CW movement
      if(hands[hand]->target_position >= hands[hand]->virtual_position) {
        // Target lies ahead of current virtual position (in line with direction)
        hands[hand]->steps_to_take = hands[hand]->target_position - hands[hand]->virtual_position;
      }
      else {
        // Target lies back of current virtual position
        hands[hand]->steps_to_take = hands[hand]->target_position - hands[hand]->virtual_position + steps_per_revolution;
      }
    }
    else {
      // CCW movement
      if(hands[hand]->target_position <= hands[hand]->virtual_position) {
        // Target lies back of current virtual position (in line with direction)
        hands[hand]->steps_to_take = hands[hand]->virtual_position - hands[hand]->target_position;
      }
      else {
        // Target lies ahead of current virtual position
        hands[hand]->steps_to_take = hands[hand]->virtual_position - hands[hand]->target_position + steps_per_revolution;
      }

    }

    hands[hand]->steps_to_take += extra_rotations*steps_per_revolution; // Account for extra rotations

    // Set minimum and maximum steps to take (for calculation of relative speeds)
    if(hands[hand]->steps_to_take > _max_steps_to_take) _max_steps_to_take = hands[hand]->steps_to_take;
    if(hands[hand]->steps_to_take < _min_steps_to_take) _min_steps_to_take = hands[hand]->steps_to_take;

  }
}

void Clockception::calculate_animation_with_delays(char extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction, bool delay_at_start) {
  calculate_steps_to_positions(extra_rotations);
  int speed = int(1000000/max_speed); // Set speed from steps per time unit to step_interval;

  for(int hand=0; hand<nr_of_hands; hand++) {
    
    
    if(hands[hand]->steps_to_take != 0) {

      int steps_to_delay = _max_steps_to_take - hands[hand]->steps_to_take; // Since hands will have exactly same speed, each hand should wait for de hand with the longest way to go.
      //unsigned int start_delay = int(steps_to_delay/float(1000)*speed); // Time in ms hand can wait to start to reach end at same time and same speed
      // if(hand == 0) Serial.println((String)"Steps to take: "+hands[hand]->steps_to_take);
      unsigned int steps_remaining = hands[hand]->steps_to_take;
      unsigned int steps_accelerating = 0;
      unsigned int steps_cruising = 0;
      
      if(steps_to_delay > 0 && delay_at_start) {
        // if(hand == 0) Serial.println("Delay");
        hands[hand]->set_instruction(DELAY, steps_to_delay, speed); // Program delay, if delay needed at start
      }
      
      if(accel_fraction > 0) {
        // if(hand == 0) Serial.println((String)"Min steps to take: "+_min_steps_to_take);
        // if(hand == 0) Serial.println((String)"cruising_fraction: "+cruising_fraction);

        if(delay_at_start) steps_accelerating = int(_min_steps_to_take*accel_fraction);
        else steps_accelerating = int(hands[hand]->steps_to_take*accel_fraction);
       
        if(steps_accelerating > 0) {
          // if(hand == 0) Serial.println("Accel");
          hands[hand]->set_instruction(ACCELERATE, steps_accelerating, speed); // Accelerate, leave steps for decel
        }
        
        steps_remaining -= steps_accelerating;
        hands[hand]->_acceleration_speed_factor = speed/float(_default_acceleration_curve_end_speed); // All hands will accelerate at same speed
        hands[hand]->_accel_speed = speed;
      }
      
      if(decel_fraction > 0) {
        if(delay_at_start) steps_cruising = int(_min_steps_to_take*(1-accel_fraction-decel_fraction) + hands[hand]->steps_to_take-_min_steps_to_take);
        else steps_cruising = int(hands[hand]->steps_to_take*(1-accel_fraction-decel_fraction));
      }
      else steps_cruising = steps_remaining;
      // if(hand == 0) Serial.println((String)"Steps cruising: "+steps_cruising);
      if(steps_cruising > 0) {
        // if(hand == 0) Serial.println("Cruise");
        hands[hand]->set_instruction(CRUISE, steps_cruising, speed); // Cruise
      }


      if(decel_fraction > 0) {
        
        steps_remaining -= steps_cruising;
        hands[hand]->_accel_vs_decel_speed_factor = 1;
        // if(hand == 0) Serial.println((String)"Steps decel: "+steps_remaining);
        // if(hand == 0) Serial.println("Decel");
        hands[hand]->set_instruction(DECELERATE, steps_remaining, speed); // Decelerate with steps left from accel and cruise
      }

      if(steps_to_delay > 0 && !delay_at_start) {
        
        // if(hand == 0) Serial.println("Delay end");
        hands[hand]->set_instruction(DELAY, steps_to_delay, speed);
      }

    }

    
  }
    
  calculate_corrected_curves(); // Precalculate the acceleration curve that leads to desired end speed (still same length as original curve)
}

void Clockception::calculate_animation_equal_duration(char extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction) {
  calculate_steps_to_positions(extra_rotations);

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hands[hand]->steps_to_take != 0) { // Calculate only if hands needs to take 1 or more steps.

      int speed = hands[hand]->steps_to_take/float(_max_steps_to_take)*max_speed; // Set speed based on relative steps to take from max steps to take (to end at same time)
      speed = int(1000000/speed); // Set speed from steps per time unit to step_interval;

      unsigned int steps_remaining = hands[hand]->steps_to_take; // Steps that need to be incorporated in an instruction
      unsigned int steps_accelerating = 0;
      unsigned int steps_cruising = 0;
      
      if(accel_fraction > 0) {
        steps_accelerating = int(hands[hand]->steps_to_take*accel_fraction); // Steps accelerating = steps not cruising

        hands[hand]->set_instruction(ACCELERATE, steps_accelerating, speed); // Program acceleration part.
        steps_remaining -= steps_accelerating;

        /* Speed depends on steps to take relative to maximum steps to take. Since all hands should arrive at the finish at the same time, the acceleration curve 
        should also be corrected for this speed. */
        float _acceleration_speed_factor = speed/float(_default_acceleration_curve_end_speed);
        hands[hand]->_acceleration_speed_factor = _acceleration_speed_factor;
        hands[hand]->_accel_speed = speed; // Save this speed, so later an deceleration speed factor can be calculated
      }
      
      if(decel_fraction > 0) steps_cruising = int(hands[hand]->steps_to_take*(1-accel_fraction-decel_fraction));
      else steps_cruising = steps_remaining; // Should be equal to line above, but accounts for rounding differences 
      
      if(steps_cruising > 0) hands[hand]->set_instruction(CRUISE, steps_cruising, speed); // Cruise
      steps_remaining -= steps_cruising; 
      
      if(decel_fraction > 0) {
        hands[hand]->set_instruction(DECELERATE, steps_remaining, speed); // Decelerate with steps left from accel and cruise
        hands[hand]->_accel_vs_decel_speed_factor = speed/float(hands[hand]->_accel_speed); // Calculate speed factor of deceleration in relation to acceleration, 
        // Since only an corrected acceleration curve is computed, not for deceleration.
      }
      
    }
    
  }
  
  calculate_corrected_curves(); // Precalculate the acceleration curve that leads to desired end speed (still same length as original curve)
}

void Clockception::calculate_run_with_same_speed(unsigned int steps, unsigned int speed) {
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->set_instruction(CRUISE, steps, int(1000000/speed)); // Cruise
  }
}

void Clockception::calculate_run_with_speed(int *types, unsigned int *steps, unsigned int *speeds) {
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(types[hand] == CRUISE) hands[hand]->set_instruction(CRUISE, steps[hand], int(1000000/speeds[hand])); // Cruise
    else if (types[hand] == DELAY) hands[hand]->set_instruction(DELAY, steps[hand], int(1000000/speeds[hand])); // Cruise
  }
}

///////////////////////////////////////////////////////////////////////////////// LONG ANIMATIONS /////////////////////////////////////////////////////////////////////////

void Clockception::animation_long_1() { // Strech_and_turn
  set_direction_of_all_hands(CW);
  
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 1) hands[hand]->target_position = int(steps_per_revolution*.5);
    else hands[hand]->target_position = steps_per_revolution;
  }
  
  unsigned int max_speed = 600;
  // Calculate and set instructions for to stretch movement
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.9, /*decel*/ 0.0, /*delay at start*/ true);

  // Rotations
  calculate_run_with_same_speed(steps_per_revolution*2, max_speed);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  
  run_animation();
}

void Clockception::animation_long_2() { // Opposite rotation
  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
    hands[hand]->target_position = 0;
  }
  
  unsigned int max_speed = 800;
  
  calculate_animation_with_delays(/*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true);
  // Rotate
  
  calculate_run_with_same_speed(steps_per_revolution*2, max_speed);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  
  run_animation();
}

void Clockception::animation_long_3_to_birds() {
  for(int hand = 0; hand<nr_of_hands; hand++) {
    long offset = 0;
    if(hand%2 == 0) {
      long offset = random(-int(0.08*steps_per_revolution), int(0.08*steps_per_revolution));
      hands[hand]->set_direction(CW);
      hands[hand]->target_position = int(.85*steps_per_revolution) + offset;
    }
    else {
      hands[hand]->set_direction(CCW);
      hands[hand]->target_position = int(.15*steps_per_revolution) + offset;
    }
  }
}

void Clockception::animation_long_3_to_bottom() {
  long offset = 0;
  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) {
      offset = random(-int(0.08*steps_per_revolution), int(0.08*steps_per_revolution));
      hands[hand]->set_direction(CCW);
      hands[hand]->target_position = int(.65*steps_per_revolution) + offset;
    }
    else {
      hands[hand]->set_direction(CW);
      hands[hand]->target_position = int(.35*steps_per_revolution) + offset;
    }
  } 
}

void Clockception::animation_long_3() { // Bird shapes

  animation_long_3_to_birds();
  
  unsigned int max_speed = 1000;
  // To bird shape
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
  
  // Loop 3 times
  for(int i=0; i<3; i++) {

    // Get back to bottom    
    animation_long_3_to_bottom();
    calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);
    run_animation();

    // Back to bird shape  
    animation_long_3_to_birds();
    calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);
    run_animation();
  }


  // Get back to bottom and further to time
  
  animation_long_3_to_bottom();
  //calculate_animation_same_start(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ true, /*decel*/ 0.0, /*cruise fraction*/ .2);
  
  // Show time
  show_time_equal_duration(/*time already fetched*/ 98, 98, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.1, /*decel*/ 0.1);
  
  run_animation();
}

void Clockception::animation_long_4() { // Opposite rotation with different speeds
  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
    hands[hand]->target_position = 0;
  }
  
  unsigned int max_speed = 800;
  
  // Should use calculate_animation_with_delays and different speeds
  calculate_animation_with_delays(/*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true);
  // Rotate

  unsigned int steps[nr_of_hands];
  unsigned int speeds[nr_of_hands];
  int types[nr_of_hands];

  for(int hand = 0; hand<nr_of_hands; hand++) {
    types[hand] = CRUISE;
    if(hand%2 == 0) {
      steps[hand] = steps_per_revolution*1.5;
      speeds[hand] = int(1.15/2.0*max_speed);
    }
    else {
      steps[hand] = steps_per_revolution*2;
      speeds[hand] = max_speed;
    }
  }

  calculate_run_with_speed(types, steps, speeds);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.4);
  
  run_animation();
}

void Clockception::animation_long_5() { // Opposite rotation oriented to center simultaniously

  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0 || hand >= 16) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);

    if(hand<=1) hands[hand]->target_position = steps_per_revolution*.5;
    else if(hand<=3) hands[hand]->target_position = int(steps_per_revolution*.625);
    else if(hand<=5) hands[hand]->target_position = int(steps_per_revolution*.75);
    else if(hand<=7) hands[hand]->target_position = int(steps_per_revolution*.875);
    else if(hand<=9) hands[hand]->target_position = int(steps_per_revolution);
    else if(hand<=11) hands[hand]->target_position = int(steps_per_revolution*.125);
    else if(hand<=13) hands[hand]->target_position = int(steps_per_revolution*.25);
    else if(hand<=15) hands[hand]->target_position = int(steps_per_revolution*.375);
    else if(hand==16) hands[hand]->target_position = int(steps_per_revolution);
    else if(hand==17) hands[hand]->target_position = int(steps_per_revolution*.5);
  }
  
  unsigned int max_speed = 800;
  
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true);
  // Rotate

  calculate_run_with_same_speed(steps_per_revolution*2, max_speed);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.4);
  
  run_animation();

}

void Clockception::animation_long_6() { // Frame turns in one minute  
  unsigned int steps[nr_of_hands];
  unsigned int speed[nr_of_hands];
  int types[nr_of_hands];

  for(int hand = 0; hand<nr_of_hands; hand++) {
    steps[hand] = steps_per_revolution; 
    speed[hand] = 80;
    types[hand] = CRUISE;
    if(hand%4 == 0 || hand%4 == 1) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
  }

  types[nr_of_hands-2] = DELAY;
  types[nr_of_hands-1] = DELAY;

  
  calculate_run_with_speed(types, steps, speed);

  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ 800, /*accel*/ 0.5, /*decel*/ 0.5);
}

void Clockception::animation_long_7() { 
  set_direction_of_all_hands(CW);
  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->target_position = 0;
    else hands[hand]->target_position = int(.5*steps_per_revolution);
  }
  
  unsigned int max_speed = 800;
  
  // This should be calculate_animation_with_delays and different end speeds

  unsigned int steps[nr_of_hands];
  unsigned int speeds[nr_of_hands];
  int types[nr_of_hands];
  float factor = 10.0;

  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand == 12 || hand == 13) steps[hand] = steps_per_revolution;
    else if(hand == 10 || hand == 11 || hand == 14 || hand == 15) steps[hand] = int(steps_per_revolution*1.5);
    else if(hand == 0 || hand == 1 || hand == 16 || hand == 17 || hand == 8 || hand == 9) steps[hand] = int(steps_per_revolution*2);
    else if(hand == 2 || hand == 3 || hand == 6 || hand == 7) steps[hand] = int(steps_per_revolution*2.5);
    else steps[hand] = int(steps_per_revolution*3);
    speeds[hand] = int(steps[hand]/factor);
    types[hand] = CRUISE;
  }

  calculate_animation_with_delays(/*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true);
  
  calculate_run_with_speed(types, steps, speeds);

  // Show time
  show_time_with_delays(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  
  run_animation();

}

void Clockception::animation_long_8() {
  int hour = _hour + 1;
  int minute = 0;
    
  int hour_in_steps = int(float(hour % 12) / 12 * float(steps_per_revolution) + floor(float(minute) / 60 / 12 * float(steps_per_revolution)));
  int minute_in_steps = int(minute / float(60) * steps_per_revolution);

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->target_position = hour_in_steps; // Set all hour hands to hour
    else hands[hand]->target_position = minute_in_steps; // Set all minute hands to minute
    hands[hand]->set_direction(CW);
  }

  unsigned int max_speed = 800;

  // This should be calculate_animation_with_delays and different end speeds
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true);
  
  unsigned int steps[nr_of_hands];
  unsigned int speed[nr_of_hands];
  int types[nr_of_hands];

  for(int hand = 0; hand<nr_of_hands; hand++) {
    types[hand] = CRUISE;
    // Run 10 hours
    if(hand%2 == 0) {
      steps[hand] = int(steps_per_revolution*10/12.0); 
      speed[hand] = int(max_speed/5.0);
    }
    else {
      steps[hand] = steps_per_revolution*5; 
      speed[hand] = max_speed;
    }
  }

  // Rotations
  calculate_run_with_speed(types, steps, speed);

  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.7); // Set extra rotations to 1 when having drift
  
  run_animation();
}

void Clockception::animation_long_9() { // Opposite rotation oriented to center after each other -> causes drift of the minute hand
  set_direction_of_all_hands(CW);
  for(int hand = 0; hand<nr_of_hands; hand++) {
    if(hand<=15) hands[hand]->target_position = int(steps_per_revolution*.5);
    else if(hand==16) hands[hand]->target_position = int(steps_per_revolution);
    else if(hand==17) hands[hand]->target_position = int(steps_per_revolution);
  }
  
  unsigned int max_speed = 800;
  
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true);
  // Rotate
  
  calculate_run_with_same_speed(steps_per_revolution*2, max_speed);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  
  run_animation();
}

void Clockception::animation_long_10() { // Strech_and_turn variation
  set_direction_of_all_hands(CW);
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 1) hands[hand]->target_position = int(steps_per_revolution*.75);
    else hands[hand]->target_position = int(steps_per_revolution*.25);

    if(hand == 10 || hand == 11 || hand == 14 || hand == 15) hands[hand]->target_position -= 120;
    else if(hand == 0 || hand == 1 || hand == 8 || hand == 9 || hand == 16 || hand == 17) hands[hand]->target_position -= 240;
    else if(hand == 2 || hand == 3 || hand == 6 || hand == 7) hands[hand]->target_position -= 360;
    else if(hand == 4 || hand == 5) hands[hand]->target_position -= 480;
  }
  
  unsigned int max_speed = 600;
  // Calculate and set instructions for to stretch movement
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.9, /*decel*/ 0.0, /*delay at start*/ true);

  // Rotations
  calculate_run_with_same_speed(steps_per_revolution*2, max_speed);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  
  run_animation();
}

void Clockception::animation_long_11() { // Stretch and each column opposite rotation
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 1) hands[hand]->target_position = 0;
    else hands[hand]->target_position = int(steps_per_revolution*.5);

    if(hand == 12 || hand == 13) hands[hand]->set_direction(CCW);
    else if(hand == 10 || hand == 11 || hand == 14 || hand == 15) hands[hand]->set_direction(CW);
    else if(hand == 0 || hand == 1 || hand == 8 || hand == 9 || hand == 16 || hand == 17) hands[hand]->set_direction(CCW);
    else if(hand == 2 || hand == 3 || hand == 6 || hand == 7) hands[hand]->set_direction(CW);
    else if(hand == 4 || hand == 5) hands[hand]->set_direction(CCW);
  }
  
  unsigned int max_speed = 400;
  // Calculate and set instructions for to stretch movement
  calculate_animation_with_delays(/*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 0.6, /*decel*/ 0.0, /*delay at start*/ true);

  // Rotations
  calculate_run_with_same_speed(steps_per_revolution*2, max_speed);
  
  // Show time
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  
  run_animation();
}

void Clockception::animation_long_12() { // Subsequent rotation downwards
  // Rotate all hands upwards
  for(int hand=0; hand<nr_of_hands; hand++) hands[hand]->target_position = 0;
  set_shortest_direction_to_target();
  unsigned int max_speed = 400;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
  
  max_speed = 200;
  int speed = int(1000000/max_speed);
  // Program delays for each row of hands and then rotation downwards
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CCW);
    else hands[hand]->set_direction(CW);

    // Hands 0 and 1 no delay
    if(hand == 14 || hand == 15 || hand == 2 || hand == 3) hands[hand]->set_instruction(DELAY, int(0.25*steps_per_revolution), speed);
    else if(hand == 12 || hand == 13 || hand == 16 || hand == 17 || hand == 4 || hand == 5) hands[hand]->set_instruction(DELAY, int(0.5*steps_per_revolution), speed);
    else if(hand == 10 || hand == 11 || hand == 6 || hand == 7) hands[hand]->set_instruction(DELAY, int(0.75*steps_per_revolution), speed);
    
    if(hand == 8 || hand == 9) {
      // Set instructions for a full rotation
      hands[hand]->set_instruction(DELAY, int(steps_per_revolution), speed);
      hands[hand]->target_position = steps_per_revolution; // These hands continue rotation upwards
      hands[hand]->set_instruction(ACCELERATE, int(0.1*steps_per_revolution), speed);
      hands[hand]->set_instruction(CRUISE, int(0.8*steps_per_revolution), speed);
      hands[hand]->set_instruction(DECELERATE, int(0.1*steps_per_revolution), speed);
    }
    else {
      // For other hands half rotation
      hands[hand]->set_instruction(ACCELERATE, int(0.1*steps_per_revolution), speed);
      hands[hand]->set_instruction(CRUISE, int(0.3*steps_per_revolution), speed);
      hands[hand]->set_instruction(DECELERATE, int(0.1*steps_per_revolution), speed);
    }

    hands[hand]->_acceleration_speed_factor = speed/float(_default_acceleration_curve_end_speed); // All hands will accelerate at same speed
    hands[hand]->_accel_speed = speed;
    hands[hand]->_accel_vs_decel_speed_factor = 1;
  }
    
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->virtual_position = 0;
    
    if(hand == 8 || hand == 9) continue; // These hands do a full rotation, so do not need extra instructions
    if(hand == 10 || hand == 11 || hand == 6 || hand == 7) hands[hand]->set_instruction(DELAY, int(0.35*steps_per_revolution), speed);
    else if(hand == 12 || hand == 13 || hand == 16 || hand == 17 || hand == 4 || hand == 5) hands[hand]->set_instruction(DELAY, int(0.85*steps_per_revolution), speed);
    else if(hand == 14 || hand == 15 || hand == 2 || hand == 3) hands[hand]->set_instruction(DELAY, int(1.35*steps_per_revolution), speed);
    else if(hand == 0 || hand == 1) hands[hand]->set_instruction(DELAY, int(1.85*steps_per_revolution), speed);
  
    hands[hand]->set_instruction(ACCELERATE, int(0.1*steps_per_revolution), speed);
    hands[hand]->set_instruction(CRUISE, int(0.3*steps_per_revolution), speed);
    hands[hand]->set_instruction(DECELERATE, int(0.1*steps_per_revolution), speed);    
  }

  calculate_corrected_curves();
  run_animation();
  
  // Show time
  set_time_and_frame_positions();
  set_shortest_direction_to_target();

  max_speed = 600;
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.3, /*decel*/ 0.3);
  
  run_animation();
}

void Clockception::animation_long_13() { // Splash animation
  // All hands to zero
  for(int hand=0; hand<nr_of_hands; hand++) hands[hand]->target_position = 0;
  set_shortest_direction_to_target();
  unsigned int max_speed = 400;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();

  delay(1000);

  // Set directions and splash down
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->target_position = int(steps_per_revolution*0.5);
    if(hand%2 == 0 && hands[hand]->direction == CW) hands[hand]->set_direction(CCW);
    else if(hand%2 == 1 && hands[hand]->direction == CCW) hands[hand]->set_direction(CW);

    int wait_time = 500;
    if(hand==2 || hand==3 || hand==14 || hand==15) hands[hand]->set_instruction(DELAY, wait_time, max_speed);
    if(hand==12 || hand==13 || hand==16 || hand==17 || hand==4 || hand==5) hands[hand]->set_instruction(DELAY, wait_time*2, max_speed);
    if(hand==6 || hand==7 || hand==10 || hand==11) hands[hand]->set_instruction(DELAY, wait_time*3, max_speed);
    if(hand==8 || hand==9) hands[hand]->set_instruction(DELAY, wait_time*4, max_speed);
  }

  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.2, /*decel*/ 0.0);

  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.2);
  run_animation();
}

///////////////////////////////////////////////////////////////////////////////// SHORT ANIMATIONS /////////////////////////////////////////////////////////////////////////

void Clockception::animation_short_1() { // Turn all hands equally
  set_direction_of_all_hands(CW);
  
  int max_speed= 400;
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 1, /*max_speed*/ max_speed, /*accel*/ 0.1, /*decel*/ 0.2);
  run_animation();
}

void Clockception::animation_short_2() { // One revolution different start and end times
  _current_animation = SHORT_2;
  int max_speed= 800;
  set_direction_of_all_hands(CW);
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->target_position = int(steps_per_revolution*.25);
  }

  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true); 
  show_time_with_delays(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.0, /*decel*/ 0.7);
  run_animation();
}

void Clockception::animation_short_3() { // Turn hands outside and back

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CCW);
    else hands[hand]->set_direction(CW);
    hands[16]->set_direction(CW);
    hands[17]->set_direction(CW);

    if(hand<=1) hands[hand]->target_position = steps_per_revolution;
    else if(hand<=3) hands[hand]->target_position = int(steps_per_revolution*.125);
    else if(hand<=5) hands[hand]->target_position = int(steps_per_revolution*.25);
    else if(hand<=7) hands[hand]->target_position = int(steps_per_revolution*.375);
    else if(hand<=9) hands[hand]->target_position = int(steps_per_revolution*.5);
    else if(hand<=11) hands[hand]->target_position = int(steps_per_revolution*.625);
    else if(hand<=13) hands[hand]->target_position = int(steps_per_revolution*.75);
    else if(hand<=15) hands[hand]->target_position = int(steps_per_revolution*.875);
    set_time_positions();
  }
  
  int max_speed= 500;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);
  run_animation();

  Serial.println(F("Wait for new minute"));
  _last_minute = _minute; // Set this now so wait_for_new_minute() works properly
  wait_for_new_minute();

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
  }
  hands[nr_of_hands-2]->set_direction(CW);
  hands[nr_of_hands-1]->set_direction(CW);
  
  set_clock_frame_positions();
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);

  show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 0, /*max_speed*/ 50, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
}

void Clockception::animation_short_4() { // Create a wave through the frame, uses custom instructions
  byte frame_order[nr_of_hands-2] = {0,3,2,5,4,7,6,9,8,11,10,13,12,15,14,1};

  int step_interval = 8000;
  int angle = int(.012 * steps_per_revolution);
  int delay = int(angle/2);
  for(int hand = 0; hand<nr_of_hands-2; hand++) {
    
    if(hand == nr_of_hands-2) hands[frame_order[hand]]->set_instruction(DELAY, delay*(nr_of_hands-2), step_interval); // Hand 0 is the last one, so delay the longest
    else hands[frame_order[hand]]->set_instruction(DELAY, delay*hand, step_interval);
    
    if(hand%4 == 1 || hand%4 == 2) hands[frame_order[hand]]->set_direction(CCW);
    else hands[frame_order[hand]]->set_direction(CW);

    hands[frame_order[hand]]->_acceleration_speed_factor = 5;
    hands[frame_order[hand]]->_accel_vs_decel_speed_factor = 1;

    // Move away
    hands[frame_order[hand]]->set_instruction(ACCELERATE, angle, step_interval);
    hands[frame_order[hand]]->set_instruction(DECELERATE, angle, step_interval);

    // Move to other side
    hands[frame_order[hand]]->set_instruction(SWITCH_DIRECTION, 0, 0);
    hands[frame_order[hand]]->set_instruction(ACCELERATE, int(4*angle), step_interval);
    hands[frame_order[hand]]->set_instruction(DECELERATE, int(4*angle), step_interval);
    
    // Move back
    hands[frame_order[hand]]->set_instruction(SWITCH_DIRECTION, 0, 0);
    hands[frame_order[hand]]->set_instruction(ACCELERATE, int(2*angle), step_interval);
    hands[frame_order[hand]]->set_instruction(DECELERATE, int(2*angle), step_interval);
  }

  // Set hands in right position
  set_time_and_frame_positions();
  for(int hand = nr_of_hands-2; hand < nr_of_hands; hand++) {
    int steps_to_take = normalize(hands[hand]->target_position - hands[hand]->virtual_position, steps_per_revolution, 0);
    if(steps_to_take < int(0.5*steps_per_revolution)) hands[hand]->set_direction(CW);
    else {
      steps_to_take = steps_per_revolution - steps_to_take;
      hands[hand]->set_direction(CCW);
    }
    hands[hand]->set_instruction(CRUISE, steps_to_take, 8000);
  }

  calculate_corrected_curves();
  run_animation();
}

void Clockception::animation_short_5() { // Rotation with random delay and speed
  for(int hand = 0; hand<nr_of_hands; hand++) {
    //hands[hand]->set_instruction(DELAY, random(1, 3000), 800);
    if(random(0,2) == 0) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
    hands[hand]->target_position = random(1000, 3000);
  }
  
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ 600, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true); 
  show_time_with_delays(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ 600, /*accel*/ 0.0, /*decel*/ 1.0);
  run_animation();
}

void Clockception::animation_short_6() { // Turn hands inwards and back
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
    hands[16]->set_direction(CW);
    hands[17]->set_direction(CW);

    if(hand<=1) hands[hand]->target_position = int(steps_per_revolution*.5);
    else if(hand<=3) hands[hand]->target_position = int(steps_per_revolution*.625);
    else if(hand<=5) hands[hand]->target_position = int(steps_per_revolution*.75);
    else if(hand<=7) hands[hand]->target_position = int(steps_per_revolution*.875);
    else if(hand<=9) hands[hand]->target_position = steps_per_revolution; 
    else if(hand<=11) hands[hand]->target_position = int(steps_per_revolution*.125);
    else if(hand<=13) hands[hand]->target_position = int(steps_per_revolution*.25);
    else if(hand<=15) hands[hand]->target_position = int(steps_per_revolution*.375);
    set_time_positions();
  }
  
  int max_speed= 500;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);
  run_animation();

  Serial.println(F("Wait for new minute"));
  _last_minute = _minute; // Set this now so wait_for_new_minute() works properly
  wait_for_new_minute();

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%2 == 0) hands[hand]->set_direction(CCW);
    else hands[hand]->set_direction(CW);
  }
  hands[nr_of_hands-2]->set_direction(CW);
  hands[nr_of_hands-1]->set_direction(CW);
  
  set_clock_frame_positions();
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);

  show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 0, /*max_speed*/ 50, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
}

void Clockception::animation_short_7() { // Turn corners first 
  int speed = 600; 
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%4 >= 2) {
      hands[hand]->set_instruction(DELAY, int(steps_per_revolution/2), int(1000000/speed));
    }
  }
  
  show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 1, /*max_speed*/ speed, /*accel*/ 0.15, /*decel*/ 0.15);
  run_animation();
}

void Clockception::animation_short_8() { // Corners out, straigts in
  for(int hand=0; hand<nr_of_hands; hand++) {

    if(hand%4 == 1 || hand%4 == 2) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);

    if(hand<=1) hands[hand]->target_position = steps_per_revolution;
    else if(hand<=3) hands[hand]->target_position = int(steps_per_revolution*.625);
    else if(hand<=5) hands[hand]->target_position = int(steps_per_revolution*.25);
    else if(hand<=7) hands[hand]->target_position = int(steps_per_revolution*.875);
    else if(hand<=9) hands[hand]->target_position = int(steps_per_revolution*0.5); 
    else if(hand<=11) hands[hand]->target_position = int(steps_per_revolution*.125);
    else if(hand<=13) hands[hand]->target_position = int(steps_per_revolution*.75);
    else if(hand<=15) hands[hand]->target_position = int(steps_per_revolution*.375);
    else hands[hand]->target_position = hands[hand]->current_position;
  }
  
  int max_speed= 500;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);
  run_animation();

  delay(1500); // Wait 2 seconds

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%4 == 1 || hand%4 == 2) hands[hand]->set_direction(CCW);
    else hands[hand]->set_direction(CW);
  }
  hands[nr_of_hands-2]->set_direction(CW);
  hands[nr_of_hands-1]->set_direction(CW);
  
  set_clock_frame_positions();
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);

  show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 0, /*max_speed*/ 50, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
}

void Clockception::animation_short_9() { // Corners in, straigts out
  for(int hand=0; hand<nr_of_hands; hand++) {

    if(hand%4 == 1 || hand%4 == 2) hands[hand]->set_direction(CCW);
    else hands[hand]->set_direction(CW);

    if(hand<=1) hands[hand]->target_position = int(steps_per_revolution*0.5);
    else if(hand<=3) hands[hand]->target_position = int(steps_per_revolution*.125);
    else if(hand<=5) hands[hand]->target_position = int(steps_per_revolution*.75);
    else if(hand<=7) hands[hand]->target_position = int(steps_per_revolution*.375);
    else if(hand<=9) hands[hand]->target_position = int(steps_per_revolution); 
    else if(hand<=11) hands[hand]->target_position = int(steps_per_revolution*.625);
    else if(hand<=13) hands[hand]->target_position = int(steps_per_revolution*.25);
    else if(hand<=15) hands[hand]->target_position = int(steps_per_revolution*.875);
    else hands[hand]->target_position = hands[hand]->current_position;
  }
  
  int max_speed= 500;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);
  run_animation();

  Serial.println(F("Wait for new minute"));
  _last_minute = _minute; // Set this now so wait_for_new_minute() works properly
  wait_for_new_minute();

  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand%4 == 1 || hand%4 == 2) hands[hand]->set_direction(CW);
    else hands[hand]->set_direction(CCW);
  }
  hands[nr_of_hands-2]->set_direction(CW);
  hands[nr_of_hands-1]->set_direction(CW);
  
  set_clock_frame_positions();
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.4, /*decel*/ 0.4);

  show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 0, /*max_speed*/ 50, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
}

void Clockception::animation_short_10() { // All hands flat
  int left = int(steps_per_revolution*.75);
  int right = int(steps_per_revolution*.25);

  hands[0]->target_position = right;
  hands[1]->target_position = left;
  hands[2]->target_position = right;
  hands[3]->target_position = left;
  hands[4]->target_position = left;
  hands[5]->target_position = left;
  hands[6]->target_position = left;
  hands[7]->target_position = right;
  hands[8]->target_position = left;
  hands[9]->target_position = right;
  hands[10]->target_position = left;
  hands[11]->target_position = right;
  hands[12]->target_position = right;
  hands[13]->target_position = right;
  hands[14]->target_position = right;
  hands[15]->target_position = left;

  // Determine distance to right side
  int hour_to_right = abs(right - hands[16]->current_position);
  if(hour_to_right > 0.5*steps_per_revolution) hour_to_right = steps_per_revolution - hour_to_right;

  int minute_to_right = abs(right - hands[17]->current_position);
  if(minute_to_right > 0.5*steps_per_revolution) minute_to_right = steps_per_revolution - minute_to_right;

  // Determine which hand is closest to right side
  if(hour_to_right <= minute_to_right) {
    hands[16]->target_position = right;
    hands[17]->target_position = left;
  }
  else {
    hands[16]->target_position = left;
    hands[17]->target_position = right;
  }

  set_shortest_direction_to_target();
  
  int max_speed= 500;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();

  delay(1500); // Wait 1500ms

  get_time();
  set_time_and_frame_positions();
  set_shortest_direction_to_target();

  show_time_equal_duration(/*time already fetched*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
}

void Clockception::animation_short_11() { // All hands straight
  int top = 0;
  int down = int(steps_per_revolution*.5);

  hands[0]->target_position = down;
  hands[1]->target_position = down;
  hands[2]->target_position = down;
  hands[3]->target_position = top;
  hands[4]->target_position = down;
  hands[5]->target_position = top;
  hands[6]->target_position = down;
  hands[7]->target_position = top;
  hands[8]->target_position = top;
  hands[9]->target_position = top;
  hands[10]->target_position = top;
  hands[11]->target_position = down;
  hands[12]->target_position = top;
  hands[13]->target_position = down;
  hands[14]->target_position = top;
  hands[15]->target_position = down;

  // Determine distance to top side
  int hour_to_top = hands[16]->current_position;
  if(hour_to_top > 0.5*steps_per_revolution) hour_to_top = steps_per_revolution - hour_to_top;

  int minute_to_top = hands[17]->current_position;
  if(minute_to_top > 0.5*steps_per_revolution) minute_to_top = steps_per_revolution - minute_to_top;

  // Determine which hand is closest to top side
  if(hour_to_top <= minute_to_top) {
    hands[16]->target_position = top;
    hands[17]->target_position = down;
  }
  else {
    hands[16]->target_position = down;
    hands[17]->target_position = top;
  }

  set_shortest_direction_to_target();
  
  int max_speed= 500;
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();

  delay(1500); // Wait 1500ms

  get_time();
  set_time_and_frame_positions();
  set_shortest_direction_to_target();

  show_time_equal_duration(/*time already fetched*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);

  run_animation();

}

void Clockception::animation_short_12() { // One rotation after each other
  int speed = 900;
  set_direction_of_all_hands(CW);
  for(int hand=0; hand<nr_of_hands; hand++) {
    hands[hand]->set_instruction(DELAY, int(steps_per_revolution/4*hand), int(1000000/speed));
  }

  show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 1, /*max_speed*/ speed, /*accel*/ 0.5, /*decel*/ 0.5);
  run_animation();
}

void Clockception::animation_short_13() { // One rotation, each row after another
  int speed = 900;
  set_direction_of_all_hands(CW);
  for(int hand=0; hand<nr_of_hands; hand++) {
    if(hand == 2 || hand == 3 || hand == 14 || hand == 15) hands[hand]->set_instruction(DELAY, int(steps_per_revolution/3), int(1000000/speed));
    else if(hand == 4 || hand == 5 || hand == 12 || hand == 13 || hand == 16 || hand == 17) hands[hand]->set_instruction(DELAY, int(steps_per_revolution/3*2), int(1000000/speed));
    else if(hand == 6 || hand == 7 || hand == 10 || hand == 11) hands[hand]->set_instruction(DELAY, int(steps_per_revolution/3*3), int(1000000/speed));
    else if(hand == 8 || hand == 9) hands[hand]->set_instruction(DELAY, int(steps_per_revolution/3*4), int(1000000/speed));
    hands[hand]->target_position = int(steps_per_revolution*0.5);
  }

  //show_time_equal_duration(/*get current time*/ 98, 98, /*extra rotations*/ 1, /*max_speed*/ speed, /*accel*/ 0.5, /*decel*/ 0.5);
  
  calculate_animation_with_delays(/*extra rotations*/ 0, /*max_speed*/ speed, /*accel*/ 1.0, /*decel*/ 0.0, /*delay at start*/ true); 
  show_time_with_delays(/*get current time*/ 98, 98, /*extra rotations*/ 0, /*max_speed*/ speed, /*accel*/ 0.0, /*decel*/ 1.0);
  run_animation();
}



///////////////////////////////////////////////////////////////////////////////// PARTIAL ANIMATIONS /////////////////////////////////////////////////////////////////////////

void Clockception::animation_to_zero() {
  for(int hand = 0; hand<nr_of_hands; hand++) hands[hand]->target_position = 0;
  unsigned int max_speed = 1000;
  
  set_shortest_direction_to_target();
  // Rotate
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  
  run_animation();
}

void Clockception::animation_to_bottom() {
  for(int hand = 0; hand<nr_of_hands; hand++) {
    hands[hand]->target_position = int(0.5*steps_per_revolution);
    hands[hand]->set_direction(CW);
  }
  unsigned int max_speed = 1000;

    // Rotate
  calculate_animation_equal_duration(/*extra rotations*/ 0, /*max_speed*/ max_speed, /*accel*/ 0.5, /*decel*/ 0.5);
  
  run_animation();
}

///////////////////////////////////////////////////////////////////////////////// SHOW TIME /////////////////////////////////////////////////////////////////////////

void Clockception::get_time() {
  DateTime now = rtc->now();
  _hour = now.hour();
  _minute = now.minute();
  _second = now.second();
}

void Clockception::set_time_and_frame_positions() {
  set_clock_frame_positions();
  set_time_positions();
}

void Clockception::set_clock_frame_positions() {
  for(int hand=0; hand<nr_of_hands-2; hand++) {
    hands[hand]->target_position = clock_frame_positions[hand]; // Set frame as defined in settings
  }
}

void Clockception::set_time_positions() {
 // Calculate the time positons in steps
  int hour_in_steps = int(float(_hour % 12) / 12 * float(steps_per_revolution) + floor(float(_minute) / 60 / 12 * float(steps_per_revolution)));
  int minute_in_steps = int(_minute / float(60) * steps_per_revolution);

  hands[nr_of_hands-2]->target_position = hour_in_steps; // Set current hour position
  hands[nr_of_hands-1]->target_position = minute_in_steps; // Set current minute position
}


void Clockception::show_time_equal_duration(uint8_t predefined_hour, uint8_t predefined_minute, int extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction) {
  // Get time
  if(predefined_hour == 99 && predefined_minute == 99) {
    get_time();
  }
  else if(predefined_hour == 98 && predefined_minute == 98) {
    // Time was fetched earlier, now not needed
  }
  else {
    _hour = predefined_hour;
    _minute = predefined_minute;
  }
  
  // Set time to target positions
  set_time_and_frame_positions();

  // Corrections to improve the visuals of different animations
  if(_current_animation == LONG_1) { // Corrections for stretch & turn
    for(int hand=0; hand<nr_of_hands; hand++) {
      while(hands[hand]->target_position - hands[hand]->virtual_position < int(1*steps_per_revolution)) hands[hand]->target_position += steps_per_revolution;
      while(hands[hand]->target_position - hands[hand]->virtual_position >= int(2*steps_per_revolution)) hands[hand]->target_position -= steps_per_revolution;
    }
  }

  if(_current_animation == LONG_2 || _current_animation == LONG_3) { // Corrections for opposite rotation or after birds
    for(int hand=0; hand<nr_of_hands; hand++) {
      if(hands[hand]->direction == CW) {
        while(hands[hand]->target_position - hands[hand]->virtual_position < int(1*steps_per_revolution)) hands[hand]->target_position += steps_per_revolution;
        while(hands[hand]->target_position - hands[hand]->virtual_position >= int(2*steps_per_revolution)) hands[hand]->target_position -= steps_per_revolution;
      }
      else {
        while(hands[hand]->virtual_position - hands[hand]->target_position < int(1*steps_per_revolution)) hands[hand]->target_position -= steps_per_revolution;
        while(hands[hand]->virtual_position - hands[hand]->target_position >= int(2*steps_per_revolution)) hands[hand]->target_position += steps_per_revolution;
      }
    }
  }

  if(_current_animation == LONG_4) {
    for(int hand=0; hand<nr_of_hands; hand++) {
      if(hands[hand]->direction == CW) while(hands[hand]->target_position - hands[hand]->virtual_position < int(1*steps_per_revolution)) hands[hand]->target_position += steps_per_revolution;
      else while(hands[hand]->virtual_position - hands[hand]->target_position < int(1*steps_per_revolution)) hands[hand]->target_position -= steps_per_revolution;
    }

  }

  if(_current_animation == LONG_5) {
    if(hands[nr_of_hands-2]->target_position - hands[nr_of_hands-2]->virtual_position < int(.33*steps_per_revolution)) hands[nr_of_hands-2]->target_position += steps_per_revolution;
    if(hands[nr_of_hands-1]->target_position - hands[nr_of_hands-1]->virtual_position < int(.33*steps_per_revolution)) hands[nr_of_hands-1]->target_position += steps_per_revolution;
  }

  calculate_animation_equal_duration(/*extra rotations*/ extra_rotations, /*max_speed*/ max_speed, /*accel*/ accel_fraction, /*decel*/  decel_fraction);
}

void Clockception::show_time_with_delays(uint8_t predefined_hour, uint8_t predefined_minute, int extra_rotations, unsigned int max_speed, float accel_fraction, float decel_fraction) {
  // Get time
  if(predefined_hour == 99 && predefined_minute == 99) {
    get_time();
  }
  else if(predefined_hour == 98 && predefined_minute == 98) {
    // Time was fetched earlier, now not needed
  }
  else {
    _hour = predefined_hour;
    _minute = predefined_minute;
  }
  
  // Set time to target positions
  set_time_and_frame_positions();
  
  if(_current_animation == LONG_7) {
    for(int hand=0; hand<nr_of_hands; hand++) {
      if(hands[hand]->target_position - hands[hand]->virtual_position < int(.33*steps_per_revolution)) hands[hand]->target_position += steps_per_revolution;
    }
  }

  calculate_animation_with_delays(/*extra rotations*/ extra_rotations, /*max_speed*/ max_speed, /*accel*/ accel_fraction, /*decel*/ decel_fraction, /*delay at start*/ false);
}

///////////////////////////////////////////////////////////////////////////////// SETTINGS /////////////////////////////////////////////////////////////////////////

void Clockception::set_settings() {
  Serial.println(F("Set settings, first get all hands up to enable upload new program"));
  animation_to_zero();

  Serial.println(F("Wait for button press to show frame to enable fine adjustment of hands"));  
  while(!button_set->pushed()) delay(1); // Wait for button press
  
  set_time();

  Serial.println(F("Settings complete, resume program"));

  // Show time in an extra rotation
  set_direction_of_all_hands(CW);
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 1, /*max_speed*/ 800, /*accel*/ 0.2, /*decel*/ 0.2);
  run_animation();

}

void Clockception::set_time_hour_back() {
  Serial.println(F("Set time hour back"));

  if(_hour <= 0) _hour = 23;
  else _hour--;

  rtc->adjust(DateTime(2000, 1, 1, _hour, _minute, _second)); // Write time to RTC

  set_time_positions();
  hands[nr_of_hands-2]->set_direction(CCW);
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ 800, /*accel*/ 0.2, /*decel*/ 0.2);
  run_animation();
}

void Clockception::set_time_hour_forward() {
  Serial.println(F("Set time hour forward"));

  if(_hour >= 23) _hour = 0;
  else _hour++;

  rtc->adjust(DateTime(2000, 1, 1, _hour, _minute, _second)); // Write time to RTC

  set_time_positions();
  hands[nr_of_hands-2]->set_direction(CW);
  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ 800, /*accel*/ 0.2, /*decel*/ 0.2);
  run_animation();
}

void Clockception::set_time() {
  Serial.println(F("Set time"));

  show_time_equal_duration(/*get current time*/ 99, 99, /*extra rotations*/ 0, /*max_speed*/ 800, /*accel*/ 0.2, /*decel*/ 0.2);
  run_animation();
  
  unsigned long previous_button_press = millis();
  unsigned long wait_time_between_buttons = 150;

  while(!button_set->pushed()) {
    bool change = false;

    if(button_forward->pushed() && (millis() - previous_button_press > wait_time_between_buttons)) {
      previous_button_press = millis();
      if(_minute >= 59) {
        _minute = 0;
        if(_hour >= 23) _hour = 0;
        else _hour++;
      }
      else _minute++; 
      change = true;
    }
    else if(button_back->pushed() && (millis() - previous_button_press > wait_time_between_buttons)) {
      previous_button_press = millis();
      if(_minute <= 0) {
          _minute = 59;
          if(_hour <=0) _hour = 23;
          else _hour--;
      }
      else _minute--;
      change = true;
    }
    else {
      // No button pushed, but let hands take steps to desired position
      // Rotate hour hand
      if(hands[hour_hand]->current_position != hands[hour_hand]->target_position) hands[hour_hand]->run_manually(QUICKEST_DIRECTION);
      // Rotate minute hand
      if(hands[minute_hand]->current_position != hands[minute_hand]->target_position) hands[minute_hand]->run_manually(QUICKEST_DIRECTION);
    }

    if(change == true) {
      // Calculate new positions
      set_time_positions();
      Serial.print("New time: ");
      Serial.print(_hour);
      Serial.print(" ");
      Serial.println(_minute);
    }
  }

  // Set new time to RTC
  _last_minute = _minute;
  rtc->adjust(DateTime(2000, 1, 1, _hour, _minute, 0)); // Write time to RTC
}

///////////////////////////////////////////////////////////////////////////////// TEST HAND ORDER /////////////////////////////////////////////////////////////
void Clockception::test_hand_order() {
  Serial.println(F("Test all hands one by one"));
  for(int hand=8; hand<nr_of_hands; hand++) {
    Serial.print(hand);
    Serial.print(" with pins: ");
    Serial.print(hands[hand]->step_pin());
    Serial.print(", ");
    Serial.println(hands[hand]->dir_pin());
    hands[hand]->target_position = steps_per_revolution-1;
    while(hands[hand]->current_position != hands[hand]->target_position) hands[hand]->run_manually(CW);
    while(!button_set->pushed()) delay(1);
  }
}

///////////////////////////////////////////////////////////////////////////////// RUN /////////////////////////////////////////////////////////////////////////

void Clockception::wait_for_new_minute() {
  while(_minute == _last_minute) {
    unsigned long waiting_start = millis();
    unsigned long wait_time = (60-_second);
    while(millis() - waiting_start <= (wait_time*1000)) { // Loop until wait time is past
      if(button_set->pushed()) set_settings(); // Check for button push
      else if(button_forward->pushed()) set_time_hour_forward();
      else if(button_back->pushed()) set_time_hour_back();
    }

    get_time(); // Check time.
  }
}

void Clockception::run() {
  randomSeed(analogRead(unused_pin)); // Set a random seed by reading an unused (floating) input pin
  get_time(); // Get time when running starts.
  
  while(true) { // Run forever
    wait_for_new_minute();

    // Print time and select animation
    Serial.print(_hour);
    Serial.print(F(":"));
    Serial.println(_minute);

    while(_current_animation == _previous_animation) { // Selet new animation, that doesn't match previous animation
      if (_minute % 5 == 0) _current_animation = random(1, 14); // Long animation
      else _current_animation = random(1, 14) + 20; // Short animations start in 20 range
    }

    switch(_current_animation) {
      case LONG_1:
        animation_long_1();
        break;
      case LONG_2:
        animation_long_2();
        break;
      case LONG_3:
        animation_long_3();
        break;
      case LONG_4:
        animation_long_4();
        break;
      case LONG_5:
        animation_long_5();
        break;
      case LONG_6:
        animation_long_6();
        break;
      case LONG_7:
        animation_long_7();
        break;
      case LONG_8:
        animation_long_8();
        break;
      case LONG_9:
        animation_long_9();
        break;
      case LONG_10:
        animation_long_10();
        break;
      case LONG_11:
        animation_long_11();
        break;
      case LONG_12:
        animation_long_12();
        break;
      case LONG_13:
        animation_long_12();
        break;
      
      case SHORT_1:
        animation_short_1();
        break;
      case SHORT_2:
        animation_short_2();
        break;
      case SHORT_3:
        animation_short_3();
        break;
      case SHORT_4:
        animation_short_4();
        break;
      case SHORT_5:
        animation_short_5();
        break;
      case SHORT_6:
        animation_short_6();
        break;
      case SHORT_7:
        animation_short_7();
        break;
      case SHORT_8:
        animation_short_8();
        break;
      case SHORT_9:
        animation_short_9();
        break;
      case SHORT_10:
        animation_short_10();
        break;
      case SHORT_11:
        animation_short_11();
        break;
      case SHORT_12:
        animation_short_12();
        break;
      case SHORT_13:
        animation_short_13();
        break;
    }
    
    _previous_animation = _current_animation;
    _last_minute = _minute;
  }
}



