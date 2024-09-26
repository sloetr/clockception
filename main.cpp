#include "Clockception.h"

Clockception clockception;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting Clockception"));
  clockception.init();
  Serial.println(F("Clockception initiated"));

  //Let user set hands straight and set time
  clockception.set_settings();
    
}

void loop() {
  //clockception.test_hand_order();
  
  //clockception.run();

  
  // Short 10 and 11 need to be tested
  delay(2000);
  clockception.show_time_equal_duration(6,30,0,800,.5,.5);
  clockception.run_animation();

  delay(1000);
  clockception.animation_short_10();
  delay(1000);

  clockception.animation_to_zero();

  delay(100000);

}
