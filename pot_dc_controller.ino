/*

  pot_dc_controller.ino

  C++ based DC controller without CBUS unterface

  A stepping stone to a CBUS version.

  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)
  and Ian Blair (c) 2024 ian.charles.blair@gmail.com

  For license, attribution and terms and conditions see associated readme.md

*/
#include <Arduino.h>
#include <Streaming.h>
#include "LEDControl.h"

#include "dc_controller_defs.h"
#include "pindefs_dc_controller_esp32.h"
#include "throttle.h"
#include "dc_controller.h"

//
/// setup - runs once at power on
//
dc_controller Controller;

void setup() {

  Controller = dc_controller();  // Instantiate and initialise dc_controller

}

//
/// loop - runs forever
//

void loop() {

  //
  /// do CBUS message, switch and LED processing
  //
  // check reversing switch
  
  Controller.update();
  delay(1);
  
}


