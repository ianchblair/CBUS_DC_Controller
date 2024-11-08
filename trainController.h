
/**
 * Copyright (c) 2017 Ian Morgan
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License
 */
#ifndef trainController_h
#define trainController_h

#include "arduino.h"
// Analogue (PWM) Train Controller.
//
// Class: trainControllerClass
//
// Methods:
//                       trainControllerClass(int setPinA, int setPinB, int setPinPWM)            Class Constructor
//      void    matchToTargets ()
//      void    emergencyStop ()
//      void    setSpeedAndDirection (int newLocoDirection, int newLocoSpeed)
//      void    setSpeed (int newLocoSpeed)
//      uint8_t getSpeed ()
//      uint8_t getDirection ()
//      void    setPWMFrequency ()

#define SF_FORWARDS    0x01      // Train is running forwards
#define SF_REVERSE     0x00      // Train is running in reverse
#define PWM_FREQUENCY  30000     // PWM frequency (in Hz)
#define SPEED_STEP     15

class trainControllerClass
{
  private:
  uint8_t  currentLocoSpeed = 0;
  uint8_t  currentLocoDirection = SF_FORWARDS;
  uint8_t  targetLocoSpeed = 0;
  uint8_t  targetLocoDirection = SF_FORWARDS;
  int      pinA;
  int      pinB;
  int      pinPWM;


  // -------------------------------------------------

  public:
  boolean  eStopped = false;

  // -------------------------------------------------

  // Constructor - initializes the member variables and state
  trainControllerClass(void);
  
  void initialise(int setPinA,
                       int setPinB,
                       int setPinPWM);

  
  void initialise(void);
  
  // -------------------------------------------------

  private:

  // -------------------------------------------------

  void setControllerTargets (int newLocoDirection, int newLocoSpeed);
  // -------------------------------------------------

  public:

  // -------------------------------------------------

  void matchToTargets (void);
  
  // -------------------------------------------------

  void emergencyStop (void);

  // -------------------------------------------------

  void setSpeedAndDirection (int newLocoDirection, int newLocoSpeed);
  // -------------------------------------------------

  void  setSpeed (int newLocoSpeed);

  // -------------------------------------------------

  uint8_t  getSpeed (void);

  // -------------------------------------------------

  uint8_t  getDirection (void);

  // -------------------------------------------------

  void  setPWMFrequency (void);

  // -------------------------------------------------
};

#endif