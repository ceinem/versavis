////////////////////////////////////////////////////////////////////////////////
//  June 2022
//  Author: Cornelius von Einem <veinemc@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Uart.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding UART in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "Uart.h"
#include "helper.h"
#include "versavis_configuration.h"

Uart::Uart(ros::NodeHandle *nh, const String &topic,
               Uart *serial_port)
      : nh_(nh){

  // Check the input.
  if (interrupt_pin == 0) {
    error((topic + " (Lrf.cpp): Interrupt pin is not set.").c_str(), 10);
  }
  // DEBUG_PRINTLN(
  //     F("Test"));
  // Sensor::newMeasurementIsNotAvailable();
  new_measurement_available_1_ = false;
  new_measurement_available_2_ = false;
  new_measurement_available_3_ = false;
  new_sentence_available_ = false;
}
