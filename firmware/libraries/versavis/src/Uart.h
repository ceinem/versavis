////////////////////////////////////////////////////////////////////////////////
//  June 2022
//  Author: Cornelius von Einem <veinemc@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Uart.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding UART in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Lrf_h
#define Lrf_h

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Bool.h>

class Uart {
public:
  Uart(ros::NodeHandle *nh, const String &topic, Uart *serial_port = &Serial1);
//

protected:
  ros::NodeHandle *nh_;
private:

};
#endif
