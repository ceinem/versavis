////////////////////////////////////////////////////////////////////////////////
//  Mai 2022
//  Author: Cornelius von Einem <veinemc@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Lrf.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding Laser Range Finders in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "Lrf.h"
#include "helper.h"
#include "versavis_configuration.h"


Lrf::Lrf(ros::NodeHandle *nh, const String &topic, const String &topic_debug,
               const uint8_t interrupt_pin,
               Uart *serial_port)
      : nh_(nh), topic_(topic), topic_deb_(topic_debug),
           publisher_((topic + "img_time").c_str(), &laser_msgs_),
           publisher_debug_((topic_debug + "img_time").c_str(), &sentence_msgs_),
           interrupt_pin_(interrupt_pin), serial_port_(serial_port),
           nmeaSubscriber_((topic_debug + "_cmd").c_str(), &Lrf::nmeaCallback, this),
           new_measurement_available_(false), initialized_(false){

  // Check the input.
  if (interrupt_pin == 0) {
    error((topic + " (Lrf.cpp): Interrupt pin is not set.").c_str(), 10);
  }
  DEBUG_PRINTLN(
      F("Test"));
  // Sensor::newMeasurementIsNotAvailable();
}

void Lrf::setup() {
  if (topic_.length() == 0) {
    // Cameras without a topic are considered as disconnected.
    DEBUG_PRINTLN(
        F("NO_TOPIC (Lrf.cpp): Skip LRF setup for disconnected LRF."));
    initialized_ = true;
    return;
  }
  DEBUG_PRINTLN(F((topic_ + " (Lrf.cpp): Setup.").c_str()));
  DEBUG_PRINTLN((topic_deb_ + " (Lrf.cpp): Setup.").c_str());

  setupPublisher();
  setupPublisherDebug();
  setupNMEASubscriber();
  //
  // pinMode(trigger_pin_, OUTPUT);
  // digitalWrite(trigger_pin_, LOW);
  pinMode(interrupt_pin_, INPUT);

  serial_port_->begin(38400);

  // parser_.setErrorHandler(Lrf::errorHandler);
  // // parser_.addHandler("CCSNQ", firstHandler);
  // parser_.addHandler("PNCOS", Lrf::pncosHandler);
  // parser_.addHandler("SNLRF", Lrf::snlrfHandler);
}

void Lrf::errorHandler()
{
  // SerialUSB.print("*** Error : ");
  // SerialUSB.println(parser_.error());
  DEBUG_PRINTLN(
      F("ERROR"));
}
void Lrf::pncosHandler()
{
DEBUG_PRINTLN(
    F("PNCOS"));
  // SerialUSB.print("*** Error : ");
  // SerialUSB.println(parser.error());
  // this.sentence_msgs_.sentence = "Got a PNCOS msg";
  // publishDebug();
}
void Lrf::snlrfHandler()
{
  DEBUG_PRINTLN(
      F("SNLRF"));
  // SerialUSB.print("*** Error : ");
  // SerialUSB.println(parser.error());
}



void Lrf::reset() {
  mState = INIT;
  mIndex = 0;
  mArgIndex = kSentenceMaxSize;
  mError = NO_ERROR;
}

void Lrf::unexpectedChar()
{
  mError = UNEXPECTED_CHAR;
  // callErrorHandler();
  reset();
}

bool Lrf::spaceAvail()
{
  return (mIndex < mArgIndex);
}
void Lrf::typeTooLong()
{
  mError = TYPE_TOO_LONG;
  // callErrorHandler();
  reset();
}
void Lrf::bufferFull()
{
  mError = BUFFER_FULL;
  // callErrorHandler();
  reset();
}

void Lrf::crcError()
{
  mError = CRC_ERROR;
  // callErrorHandler();
  reset();
}

void Lrf::internalError()
{
  mError = INTERNAL_ERROR;
  // callErrorHandler();
  reset();
}

int8_t Lrf::getHandler(const char *inToken)
{
  /* Look for the token */
  if (strnwcmp("PNCOS", inToken, 5)) {
    pncosHandler();
  } else if (strnwcmp("SNLRF", inToken, 5)) {
    snlrfHandler();
  } else {
    errorHandler();
  }
}

void Lrf::processSentence()
{
  /* Look for the token */
  uint8_t endPos = startArgPos(0);
  int8_t slot;
  {
    // NMEAParserStringify stfy(this, endPos);
    // String mBufferTok(this->mBuffer);
    slot = getHandler(this->mBuffer);
  }
  // if (slot != -1) {
  //   mHandlers[slot].mHandler();
  // }
  // else {
  //   // if (mDefaultHandler != NULL) {
  //   //   mDefaultHandler();
  //   // }
  // }
}

int8_t Lrf::hexToNum(const char inChar)
{
if (isdigit(inChar)) return inChar - '0';
else if (isupper(inChar) && inChar <= 'F') return inChar - 'A' + 10;
else if (islower(inChar) && inChar <= 'f') return inChar - 'a' + 10;
else return -1;
}


void Lrf::operator<<(char inChar) {
  int8_t tmp;

      switch (mState) {

        /* Waiting for the starting $ character */
        case INIT:
          mError = NO_ERROR;
          if (inChar == '$') {
            mComputedCRC = 0;
            mState = SENT;
          }
          else unexpectedChar();
          break;

        case SENT:
          if (isalnum(inChar)) {
            if (spaceAvail()) {
              if (mIndex < 5) {
                mBuffer[mIndex++] = inChar;
                mComputedCRC ^= inChar;
              }
              else {
                typeTooLong();
              }
            }
            else bufferFull();
          }
          else {
            switch(inChar) {
              case ',' :
                mComputedCRC ^= inChar;
                mBuffer[--mArgIndex] = mIndex;
                mState = ARG;
                break;
              case '*' :
                mGotCRC = 0;
                mBuffer[--mArgIndex] = mIndex;
                mState = CRCH;
                break;
              default :
                unexpectedChar();
                break;
            }
          }
          break;

        case ARG:
          if (spaceAvail()) {
            switch(inChar) {
              case ',' :
                mComputedCRC ^= inChar;
                mBuffer[--mArgIndex] = mIndex;
                break;
              case '*' :
                mGotCRC = 0;
                mBuffer[--mArgIndex] = mIndex;
                mState = CRCH;
                break;
              default :
                mComputedCRC ^= inChar;
                mBuffer[mIndex++] = inChar;
                break;
            }
          }
          else bufferFull();
          break;

        case CRCH:
          tmp = hexToNum(inChar);
          if (tmp != -1) {
            mGotCRC |= (uint8_t)tmp << 4;
            mState = CRCL;
          }
          else unexpectedChar();
          break;

        case CRCL:
          tmp = hexToNum(inChar);
          if (tmp != -1) {
            mGotCRC |= (uint8_t)tmp;
            mState = CRLFCR;
          }
          else unexpectedChar();
          break;

        case CRLFCR:
          if (inChar == '\r') {
            mState = CRLFLF;
          }
          else unexpectedChar();
          break;

        case CRLFLF:
          if (inChar == '\n') {
            if (mHandleCRC && (mGotCRC != mComputedCRC)) {
              crcError();
            }
            else {
              processSentence();
            }
            reset();
          }
          else unexpectedChar();
          break;

        default:
          internalError();
          break;
      }
}



// void Camera::initialize() {
//   if (!is_configured_ || initialized_) {
//     return;
//   }
//   DEBUG_PRINTLN((topic_ + " (Camera.cpp): Initialize.").c_str());
//   Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US * 10, type_);
//   Sensor::setTimestampNow();
//   ++image_number_;
//   image_time_msg_.number = image_number_;
//   Sensor::newMeasurementIsAvailable();
//   publish();
// #ifdef DEBUG
//   initialized_ = true;
// #endif
// }

// void Camera::begin() {
//   if (!is_configured_) {
//     return;
//   }
//   DEBUG_PRINTLN((topic_ + " (Camera.cpp): Begin.").c_str());
//   // Maximal exposure time to still be able to keep up with the frequency
//   // considering a security factor of 0.99, in us.
//   max_exposure_time_us_ = 0.99 * 1e6 / rate_hz_;
//
//   // Setup timer to periodically trigger the camera.
//   Sensor::setupTimer();
// }

void Lrf::setupPublisher() {
  pub_topic_ = topic_;
  publisher_ = ros::Publisher(pub_topic_.c_str(), &laser_msgs_);
  DEBUG_PRINT(F((topic_ + " (Lrf.cpp): Setup publisher with topic ").c_str()));
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}

void Lrf::setupPublisherDebug() {
  pub_topic_debug_ = topic_deb_;
  publisher_debug_ = ros::Publisher(pub_topic_debug_.c_str(), &sentence_msgs_);
  DEBUG_PRINT(F((topic_ + " (Lrf.cpp): Setup publisher with topic ").c_str()));
  DEBUG_PRINTLN(publisher_debug_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_debug_);
#endif
}

void Lrf::setupNMEASubscriber() {
  nmea_sub_topic_ = pub_topic_debug_ + "_cmd";
  nmeaSubscriber_ = ros::Subscriber<lrf_msgs::Sentence, Lrf>(nmea_sub_topic_.c_str(), &Lrf::nmeaCallback, this);
DEBUG_PRINTLN(F((" (Lrf.cpp): Setup NMEA subscriber")));
#ifndef DEBUG
  nh_->subscribe(nmeaSubscriber_);
#endif
}

void Lrf::nmeaCallback(const lrf_msgs::Sentence &msg)
{
  // sendCmd(msg.sentence, size_of(msg.sentence));
}

// void Camera::triggerMeasurement() {
//   // Check whether an overflow caused the interrupt.
//   if (!timer_.checkOverflow()) {
//     DEBUG_PRINTLN(
//         (topic_ + " (Camera.cpp): Timer interrupt but not overflown.").c_str());
//     return;
//   }
//   if (!is_configured_ || !initialized_) {
//     return;
//   }
//   DEBUG_PRINTLN((topic_ + " (Camera.cpp): Timer overflow.").c_str());
//
//   if (exposure_compensation_ && compensating_) {
//     DEBUG_PRINTLN((topic_ + " (Camera.cpp): Compensating.").c_str());
//     // Exposure-time compensating mode (Nikolic 2014). During exposure, the
//     // timer will interrupt in the middle of the exposure time. At the end of
//     // the exposure, the external interrupt will trigger exposureEnd() and
//     // reset the timer to trigger the camera at the appropriate time.
//     if (!exposing_) {
//       DEBUG_PRINTLN(
//           (topic_ + " (Camera.cpp): Not exposing. Trigger camera.").c_str());
//       // The camera is currently not exposing meaning that the interrupt
//       // triggers at the beginning of the next image.
//       exposing_ = true;
//
// #ifdef ILLUMINATION_MODULE
//       // If the illumination module is active, the LEDs should turn on just
//       // before the camera is exposing.
//       digitalWrite(ILLUMINATION_PIN, HIGH);
//       // Here, a warm-up delay for the LEDs can be added (needs checking).
//       // delayMicroseconds(10);
// #endif
//       // Trigger the actual pulse.
//       Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);
//
//       // Save the current time to estimate the exposure time in the pin
//       // interrupt.
//       exposure_start_us_ = micros();
//
//       // Increament the image number as the camera is triggered now.
//       ++image_number_;
//
//       // Set the timer to the mid exposure point, e.g. half the exposure time.
//       timer_.setCompare(exposure_delay_ticks_ > 0 ? exposure_delay_ticks_ - 1
//                                                   : compare_);
//     } else {
//       DEBUG_PRINTLN(
//           (topic_ + " (Camera.cpp): Exposing right now, get timestamp.")
//               .c_str());
//       // The camera is currently exposing. In this case, the interrupt is
//       // triggered in the middle of the exposure time, where the timestamp
//       // should be taken.
//       Sensor::setTimestampNow();
//       Sensor::newMeasurementIsAvailable();
// #ifdef ADD_TRIGGERS
//       trigger(ADDITIONAL_TEST_PIN, TRIGGER_PULSE_US,
//               Sensor::trigger_type::NON_INVERTED);
// #endif
//
//       // Even though we are still in the compensating mode, deactivating here
//       // ensures that we detect if a exposure signal is dropped and we switch
//       // to non-compensating mode.
//       compensating_ = false;
//
//       // Set the timer to the standard period as we dont know the current
//       // exposure time yet.
//       timer_.setCompare(compare_);
//     }
//   } else {
//     // "Standard" mode where the camera is triggered purely periodic.
//     DEBUG_PRINTLN(
//         (topic_ +
//          " (Camera.cpp): Not compensating. Trigger camera and take timestamp.")
//             .c_str());
//     exposing_ = true;
//
// #ifdef ILLUMINATION_MODULE
//     // Deactivate the LEDs as we are not sure yet whether we get an exposure
//     // signal.
//     digitalWrite(ILLUMINATION_PIN, LOW);
// #endif
//     // Trigger the actual pulse.
//     Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);
//
//     Sensor::setTimestampNow();
//     Sensor::newMeasurementIsAvailable();
//
//     // Save the current time to estimate the exposure time in the pin
//     // interrupt.
//     exposure_start_us_ = micros();
//
//     // Increament the image number as the camera is triggered now.
//     image_number_++;
//
//     // Set the timer to make sure that the camera is triggered in a periodic
//     // mode.
//     timer_.setCompare(compare_);
//   }
//   // Reset the timer.
//   timer_.resetOverflow();
// }

void Lrf::exposureBegin() {
  DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Exposure begin.").c_str());
  // if (exposure_compensation_) {
  //   unsigned long last_exposure_time_us = micros() - exposure_start_us_;
  //   DEBUG_PRINT((topic_ + " (Camera.cpp): exposure time [us] ").c_str());
  //   DEBUG_PRINTDECLN(last_exposure_time_us);
  //   calculateDelayTicksAndCompensate(last_exposure_time_us);
  //   exposing_ = false;
  // }
}

void Lrf::exposureEnd() {
  DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Exposure end.").c_str());
  // if (exposure_compensation_) {
  //   unsigned long last_exposure_time_us = micros() - exposure_start_us_;
  //   DEBUG_PRINT((topic_ + " (Camera.cpp): exposure time [us] ").c_str());
  //   DEBUG_PRINTDECLN(last_exposure_time_us);
  //   calculateDelayTicksAndCompensate(last_exposure_time_us);
  //   exposing_ = false;
  // }
}

void Lrf::publish() {
//   if (Sensor::isNewMeasurementAvailable()) {
//     DEBUG_PRINTLN((topic_ + " (Camera.cpp): Publish.").c_str());
//     image_time_msg_.time = Sensor::getTimestamp();
//     image_time_msg_.number = image_number_;
// #ifndef DEBUG
//     publisher_.publish(&image_time_msg_);
// #endif
//     Sensor::newMeasurementIsNotAvailable();
//   }
}

void Lrf::publishDebug() {
    DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Publish debug.").c_str());
#ifndef DEBUG
    publisher_debug_.publish(&sentence_msgs_);
#endif
}

void Lrf::sendCmd(const char *input_array, int array_size) {
  int new_length = array_size + 4;
  char output_array[new_length];


  int ind = 0;
  output_array[ind] = '$';

  int mComputedCRC = 0;
  for(int i = 0; i < array_size-1; i++){
    char inChar = input_array[i];
    ind++;
    output_array[ind] = inChar;
    mComputedCRC ^= inChar;
  }
  ind++;
  output_array[ind] = '*';
  output_array[ind+1] = String(mComputedCRC,HEX)[0];
  output_array[ind+2] = String(mComputedCRC,HEX)[1];
  output_array[ind+3] = '\r';
  output_array[ind+5] = '\n';
  // SerialUSB.print("Sending cmd: ");
  // SerialUSB.write(output_array);
  // SerialUSB.println();
  serial_port_->write(output_array,sizeof(output_array));
}

// void Camera::calculateDelayTicksAndCompensate(
//     const unsigned long &last_exposure_time_us) {
//   // The goal here is to shift the time of the next camera trigger half the
//   // exposure time before the mid-exposure time.
//
//   // The next frame should be triggered by this time before the mid-exposure
//   // time (in CPU ticks).
//   if (last_exposure_time_us == 0 ||
//       last_exposure_time_us >= max_exposure_time_us_) {
//     // In this case, something with the measurement went wrong or the camera
//     // is dropping triggers due to a too high exposure time (constrain the
//     // maximal exposure time of your camera to be within the period time of
//     // your triggering). Switch to non-compensating mode.
//     exposure_delay_ticks_ = 0;
//     compensating_ = false;
//   } else {
//     exposure_delay_ticks_ = static_cast<double>(last_exposure_time_us) / 2.0 /
//                             1000000.0 * cpu_freq_prescaler_;
//     compensating_ = true;
//   }
//
//   // Reset the compare register of the timer.
//   timer_.setCompare(compare_ - exposure_delay_ticks_);
// }
