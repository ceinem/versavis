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
           new_measurement_available_1_(false), new_measurement_available_2_(false),
           new_measurement_available_3_(false), new_sentence_available_(false),
           initialized_(false){

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
  // //
  // // pinMode(trigger_pin_, OUTPUT);
  // // digitalWrite(trigger_pin_, LOW);
  // pinMode(interrupt_pin_, INPUT);
  //
  serial_port_->begin(38400);

}


uint8_t Lrf::argCount()
{
  return kSentenceMaxSize - mArgIndex - 1;
}

uint8_t Lrf::startArgPos(uint8_t inArgNum)
{
  return mBuffer[kSentenceMaxSize - 1 - inArgNum];
}

uint8_t Lrf::endArgPos(uint8_t inArgNum)
{
  return mBuffer[kSentenceMaxSize - 2 - inArgNum];
}

bool Lrf::validArgNum(uint8_t inArgNum)
{
  return inArgNum < (kSentenceMaxSize - mArgIndex);
}

bool Lrf::getArg(uint8_t num, char *arg)
{
  if (validArgNum(num)) {
    uint8_t startPos = startArgPos(num);
    uint8_t endPos = endArgPos(num);
    {
      char mTmp = mBuffer[endPos];
      mBuffer[endPos] = '\0';
      // NMEAParserStringify stfy(this, endPos);
      strcpy(arg, &mBuffer[startPos]);
      mBuffer[endPos] = mTmp;
    }
    return true;
  }
  else return false;
}

bool Lrf::getArg(uint8_t num, char &arg)
{
  if (validArgNum(num)) {
    uint8_t startPos = startArgPos(num);
    uint8_t endPos = endArgPos(num);
    arg = mBuffer[startPos];
    return (endPos - startPos) == 1;
  }
  else return false;
}

bool Lrf::getArg(uint8_t num, int &arg)
{
  if (validArgNum(num)) {
    uint8_t startPos = startArgPos(num);
    uint8_t endPos = endArgPos(num);
    {
      char mTmp = mBuffer[endPos];
      mBuffer[endPos] = '\0';
      // NMEAParserStringify stfy(this, endPos);
      arg = atoi(&mBuffer[startPos]);
      mBuffer[endPos] = mTmp;
    }
    return true;
  }
  else return false;
}

bool Lrf::getArg(uint8_t num, float &arg)
{
  if (validArgNum(num)) {
    uint8_t startPos = startArgPos(num);
    uint8_t endPos = endArgPos(num);
    {
      char mTmp = mBuffer[endPos];
      mBuffer[endPos] = '\0';
      // NMEAParserStringify stfy(this, endPos);
      arg = atof(&mBuffer[startPos]);
      mBuffer[endPos] = mTmp;
    }
    return true;
  }
  else return false;
}

void Lrf::errorHandler()
{
  // SerialUSB.print("*** Error : ");
  // SerialUSB.println(parser_.error());
  DEBUG_PRINTLN(
      F("ERROR"));
      sentence_msgs_.sentence = mBuffer;
      new_sentence_available_ = true;
}

void Lrf::pncosHandler()
{
DEBUG_PRINTLN(
    F("PNCOS"));
  DEBUG_PRINT(argCount());
  DEBUG_PRINTLN(" arguments");
  char arg0; // Power Quality
  char arg1[3]; // Measurement Mode
  char arg2; // unit distance
  char arg3; //unit speed
  char arg4[4]; //gate
  char arg5[5]; //Target
  int arg6;
  char arg7; // Repetition rate
  char arg8[3]; // Action time
  char arg9[4]; // Idle time
  if (getArg(0,arg0))
  if (getArg(1,arg1))
  if (getArg(2,arg2))
  if (getArg(3,arg3))
  if (getArg(4,arg4))
  if (getArg(5,arg5))
  if (getArg(6,arg6))
  if (getArg(7,arg7))
  if (getArg(8,arg8))
  if (getArg(9,arg9))
  //
  // if(debug) {
    DEBUG_PRINT("Power Quality: "); DEBUG_PRINTLN(arg0);
    DEBUG_PRINT("Measurement Mode: "); DEBUG_PRINTLN(arg1);
    DEBUG_PRINT("Distance unit: "); DEBUG_PRINTLN(arg2);
    DEBUG_PRINT("Speed unit: "); DEBUG_PRINTLN(arg3);
    DEBUG_PRINT("Distance gate: "); DEBUG_PRINTLN(arg4);
    DEBUG_PRINT("Target selection: "); DEBUG_PRINTLN(arg5);
    DEBUG_PRINT("??: "); DEBUG_PRINTLN(arg6);
    DEBUG_PRINT("Repetition Mode: "); DEBUG_PRINTLN(arg7);
    DEBUG_PRINT("Acion Time: "); DEBUG_PRINTLN(arg8);
    DEBUG_PRINT("Idle Time: "); DEBUG_PRINTLN(arg9);
  // }
  sentence_msgs_.sentence = mBuffer;
  new_sentence_available_ = true;


}


void Lrf::snlrfHandler()
{
  DEBUG_PRINTLN(
      F("SNLRF"));
  // SerialUSB.print("*** Error : ");
  // SerialUSB.println(parser.error());
  DEBUG_PRINT("Got $SNLRF with ");
  DEBUG_PRINT(argCount());
  DEBUG_PRINTLN(" arguments");
  char arg0; // Power Quality
  char arg1; // Result
  char arg2; // Result type
  float arg3; //Result 1
  char arg4; //Result 1 Unit
  char arg5; //Result 2 type
  float arg6; //Result 2
  char arg7; // Result 2 unit
  char arg8; // Result 3 type
  float arg9; // Result 3
  char arg10; // Result 3 unit

  if(argCount() > 0) {
    getArg(0,arg0);
    getArg(1,arg1);
    getArg(2,arg2);
    getArg(3,arg3);
    getArg(4,arg4);
    laser_msgs_.data = arg3;
    new_measurement_available_1_ = true;
  }
  if(argCount() > 5) {
    getArg(5,arg5);
    getArg(6,arg6);
    getArg(7,arg7);
  }
  if(argCount() > 8) {
    getArg(8,arg8);
    getArg(9,arg9);
    getArg(10,arg10);
  }



    DEBUG_PRINT("Power Quality: "); DEBUG_PRINTLN(arg0);
    DEBUG_PRINT("Result: "); DEBUG_PRINTLN(arg1);
    DEBUG_PRINT("Res1 type: "); DEBUG_PRINTLN(arg2);
    DEBUG_PRINT("Res1 value: "); DEBUG_PRINTLN(arg3);
    DEBUG_PRINT("Res1 unit: "); DEBUG_PRINTLN(arg4);
    DEBUG_PRINT("Res2 type: "); DEBUG_PRINTLN(arg5);
    DEBUG_PRINT("Res2 value: "); DEBUG_PRINTLN(arg6);
    DEBUG_PRINT("Res2 unit: "); DEBUG_PRINTLN(arg7);
    DEBUG_PRINT("Res3 type: "); DEBUG_PRINTLN(arg8);
    DEBUG_PRINT("Res3 value: "); DEBUG_PRINTLN(arg9);
    DEBUG_PRINT("Res3 unit: "); DEBUG_PRINTLN(arg9);

    sentence_msgs_.sentence = mBuffer;
    new_sentence_available_ = true;


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
  uint8_t endPos = startArgPos(0);
  int8_t slot;
  {
    slot = getHandler(this->mBuffer);
  }
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
//
//
//
// // void Camera::initialize() {
// //   if (!is_configured_ || initialized_) {
// //     return;
// //   }
// //   DEBUG_PRINTLN((topic_ + " (Camera.cpp): Initialize.").c_str());
// //   Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US * 10, type_);
// //   Sensor::setTimestampNow();
// //   ++image_number_;
// //   image_time_msg_.number = image_number_;
// //   Sensor::newMeasurementIsAvailable();
// //   publish();
// // #ifdef DEBUG
// //   initialized_ = true;
// // #endif
// // }
//
// // void Camera::begin() {
// //   if (!is_configured_) {
// //     return;
// //   }
// //   DEBUG_PRINTLN((topic_ + " (Camera.cpp): Begin.").c_str());
// //   // Maximal exposure time to still be able to keep up with the frequency
// //   // considering a security factor of 0.99, in us.
// //   max_exposure_time_us_ = 0.99 * 1e6 / rate_hz_;
// //
// //   // Setup timer to periodically trigger the camera.
// //   Sensor::setupTimer();
// // }
//
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
  sendCmd(msg.sentence);
}



void Lrf::exposureBegin() {
  DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Exposure begin.").c_str());
  timestamp_begin_ = nh_->now();
  new_measurement_available_2_ = true;
}

void Lrf::exposureEnd() {
  DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Exposure end.").c_str());
  timestamp_end_ = nh_->now();
  new_measurement_available_3_ = true;
}

void Lrf::publish() {
  if (new_measurement_available_1_ && new_measurement_available_2_ && new_measurement_available_3_) {
    DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Publish.").c_str());
//     image_time_msg_.time = Sensor::getTimestamp();
//     image_time_msg_.number = image_number_;
    laser_msgs_.stamp_start = timestamp_begin_;
    float duration = (timestamp_end_- timestamp_begin_).toSec();
    laser_msgs_.duration = duration;
    if((nh_->now() - timestamp_end_).toSec() < 1.0) {

    } else {
      laser_msgs_.data = 33333.333;
    }

#ifndef DEBUG
    publisher_.publish(&laser_msgs_);
#endif
    new_measurement_available_1_ = false;
    new_measurement_available_2_ = false;
    new_measurement_available_3_ = false;
  }
}

void Lrf::publishDebug() {
  if(new_sentence_available_) {
    DEBUG_PRINTLN((topic_ + " (Lrf.cpp): Publish debug.").c_str());
#ifndef DEBUG
    publisher_debug_.publish(&sentence_msgs_);
#endif
    new_sentence_available_ = false;
  }
}

void Lrf::sendCmd(const char *input_array) {
  int array_size = strlen(input_array);
  int new_length = array_size + 8;
  char output_array[new_length];


  int ind = 0;
  output_array[ind] = '$';

  int mComputedCRC = 0;
  for(int i = 0; i < array_size; i++){
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


  serial_port_->write(output_array,sizeof(output_array));
}
