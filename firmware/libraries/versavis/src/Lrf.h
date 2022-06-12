////////////////////////////////////////////////////////////////////////////////
//  Mai 2022
//  Author: Cornelius von Einem <veinemc@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Lrf.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding Laser Range Finders in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Lrf_h
#define Lrf_h

#include "Arduino.h"
// #include "Sensor.h"
#include <ros.h>
#include <std_msgs/Bool.h>
// #include <NMEAParser.h>
#include <lrf_msgs/Laser.h>
#include <lrf_msgs/Sentence.h>
#include <lrf_msgs/Nmea.h>


// timers Class Definition
class Lrf {
public:
  Lrf(ros::NodeHandle *nh, const String &topic, const String &topic_debug, const uint8_t interrupt_pin, Uart *serial_port = &Serial1);
//
//   void begin();
  int interruptPin() const { return interrupt_pin_; };


  void exposureBegin();
  void exposureEnd();
//
  void publish();
  void publishDebug();
  void setup();      // Setup publishers and pins.


//   void initialize(); // Perform initialization procedure.
  void setupPublisher();
  void setupPublisherDebug();
  void setupNMEASubscriber();
  void nmeaCallback(const lrf_msgs::Sentence &msg);
  void errorHandler();
  void pncosHandler();
  void snlrfHandler();

  Uart *serial_port_;


  void operator<<(char inChar);

protected:
  ros::NodeHandle *nh_;
  String topic_;
  String topic_deb_;
  ros::Publisher publisher_;
  ros::Publisher publisher_debug_;



private:
  ros::Time timestamp_begin_;
  ros::Time timestamp_end_;
  bool new_measurement_available_1_;
  bool new_measurement_available_2_;
  bool new_measurement_available_3_;
  bool new_sentence_available_;
  // Message to save sequence number and timestamp.
  lrf_msgs::Laser laser_msgs_;
  // lrf_msgs::Nmea nmea_srv_;
  lrf_msgs::Sentence sentence_msgs_;


  // Hardware pins for trigger and exposure signal.
  const uint8_t interrupt_pin_;




  void sendCmd(const char *input_array);


  // Parser
  void crcError();
  void bufferFull();
  void typeTooLong();
  void reset();
  void unexpectedChar();
  bool spaceAvail();
  void processSentence();
  void internalError();
  typedef enum { INIT, SENT, ARG, CRCH, CRCL, CRLFCR, CRLFLF } State;
  State mState;
  typedef enum {
      NO_ERROR,
      UNEXPECTED_CHAR,
      BUFFER_FULL,
      TYPE_TOO_LONG,
      CRC_ERROR,
      INTERNAL_ERROR
  } ErrorCode;
  ErrorCode mError;
  uint8_t mComputedCRC;
  bool mHandleCRC;
  uint8_t mGotCRC;
  uint8_t mIndex;
  uint8_t mArgIndex;
  uint8_t mHandlerCount;
  static const uint8_t kSentenceMaxSize = 77;
  char mBuffer[kSentenceMaxSize];
  static int8_t hexToNum(const char inChar);
  int8_t getHandler(const char *inToken);
  uint8_t argCount();
  uint8_t endArgPos(uint8_t inArgNum);
  bool validArgNum(uint8_t inArgNum);
  bool getArg(uint8_t num, char &arg);
  bool getArg(uint8_t num, char *arg);
  bool getArg(uint8_t num, int &arg);
  bool getArg(uint8_t num, float &arg);
  uint8_t startArgPos(uint8_t inArgNum);
  static bool strnwcmp(const char *s1, const char *s2, uint8_t len)
  {
    while (len-- > 0) {
      if (*s1 != *s2 && *s1 != '-' && *s2 != '-') return false;
      s1++;
      s2++;
    }
    return true;
  }

//   // Flag whether the camera should perform exposure compensation and is
//   // currently in compensating mode (see Nikolic 2014).
//   const bool exposure_compensation_;
//   bool is_configured_;
//   bool compensating_;
//


//
//   // Time when exposure started.
//   uint64_t exposure_start_us_;
//
//   // The image number is a strictly incrasing number used for data association
//   // of time stamp and image.
//   uint32_t image_number_;
//
//   // Time the next frame should be triggered before the mid-exposure time (in
//   // CPU ticks).
//   unsigned long exposure_delay_ticks_;
//
  // ROS related.
  String pub_topic_;
  String pub_topic_debug_;
  String nmea_sub_topic_;
  bool initialized_;
  ros::Subscriber<lrf_msgs::Sentence, Lrf> nmeaSubscriber_;
};

#endif
