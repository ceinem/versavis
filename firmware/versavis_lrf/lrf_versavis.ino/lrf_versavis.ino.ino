#include <NMEAParser.h>
NMEAParser<3> parser;

const char firstSentence[] = "$DBAXK*54\r\n";
const char secondSentence[] = "$EJCHQ,03,O,UUEIE,S,953.11,S,4.172,ASBUX,J*54\r\n";
const char thirdSentence[] = "$CCSNQ,RCS*22\r\n";

const char read_current_status[] = "CCSNQ,RCS";
const char read_current_options[] = "CCSNQ,RCO";
const char set_default_status[] = "CCSNQ,DFL";
//const char write_new_status[] = "CCSNQ,";
const char action_measurement[] = "CCSNQ,ACT";
const char self_test[] = "CCSNQ,TST*33\r\n";
const char distance_correction[] = "CCSNQ,COR";
//const char write_distance_correction[] = "CCSNQ,\r\n";


unsigned long StartTime;
unsigned long EndTime;

int incoming;

int last_input_state = 0;
int mes_complete = false;

bool debug = true;


//void errorHandler()
//{
//  SerialUSB.print("*** Error : ");
//  SerialUSB.println(parser.error()); 
//}

void firstHandler()
{
  SerialUSB.print("Got DBAXK with ");
  SerialUSB.print(parser.argCount());
  SerialUSB.println(" arguments");
  
  char arg0[3];
  if (parser.getArg(0,arg0)) SerialUSB.println(arg0);
}

void pncosHandler()
{
  SerialUSB.print("Got $PNCOS with ");
  SerialUSB.print(parser.argCount());
  SerialUSB.println(" arguments");
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
  if (parser.getArg(0,arg0)) 
  if (parser.getArg(1,arg1)) 
  if (parser.getArg(2,arg2)) 
  if (parser.getArg(3,arg3)) 
  if (parser.getArg(4,arg4)) 
  if (parser.getArg(5,arg5)) 
  if (parser.getArg(6,arg6)) 
  if (parser.getArg(7,arg7)) 
  if (parser.getArg(8,arg8)) 
  if (parser.getArg(9,arg9)) 

  if(debug) {
    SerialUSB.print("Power Quality: "); SerialUSB.println(arg0);
    SerialUSB.print("Measurement Mode: "); SerialUSB.println(arg1);
    SerialUSB.print("Distance unit: "); SerialUSB.println(arg2);
    SerialUSB.print("Speed unit: "); SerialUSB.println(arg3);
    SerialUSB.print("Distance gate: "); SerialUSB.println(arg4);
    SerialUSB.print("Target selection: "); SerialUSB.println(arg5);
    SerialUSB.print("??: "); SerialUSB.println(arg6);
    SerialUSB.print("Repetition Mode: "); SerialUSB.println(arg7);
    SerialUSB.print("Acion Time: "); SerialUSB.println(arg8);
    SerialUSB.print("Idle Time: "); SerialUSB.println(arg9);
  }
}

void snlrfHandler()
{
  SerialUSB.print("Got $SNLRF with ");
  SerialUSB.print(parser.argCount());
  SerialUSB.println(" arguments");
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

  if(parser.argCount() > 0) {
    parser.getArg(0,arg0);
    parser.getArg(1,arg1);
    parser.getArg(2,arg2);
    parser.getArg(3,arg3);
    parser.getArg(4,arg4);
  }
  if(parser.argCount() > 5) {
    parser.getArg(5,arg5);
    parser.getArg(6,arg6);
    parser.getArg(7,arg7);
  }
  if(parser.argCount() > 8) {
    parser.getArg(8,arg8);
    parser.getArg(9,arg9);
    parser.getArg(10,arg10);
  }



  if(debug) {
    SerialUSB.print("Power Quality: "); SerialUSB.println(arg0);
    SerialUSB.print("Result: "); SerialUSB.println(arg1);
    SerialUSB.print("Res1 type: "); SerialUSB.println(arg2);
    SerialUSB.print("Res1 value: "); SerialUSB.println(arg3);
    SerialUSB.print("Res1 unit: "); SerialUSB.println(arg4);
    SerialUSB.print("Res2 type: "); SerialUSB.println(arg5);
    SerialUSB.print("Res2 value: "); SerialUSB.println(arg6);
    SerialUSB.print("Res2 unit: "); SerialUSB.println(arg7);
    SerialUSB.print("Res3 type: "); SerialUSB.println(arg8);
    SerialUSB.print("Res3 value: "); SerialUSB.println(arg9);
    SerialUSB.print("Res3 unit: "); SerialUSB.println(arg9);
  }
}


void setup() {
//  SerialUSB.begin(38400);
//  Serial1.begin(38400);
//  pinMode(31, INPUT);
//  attachInterrupt(31, isr, CHANGE);

//  parser.setErrorHandler(errorHandler);
//  parser.addHandler("CCSNQ", firstHandler);
//  parser.addHandler("PNCOS", pncosHandler);  
//  parser.addHandler("SNLRF", snlrfHandler); 
}

//
//void sendCmd(const char *input_array, int array_size){
//
//  int new_length = array_size + 4;
//  char output_array[new_length];
//
//
//  int ind = 0;
//  output_array[ind] = '$';
//  
//  int mComputedCRC = 0;
//  for(int i = 0; i < array_size-1; i++){
//    char inChar = input_array[i];
//    ind++;
//    output_array[ind] = inChar;
//    //SerialUSB.println(inChar);
//    mComputedCRC ^= inChar;
//  }
//  ind++;
//  output_array[ind] = '*';
//  output_array[ind+1] = String(mComputedCRC,HEX)[0];
//  output_array[ind+2] = String(mComputedCRC,HEX)[1];
//  output_array[ind+3] = '\r';
//  output_array[ind+5] = '\n';
//  SerialUSB.print("Sending cmd: ");
//  SerialUSB.write(output_array);
//  SerialUSB.println();
//  Serial1.write(output_array,sizeof(output_array));
//}

void loop() {
  while (SerialUSB.available() > 0){      // If anything comes in Serial (USB),
    incoming = SerialUSB.read();


    if (incoming == 97) { // a
      sendCmd(action_measurement,sizeof(action_measurement));
    }
    if (incoming == 115) { // s
      sendCmd(read_current_status,sizeof(read_current_status));
    }
    if (incoming == 110) { //n
      const char write_new_status[] = "CCSNQ,WNS,D,M,M,0,AUTO,,O,0.5,0.5";
      sendCmd(write_new_status,sizeof(write_new_status));
    }
  }

//  while (Serial1.available() > 0) {      // If anything comes in Serial (USB),
//    incoming = Serial1.read();
//    SerialUSB.write(incoming);   // read it and send it out Serial1 (pins 0 & 1)
//    parser << incoming;
//  }
   
//    if (mes_complete) {
//      SerialUSB.println("Mes done");
//      SerialUSB.println(EndTime-StartTime);
//      mes_complete = false;
//    }

  
}


//void isr()
//{
//    if(digitalRead(31) == HIGH){
//      EndTime = millis();
//      mes_complete = true;
//    } else {
//      StartTime = millis();
//    }   
//}
