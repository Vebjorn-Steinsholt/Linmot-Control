//Linmot Serial2CAN-BUS Controller
//Robert Opland
//NTNU - Norwegian University of Science and Technology
//Version 1.1
//Features added by Vebjørn Steinsholt
//Added force control
//NTNU - Norwegian University of Science and Technology
//Adapted to use Serial instead of UDP
#include <mcp_can.h>    //Using this library for the canbus communication
#include <SPI.h>
#include "Wire.h"
#define CAN0_INT 5        // Set INT to pin 5
MCP_CAN CAN0(10);     // Set CS to pin 10 normally. If stacked with ethernet shield v2, bend pin 10 and solder a wire that can plug into port 3 for example, and then change cs pin to 3.


//int CAN_ID = 0x000;
int ext = 0;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];   // Array to store serial string
float returnFloatPos;

//Creating variables to control/set motion commands
int choice = 0;
int prevChoice = 0;
long targetPos = 0;
unsigned long maxVel = 0;
unsigned long Acceleration = 0;
unsigned long Deceleration = 0;
int commandChoice = 0;
int targetForce = 0;
int forceLimit = 0;
byte txBuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

union long_pack{
  long l;
  byte b[4];
};
union unsigned_long_pack{
  unsigned long l;
  byte b[4];
};

long_pack pos;
unsigned_long_pack vel;
unsigned_long_pack acc;
unsigned_long_pack dacc;

// Serial communication buffer (legacy buffers removed to avoid name clash with core SERIAL_BUFFER_SIZE)



//Function to listen to Serial messages:__________________________________________
int listenToSerial(){
  
  // Check if a complete line is available
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim(); // Remove any whitespace
    
    if (line.length() == 0) {
      return 0;
    }
    
    // Debug: uncomment to see received data
    Serial.print(F("Received: "));
    Serial.println(line);
    
    // Parse CSV format: choice,targetPos,maxVel,acc,dec,targetForce,forceLimit
    int commaIndex[6];
    int fieldCount = 0;
    
    // Find comma positions
    for (int i = 0; i < line.length() && fieldCount < 6; i++) {
      if (line.charAt(i) == ',') {
        commaIndex[fieldCount++] = i;
      }
    }
    
    // Need at least 6 commas for 7 fields
    if (fieldCount < 6) {
      return 0; // Invalid format
    }
    
    // Parse each field
    float packetVal_1 = line.substring(0, commaIndex[0]).toFloat();
    float packetVal_2 = line.substring(commaIndex[0] + 1, commaIndex[1]).toFloat();
    float packetVal_3 = line.substring(commaIndex[1] + 1, commaIndex[2]).toFloat();
    float packetVal_4 = line.substring(commaIndex[2] + 1, commaIndex[3]).toFloat();
    float packetVal_5 = line.substring(commaIndex[3] + 1, commaIndex[4]).toFloat();
    float packetVal_6 = line.substring(commaIndex[4] + 1, commaIndex[5]).toFloat();
    float packetVal_7 = line.substring(commaIndex[5] + 1).toFloat();
    
    //Scaling variables from Serial to make some common sense: pos=mm, V=m/s, acc/dcc=m/s^2. These are the units you send from Serial source
    choice        = packetVal_1;          //Valid choices: 1-11
    targetPos     = packetVal_2*10000;       //Wanted position in millimeters
    maxVel        = packetVal_3*1000000;     //Maximum velocity in m/s
    Acceleration  = packetVal_4*100000;       //Acceleration in m/s^2
    Deceleration  = packetVal_5*100000;       //Deceleration in m/s^2
    targetForce   = packetVal_6*10;          //Target force
    forceLimit    = packetVal_7*10;          //Force limit

    // Debug: uncomment to see parsed values
    Serial.print(F("Parsed - Choice: "));
    Serial.print(choice);
    Serial.print(F(", Pos: "));
    Serial.print(packetVal_2);
    Serial.print(F("mm, Vel: "));
    Serial.print(packetVal_3);
    Serial.print(F("m/s, Acc: "));
    Serial.print(packetVal_4);
    Serial.print(F(", Dec: "));
    Serial.print(packetVal_5);
    Serial.print(F(", Force: "));
    Serial.print(packetVal_6);
    Serial.print(F(", Limit: "));
    Serial.println(packetVal_7);

    return line.length();        
  }
  return 0;
}

//Function to send Serial messages___________________________________________________
void sendSerial(float currPos,long int statWord){
  // Send CSV format: actualPos,statusWord\n
  Serial.print(F("Sending: "));
  Serial.print(currPos, 4); // 4 decimal places for position
  Serial.print(',');
  Serial.print(statWord);
  Serial.print('\n');
}

// send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send__________________________________
void send_Msg(int Can_Id, int ExtID, int dL, byte dataArray[]){
  int CANID = Can_Id;
  int external = ExtID;
  int dataLength = dL;
  byte msgData[dataLength] = {int(dataArray[0]),int(dataArray[1]),int(dataArray[2]),int(dataArray[3]),int(dataArray[4]),int(dataArray[5]),int(dataArray[6]),int(dataArray[7])}; 
  byte sndStat = CAN0.sendMsgBuf(CANID, external, dataLength, msgData);
}

//Get position from actuator
void askPosition(int nodeID){
  int CANID_TPDO1 = 0x180 + nodeID;

  byte sendDataArray_TPDO1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_Msg(CANID_TPDO1, 0, 8, sendDataArray_TPDO1);
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}

//Function to send setpoint to drive
void send_VAI_16Bit_gotoPos(int nodeID, int targetPos,int maxVelocity,int Acceleration,int Deceleration){
  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   Here you send the control word, mc-header, target position and maximum velocity
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   Here you send Acceleration and Deceleration
  byte cmdHeader[] = {0x09, 0x01};
  byte ctrlWord[] = {0x00,0x3f};

  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], lowByte(targetPos), highByte(targetPos), lowByte(maxVelocity), highByte(maxVelocity)};
  byte sendDataArray_PDO2[] = {lowByte(Acceleration), highByte(Acceleration), lowByte(Deceleration), highByte(Deceleration), 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);  
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}

//Function to send setpoint to drive
void send_VAI_gotoPos(int nodeID, long targetPos,unsigned long maxVelocity,unsigned long Acceleration, unsigned long Deceleration){
  pos.l = targetPos;
  vel.l = maxVelocity;
  acc.l = Acceleration;
  dacc.l = Deceleration;

  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   Here you send the control word, mc-header, target position and maximum velocity
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   Here you send maximum velocity
  int CANID_PDO3 = 0x400 + nodeID;   //Can_id = COB_ID + node_ID(400 + 4  for pdo3)   Here you send Acceleration and Deceleration
  byte cmdHeader[] = {0x01, 0x01};
  byte ctrlWord[] = {0x00,0x3f};

  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0],pos.b[0],pos.b[1],pos.b[2],pos.b[3]};
  byte sendDataArray_PDO2[] = {vel.b[0],vel.b[1],vel.b[2],vel.b[3],acc.b[0],acc.b[1],acc.b[2],acc.b[3]};
  byte sendDataArray_PDO3[] = {dacc.b[0],dacc.b[1],dacc.b[2],dacc.b[3], 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);
  send_Msg(CANID_PDO3, 0, 8, sendDataArray_PDO3);    
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}
void send_VAI_gotoPosAndResetForceControl(int nodeID, long targetPos,unsigned long maxVelocity,unsigned long Acceleration, unsigned long Deceleration){
  pos.l = targetPos;
  vel.l = maxVelocity;
  acc.l = Acceleration;
  dacc.l = Deceleration;

  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   Here you send the control word, mc-header, target position and maximum velocity
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   Here you send maximum velocity
  int CANID_PDO3 = 0x400 + nodeID;   //Can_id = COB_ID + node_ID(400 + 4  for pdo3)   Here you send Acceleration and Deceleration
  byte cmdHeader[] = {0x38, 0x11};
  byte ctrlWord[] = {0x00,0x3f};

  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0],pos.b[0],pos.b[1],pos.b[2],pos.b[3]};
  byte sendDataArray_PDO2[] = {vel.b[0],vel.b[1],vel.b[2],vel.b[3],acc.b[0],acc.b[1],acc.b[2],acc.b[3]};
  byte sendDataArray_PDO3[] = {dacc.b[0],dacc.b[1],dacc.b[2],dacc.b[3], 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);
  send_Msg(CANID_PDO3, 0, 8, sendDataArray_PDO3);    
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}
void Force_Ctrl_ChangeTargetForce(int nodeID, int targetForce){
  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   Here you send the control word, mc-header, target position and maximum velocity
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   Here you send maximum velocity
  int CANID_PDO3 = 0x400 + nodeID;   //Can_id = COB_ID + node_ID(400 + 4  for pdo3)   Here you send Acceleration and Deceleration
  byte cmdHeader[] = {0x38, 0x22};
  byte ctrlWord[] = {0x00,0x3f};

  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], lowByte(targetForce), highByte(targetForce), 0x00, 0x00};
  byte sendDataArray_PDO2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}
void send_VAI_gotoPosWithHigherForceCtrlLimitandTargetForce(int nodeID, long targetPos,unsigned long maxVelocity,unsigned long Acceleration, unsigned long Deceleration, int forceLimit, int targetForce){
  pos.l = targetPos;
  vel.l = maxVelocity;
  acc.l = Acceleration;
  dacc.l = Deceleration;
  

  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   Here you send the control word, mc-header, target position and maximum velocity
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   Here you send maximum velocity
  int CANID_PDO3 = 0x400 + nodeID;   //Can_id = COB_ID + node_ID(400 + 4  for pdo3)   Here you send Acceleration and Deceleration
  byte cmdHeader[] = {0x38, 0x31};
  byte ctrlWord[] = {0x00,0x3f};

  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0],pos.b[0],pos.b[1],pos.b[2],pos.b[3]};
  byte sendDataArray_PDO2[] = {vel.b[0],vel.b[1],vel.b[2],vel.b[3],acc.b[0],acc.b[1],acc.b[2],acc.b[3]};
  byte sendDataArray_PDO3[] = {lowByte(forceLimit), highByte(forceLimit), lowByte(targetForce), highByte(targetForce),0x00,0x00,0x00,0x00};

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);
  send_Msg(CANID_PDO3, 0, 8, sendDataArray_PDO3);    
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}
//Function to send sine motion command to drive
void send_SIN_VA_GoToPos(int nodeID, long targetPos,long maxVelocity, long Acceleration, long Deceleration){
  
  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   PDO1 is the 0x200 series
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   PDO2 is the 0x300 series
  byte cmdHeader[] = {0x0E, 0x04};
  byte ctrlWord[] = {0x00,0x3f};

  //creating the array of data to be transmitted
  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    //Message to enter operational state
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], highByte(targetPos), lowByte(targetPos), highByte(maxVelocity), lowByte(maxVelocity)};  //Message to send data to PDO1(header, target pos, max velocity)
  byte sendDataArray_PDO2[] = {highByte(Acceleration), lowByte(Acceleration), highByte(Deceleration), lowByte(Deceleration), 0x00, 0x00, 0x00, 0x00};  //Message to send data to PDO2 (acceleration and deceleration)

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2);
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);  
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command    
}

//Function to send new setpoint to drive before reaching the previous setpoint
void send_VAI_goToPosFromActPos(int nodeID, int targetPos,int maxVelocity,int Acceleration,int Deceleration){
  
  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   Here you send the control word, mc-header, target position and maximum velocity
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   Here you send Acceleration and Deceleration
  byte cmdHeader[] = {0x01, 0x03};
  byte ctrlWord[] = {0x00,0x3f};

  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], lowByte(targetPos), highByte(targetPos), lowByte(maxVelocity), highByte(maxVelocity)};
  byte sendDataArray_PDO2[] = {lowByte(Acceleration), highByte(Acceleration), lowByte(Deceleration), highByte(Deceleration), 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  send_Msg(CANID_PDO1, 0, 8, sendDataArray_PDO1);
  send_Msg(CANID_PDO2, 0, 8, sendDataArray_PDO2);  
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}

//Function to home actuator
void sendHoming(){
  byte sendDataArray_PDO1[] = {0x3f, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  //Homing control word = 0x083f
  send_Msg(0x204, 0, 8, sendDataArray_PDO1);
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}

//Function to put drive in operational mode
void operationalMode(){
  byte sendDataArray[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};            //0x01 is the command for putting device in operational mode, and 0x04 is the node-ID
  send_Msg(0x000, 0, 2, sendDataArray);  
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command
}


//Function to switch on
void switchOn(){
  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(0x204, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command  
}

//Function to switch off
void switchOff(){
  byte sendDataArray_2[] = {0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(0x204, 0, 8, sendDataArray_2); 
  byte sndSync2 = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command  
}

//_______________________________________________SETUP_____________________________________________________________________
void setup() {
  //Open serial port:
  Serial.begin(115200);

  Serial.println(F("Serial communication initialized..."));

  //CANBUS SETUP******
  pinMode(CAN0_INT, INPUT);     // Configuring pin for /INT input
  
  // Initialize MCP2515 running at 16MHz with a baudrate of (500kb/s) and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){ 
    Serial.println(F("MCP2515 Initialized Successfully!"));
  }else{ 
    Serial.println(F("Error Init CAN, waiting for connection..."));
    while(true){
      if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
        break;        
      }      
    }
  }

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  Serial.println(F("SETUP..OK"));
}

//_________________________________________________________MAIN__________________________________________________________
void loop() {
  
  if(listenToSerial() != 0){
      
      if(choice == 9 && prevChoice != 9){
        prevChoice = choice;
        Serial.println(F("operational mode.."));
        operationalMode();  
      }
      if(choice == 2 && prevChoice != 2){
        prevChoice = choice;
        Serial.println(F("homing.."));
        sendHoming();  
      }
      if(choice == 3){
        prevChoice = choice;
        //Serial.println(F("go to pos..."));
        send_VAI_gotoPos(0x04, targetPos, maxVel, Acceleration, Deceleration);  
      }
      if(choice == 4 && prevChoice != 4){
        prevChoice = choice;
        Serial.println(F("switch on drive.."));
        switchOn();  
      }
      if(choice == 5 && prevChoice != 5){
        prevChoice = choice;
        Serial.println(F("switch off drive.."));
        switchOff();  
      }
      if(choice == 8 && prevChoice != 8){
        prevChoice = choice;
        send_SIN_VA_GoToPos(0x04, targetPos, maxVel, Acceleration, Deceleration);  
      }
      if(choice == 6){
        prevChoice = choice;
        //Serial.println(F("Getting position.."));
        askPosition(0x04);  
      }
      if(choice == 7){
        send_VAI_gotoPosAndResetForceControl(0x04, targetPos, maxVel, Acceleration, Deceleration);  
      }
      if(choice == 10){
        Force_Ctrl_ChangeTargetForce(0x04, targetForce);  
      }
      if(choice == 11){
        send_VAI_gotoPosWithHigherForceCtrlLimitandTargetForce(0x04, targetPos, maxVel,Acceleration, Deceleration, forceLimit, targetForce);
      }
      if(choice == 1 && prevChoice != 1){
        prevChoice = choice;
        Serial.println(F("operational mode.."));
        operationalMode();  
      } 
  }

  if(!digitalRead(CAN0_INT)){                         // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    if(rxId == 0x184){
      long int actualPos = 0;
      actualPos += ((long int) rxBuf[7] << 24);
      actualPos += ((long int) rxBuf[6] << 16);
      actualPos += ((long int) rxBuf[5] << 8);
      actualPos += ((long int) rxBuf[4]);
      returnFloatPos = actualPos/10000.0f;

      long int statusWord = 0;
      statusWord += ((long int) rxBuf[1] << 8);
      statusWord += ((long int) rxBuf[0]);
      //Serial.println(statusWord, HEX);
      sendSerial(returnFloatPos, statusWord);       
    }
  }
}