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
MCP_CAN CAN0(3);     // Set CS to pin 10 normally. If stacked with ethernet shield v2, bend pin 10 and solder a wire that can plug into port 3 for example, and then change cs pin to 3.


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
bool listenToSerial(){
  
  // Check if a complete line is available
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim(); // Remove any whitespace
    
    if (line.length() == 0) {
      return false;
    }
    
    // Debug: uncomment to see received data
    // Serial.print(F("Received: "));
    // Serial.println(line);
    
    // Parse CSV format: choice,targetPos,maxVel,acc,dec,targetForce,forceLimit
    int commaIndex[6];
    int fieldCount = 0;
    
    // Find comma positions
    for (int i = 0; i < line.length() && fieldCount < 6; i++) {
      if (line.charAt(i) == ',') {
        commaIndex[fieldCount++] = i;
      }
    }
    
    // Validate: need exactly 6 commas for 7 fields
    if (fieldCount != 6) {
      Serial.print(F("Error: Invalid format, expected 7 fields, found "));
      Serial.println(fieldCount + 1);
      return false;
    }
    
    // Parse each field - convert directly to appropriate types
    // Field 1: choice (integer 1-11)
    choice = line.substring(0, commaIndex[0]).toInt();
    
    // Field 2: targetPos in mm (convert to internal units: mm * 10000)
    float targetPos_mm = line.substring(commaIndex[0] + 1, commaIndex[1]).toFloat();
    targetPos = (long)(targetPos_mm * 10000.0f);
    
    // Field 3: maxVel in m/s (convert to internal units: m/s * 1000000)
    float maxVel_ms = line.substring(commaIndex[1] + 1, commaIndex[2]).toFloat();
    maxVel = (unsigned long)(maxVel_ms * 1000000.0f);
    
    // Field 4: Acceleration in m/s² (convert to internal units: m/s² * 100000)
    float acc_ms2 = line.substring(commaIndex[2] + 1, commaIndex[3]).toFloat();
    Acceleration = (unsigned long)(acc_ms2 * 100000.0f);
    
    // Field 5: Deceleration in m/s² (convert to internal units: m/s² * 100000)
    float dec_ms2 = line.substring(commaIndex[3] + 1, commaIndex[4]).toFloat();
    Deceleration = (unsigned long)(dec_ms2 * 100000.0f);
    
    // Field 6: targetForce (convert to internal units: * 10)
    float targetForce_raw = line.substring(commaIndex[4] + 1, commaIndex[5]).toFloat();
    targetForce = (int)(targetForce_raw * 10.0f);
    
    // Field 7: forceLimit (convert to internal units: * 10)
    float forceLimit_raw = line.substring(commaIndex[5] + 1).toFloat();
    forceLimit = (int)(forceLimit_raw * 10.0f);
    
    // Validate choice range
    if (choice < 1 || choice > 11) {
      Serial.print(F("Warning: Invalid choice value "));
      Serial.print(choice);
      Serial.println(F(", expected 1-11"));
      return false;
    }

    // Debug: uncomment to see parsed values
    // Serial.print(F("Parsed - Choice: "));
    // Serial.print(choice);
    // Serial.print(F(", Pos: "));
    // Serial.print(targetPos_mm);
    // Serial.print(F("mm, Vel: "));
    // Serial.print(maxVel_ms);
    // Serial.print(F("m/s, Acc: "));
    // Serial.print(acc_ms2);
    // Serial.print(F(", Dec: "));
    // Serial.print(dec_ms2);
    // Serial.print(F(", Force: "));
    // Serial.print(targetForce_raw);
    // Serial.print(F(", Limit: "));
    // Serial.println(forceLimit_raw);

    return true;        
  }
  return false;
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

//Helper function to send sync message
void sendSync(){
  CAN0.sendMsgBuf(0x80, 0, 0, 0);
}

//Helper function to send PDO messages (pdo2 and pdo3 are optional, pass nullptr if not needed)
void sendPDO(int nodeID, byte* pdo1, byte* pdo2, byte* pdo3){
  if (pdo1 != nullptr) {
    int CANID_PDO1 = 0x200 + nodeID;
    send_Msg(CANID_PDO1, 0, 8, pdo1);
  }
  if (pdo2 != nullptr) {
    int CANID_PDO2 = 0x300 + nodeID;
    send_Msg(CANID_PDO2, 0, 8, pdo2);
  }
  if (pdo3 != nullptr) {
    int CANID_PDO3 = 0x400 + nodeID;
    send_Msg(CANID_PDO3, 0, 8, pdo3);
  }
  sendSync();
}

//Get position from actuator
void askPosition(int nodeID){
  int CANID_TPDO1 = 0x180 + nodeID;

  byte sendDataArray_TPDO1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_Msg(CANID_TPDO1, 0, 8, sendDataArray_TPDO1);
  sendSync();
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
  sendSync();

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], lowByte(targetPos), highByte(targetPos), lowByte(maxVelocity), highByte(maxVelocity)};
  byte sendDataArray_PDO2[] = {lowByte(Acceleration), highByte(Acceleration), lowByte(Deceleration), highByte(Deceleration), 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, nullptr);
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
  sendSync();

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0],pos.b[0],pos.b[1],pos.b[2],pos.b[3]};
  byte sendDataArray_PDO2[] = {vel.b[0],vel.b[1],vel.b[2],vel.b[3],acc.b[0],acc.b[1],acc.b[2],acc.b[3]};
  byte sendDataArray_PDO3[] = {dacc.b[0],dacc.b[1],dacc.b[2],dacc.b[3], 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, sendDataArray_PDO3);
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
  sendSync();

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0],pos.b[0],pos.b[1],pos.b[2],pos.b[3]};
  byte sendDataArray_PDO2[] = {vel.b[0],vel.b[1],vel.b[2],vel.b[3],acc.b[0],acc.b[1],acc.b[2],acc.b[3]};
  byte sendDataArray_PDO3[] = {dacc.b[0],dacc.b[1],dacc.b[2],dacc.b[3], 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, sendDataArray_PDO3);
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
  sendSync();

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], lowByte(targetForce), highByte(targetForce), 0x00, 0x00};
  byte sendDataArray_PDO2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, nullptr);
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
  sendSync();

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0],pos.b[0],pos.b[1],pos.b[2],pos.b[3]};
  byte sendDataArray_PDO2[] = {vel.b[0],vel.b[1],vel.b[2],vel.b[3],acc.b[0],acc.b[1],acc.b[2],acc.b[3]};
  byte sendDataArray_PDO3[] = {lowByte(forceLimit), highByte(forceLimit), lowByte(targetForce), highByte(targetForce),0x00,0x00,0x00,0x00};

  //Sending the data with sync message at the end
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, sendDataArray_PDO3);
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
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, nullptr);
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
  sendSync();

  //creating the array of data to be transmitted
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], lowByte(targetPos), highByte(targetPos), lowByte(maxVelocity), highByte(maxVelocity)};
  byte sendDataArray_PDO2[] = {lowByte(Acceleration), highByte(Acceleration), lowByte(Deceleration), highByte(Deceleration), 0x00, 0x00, 0x00, 0x00};

  //Sending the data with sync message at the end
  sendPDO(nodeID, sendDataArray_PDO1, sendDataArray_PDO2, nullptr);
}

//Function to home actuator
void sendHoming(){
  byte sendDataArray_PDO1[] = {0x3f, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  //Homing control word = 0x083f
  send_Msg(0x204, 0, 8, sendDataArray_PDO1);
  sendSync();
}

//Function to put drive in operational mode
void operationalMode(){
  byte sendDataArray[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};            //0x01 is the command for putting device in operational mode, and 0x04 is the node-ID
  send_Msg(0x000, 0, 2, sendDataArray);  
  sendSync();
}


//Function to switch on
void switchOn(){
  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(0x204, 0, 8, sendDataArray_2); 
  sendSync();
}

//Function to switch off
void switchOff(){
  byte sendDataArray_2[] = {0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_Msg(0x204, 0, 8, sendDataArray_2); 
  sendSync();
}

//_______________________________________________SETUP_____________________________________________________________________
void setup() {
  //Open serial port:
  Serial.begin(115200);

  Serial.println(F("Serial communication initialized..."));

  //CANBUS SETUP******
  pinMode(CAN0_INT, INPUT);     // Configuring pin for /INT input
  
  // Initialize MCP2515 ng at 16MHz with a baudrate of (500kb/s) and the masks and filters disabled.
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

    if (listenToSerial() != 0) {

        switch (choice) {

            // -------- ONE-SHOT COMMANDS --------
            case 1:
            case 9:
                if (prevChoice != choice) {
                    Serial.println(F("operational mode.."));
                    operationalMode();
                    prevChoice = choice;
                }
                break;

            case 2:
                if (prevChoice != 2) {
                    Serial.println(F("homing.."));
                    sendHoming();
                    prevChoice = 2;
                }
                break;

            case 4:
                if (prevChoice != 4) {
                    Serial.println(F("switch on drive.."));
                    switchOn();
                    prevChoice = 4;
                }
                break;

            case 5:
                if (prevChoice != 5) {
                    Serial.println(F("switch off drive.."));
                    switchOff();
                    prevChoice = 5;
                }
                break;

            case 8:
                if (prevChoice != 8) {
                    send_SIN_VA_GoToPos(0x04, targetPos, maxVel, Acceleration, Deceleration);
                    prevChoice = 8;
                }
                break;


            // -------- REPEATING COMMANDS --------
            case 3:
                send_VAI_gotoPos(0x04, targetPos, maxVel, Acceleration, Deceleration);
                break;

            case 6:
                askPosition(0x04);
                break;

            case 7:
                send_VAI_gotoPosAndResetForceControl(0x04, targetPos, maxVel, Acceleration, Deceleration);
                break;

            case 10:
                Force_Ctrl_ChangeTargetForce(0x04, targetForce);
                break;

            case 11:
                send_VAI_gotoPosWithHigherForceCtrlLimitandTargetForce(
                    0x04, targetPos, maxVel, Acceleration, Deceleration, forceLimit, targetForce
                );
                break;

            default:
                break;
        }
    }

    if (!digitalRead(CAN0_INT)) {

        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        if (rxId == 0x184) {

            long int actualPos = 0;
            actualPos |= ((long int)rxBuf[7] << 24);
            actualPos |= ((long int)rxBuf[6] << 16);
            actualPos |= ((long int)rxBuf[5] << 8);
            actualPos |= ((long int)rxBuf[4]);

            returnFloatPos = actualPos / 10000.0f;

            long int statusWord = 0;
            statusWord |= ((long int)rxBuf[1] << 8);
            statusWord |= ((long int)rxBuf[0]);

            sendSerial(returnFloatPos, statusWord);
        }
    }
}
