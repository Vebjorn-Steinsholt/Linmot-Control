//Linmot UDP2CAN-BUS Controller
//Robert Opland
//NTNU - Norwegian University of Science and Technology
//Version 0.1
//Still some cleaning up to do, and some optimization would be nice...

#include <mcp_can.h>    //Using this library for the canbus communication
#include <SPI.h>
#include "Wire.h"
#include <Ethernet.h>   //Using this library for the UDP communication
#include <EthernetUdp.h>
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
int targetPos = 0;
int maxVel = 0;
int Acceleration = 0;
int Deceleration = 0;
int commandChoice = 0;
int stopGoToZero = 0;
byte txBuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


//Ethernet/UDP stuff
// Enter a MAC address and IP address for your controller below. The IP address will be dependent on your local network:
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x95, 0xD7}; //<<< ENTER YOUR ARDUINO'S MAC ADDRESS HERE!!!
IPAddress ip(192,168,1,5); //<<< ENTER YOUR ARDUINO'S IP ADDRESS HERE!!!
IPAddress subnet(255, 255, 255, 0);
IPAddress myDns(8, 8, 8, 8);

const unsigned int localPort = 8888;      // local port to listen on

const byte remoteIp[] = {192,168,1,199};  //This is the IP of the pc that communicates by UDP(HLCC)

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


void tryReconnectUDP(){  
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("Ethernet shield was not found.  Sorry, can't run without hardware. waiting for shield to respond..."));
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
      if(Ethernet.hardwareStatus() != EthernetNoHardware){
        break;  
      }
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println(F("Ethernet cable is not connected. Waiting for cable connection..."));
    while (true) {
      delay(1); // do nothing, no point running without Ethernet connection
      if(Ethernet.linkStatus() == LinkON){
        break;  
      }
    }
  }   
}



//Function to listen to UDP messages:__________________________________________
int listenToUdp(){
  
  // buffer for receiving data from UDP
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet  
  float packetVal_1 = 0;
  float packetVal_2 = 0;
  float packetVal_3 = 0;
  float packetVal_4 = 0;
  float packetVal_5 = 0;
  float packetVal_6 = 0;
  
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {

    //pointers for grabbing values from packetBuffer, swapping endian and then stuffing them into packetVal
    uint8_t * dataPtr_1 = (uint8_t *) &packetVal_1;
    uint8_t * dataPtr_2 = (uint8_t *) &packetVal_2;
    uint8_t * dataPtr_3 = (uint8_t *) &packetVal_3;
    uint8_t * dataPtr_4 = (uint8_t *) &packetVal_4;
    uint8_t * dataPtr_5 = (uint8_t *) &packetVal_5;
    uint8_t * dataPtr_6 = (uint8_t *) &packetVal_6;    
    
    //read the packet into packetBuffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);     

    //Swapping from big to little endian and turning into float32. to turn of the swap match the arrays: dataPtr[0] = packetBuffer[0]
    dataPtr_1[0] = packetBuffer[3];
    dataPtr_1[1] = packetBuffer[2];
    dataPtr_1[2] = packetBuffer[1];
    dataPtr_1[3] = packetBuffer[0];

    dataPtr_2[0] = packetBuffer[7];
    dataPtr_2[1] = packetBuffer[6];
    dataPtr_2[2] = packetBuffer[5];
    dataPtr_2[3] = packetBuffer[4];

    dataPtr_3[0] = packetBuffer[11];
    dataPtr_3[1] = packetBuffer[10];
    dataPtr_3[2] = packetBuffer[9];
    dataPtr_3[3] = packetBuffer[8];

    dataPtr_4[0] = packetBuffer[15];
    dataPtr_4[1] = packetBuffer[14];
    dataPtr_4[2] = packetBuffer[13];
    dataPtr_4[3] = packetBuffer[12];

    dataPtr_5[0] = packetBuffer[19];
    dataPtr_5[1] = packetBuffer[18];
    dataPtr_5[2] = packetBuffer[17];
    dataPtr_5[3] = packetBuffer[16];

    dataPtr_6[0] = packetBuffer[23];
    dataPtr_6[1] = packetBuffer[22];
    dataPtr_6[2] = packetBuffer[21];
    dataPtr_6[3] = packetBuffer[20];

    //Scaling variables from UDP to make some common sense: pos=mm, V=m/s, acc/dcc=m/s^2. These are the units you send from UDP source
    choice        = packetVal_1;          //Valid choices: 1-6
    targetPos     = packetVal_2*10;       //Wanted position in millimeters
    maxVel        = packetVal_3*1000;     //Maximum velocity in m/s
    Acceleration  = packetVal_4*10;       //Acceleration in m/s^2
    Deceleration  = packetVal_5*10;       //Deceleration in m/s^2
    stopGoToZero  = packetVal_6;          //TODO: If this variable goes high, have the motor go to zero and then turn it off? or something similar...

//    Serial.println(F("Receiving from python.."));
//    Serial.print(choice);
//    Serial.print(F(" "));
//    Serial.print(targetPos);
//    Serial.print(F(" "));
//    Serial.print(maxVel);
//    Serial.print(F(" "));
//    Serial.print(Acceleration);
//    Serial.print(F(" "));
//    Serial.println(Deceleration);
    
    
    return(packetSize);        
  } 
}

//Function to send UDP messages___________________________________________________
void sendUdp(float currPos,long int statWord){
  uint16_t remoteConnectPort = 61235;
  float txVal = currPos;
  float txStatVal = statWord;
  Udp.beginPacket(remoteIp, remoteConnectPort);      //Udp.remotePort() if you want to send to the same remote port you received from

  //rebuilding the two floats for actual position and status-word.
  for(int i = 0; i < 1; i++){
    uint8_t * dataPtr = (uint8_t *) &txVal;
    Udp.write(dataPtr[3]);
    Udp.write(dataPtr[2]);
    Udp.write(dataPtr[1]);
    Udp.write(dataPtr[0]);
    uint8_t * statDataPtr = (uint8_t *) &txStatVal;
    Udp.write(statDataPtr[3]);
    Udp.write(statDataPtr[2]);
    Udp.write(statDataPtr[1]);
    Udp.write(statDataPtr[0]);
  }  
  Udp.endPacket(); 
}

// send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send__________________________________
void send_Msg(int Can_Id, int ExtID, int dL, byte dataArray[]){
  int CANID = Can_Id;
  int external = ExtID;
  int dataLength = dL;
  byte msgData[dataLength] = {int(dataArray[0]),int(dataArray[1]),int(dataArray[2]),int(dataArray[3]),int(dataArray[4]),int(dataArray[5]),int(dataArray[6]),int(dataArray[7])}; 
  byte sndStat = CAN0.sendMsgBuf(CANID, external, dataLength, msgData);
//  char bufferMsg[100];
//  if(sndStat == CAN_OK){
//    sprintf(bufferMsg, "Send standard ID:   0x%02X  DLC: %u  Data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", CANID, dataLength,(msgData[0]),(msgData[1]),(msgData[2]),(msgData[3]),(msgData[4]),(msgData[5]),(msgData[6]),(msgData[7]));
//    Serial.println(bufferMsg);  
//  }else{
//    Serial.println(F("Error Sending Message..."));
//  }    
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

//Function to send sine motion command to drive
void send_SIN_VA_GoToPos(int nodeID, byte targetPos[],byte maxVelocity[], byte Acceleration[], byte Deceleration[]){
  
  //Setting the canid, header and control word
  int CANID_PDO1 = 0x200 + nodeID;   //Can_id = COB_ID + node_ID(200 + 4  for pdo1)   PDO1 is the 0x200 series
  int CANID_PDO2 = 0x300 + nodeID;   //Can_id = COB_ID + node_ID(300 + 4  for pdo2)   PDO2 is the 0x300 series
  byte cmdHeader[] = {0x0E, 0x04};
  byte ctrlWord[] = {0x00,0x3f};

  //creating the array of data to be transmitted
  byte sendDataArray_2[] = {0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    //Message to enter operational state
  byte sendDataArray_PDO1[] = {ctrlWord[1],ctrlWord[0], cmdHeader[1],cmdHeader[0], targetPos[1], targetPos[0], maxVelocity[1],maxVelocity[0]};  //Message to send data to PDO1(header, target pos, max velocity)
  byte sendDataArray_PDO2[] = {Acceleration[1],Acceleration[0], Deceleration[1], Deceleration[0], 0x00, 0x00, 0x00, 0x00};  //Message to send data to PDO2 (acceleration and deceleration)

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
  send_Msg(0x00, 0, 2, sendDataArray);  
  byte sndSync = CAN0.sendMsgBuf(0x80, 0, 0, 0);   //Sending sync message to actually get the linmot drive to accept/run command   
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

  //ETHERNET SHIELD SETUP***
    // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(10);  // Most Arduino shields

  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("Ethernet shield was not found.  Sorry, can't run without hardware. waiting for shield to respond..."));
    while (true) {
      //delay(1); // do nothing, no point running without Ethernet hardware
      if(Ethernet.hardwareStatus() != EthernetNoHardware){
        break;  
      }
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println(F("Ethernet cable is not connected. Waiting for cable connection..."));
    while (true) {
      //delay(1); // do nothing, no point running without Ethernet connection
      if(Ethernet.linkStatus() == LinkON){
        break;  
      }
    }
  }
  if(Ethernet.linkStatus() == LinkON){
    Serial.println(F("Ethernet link established.."));
    
    // print out Arduino's IP address, subnet mask, gateway's IP address, and DNS server's IP address
    Serial.print(F("- Arduino's IP address   : "));
    Serial.println(Ethernet.localIP());
  
    Serial.print(F("- Gateway's IP address   : "));
    Serial.println(Ethernet.gatewayIP());
  
    Serial.print(F("- Network's subnet mask  : "));
    Serial.println(Ethernet.subnetMask());
  
    Serial.print(F("- DNS server's IP address: "));
    Serial.println(Ethernet.dnsServerIP());     
  }
  

  // start UDP
  Udp.begin(localPort);
  
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
  tryReconnectUDP();
  
  if(listenToUdp() != 0){
      
      if(choice == 1 && prevChoice != 1){
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
        send_VAI_16Bit_gotoPos(0x04, targetPos, maxVel, Acceleration, Deceleration);  
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
      if(choice == 6){
        prevChoice = choice;
        //Serial.println(F("Getting position.."));
        askPosition(0x04);  
      }
      if(choice == 7 && prevChoice != 7){
        prevChoice = choice;
        Serial.println(F("Receiving from python.."));
        Serial.print(choice);
        Serial.print(F(" "));
        Serial.print(targetPos);
        Serial.print(F(" "));
        Serial.print(maxVel);
        Serial.print(F(" "));
        Serial.print(Acceleration);
        Serial.print(F(" "));
        Serial.println(Deceleration);
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
      sendUdp(returnFloatPos, statusWord);       
    }
    
//    if(len != 0){
//      if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
//        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
//      else
//        sprintf(msgString, "Receive Standard ID: 0x%.3lX  DLC: %1d  Data:", rxId, len);
//    
//      Serial.print(msgString);
//    
//      if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
//        sprintf(msgString, " REMOTE REQUEST FRAME");
//        Serial.print(msgString);
//      } else {
//        for(byte i = 0; i<len; i++){
//          sprintf(msgString, " 0x%.2X", rxBuf[i]);
//          Serial.print(msgString);
//        }
//      }          
//      Serial.println();
//      len = 0;
//    }
  }
}
