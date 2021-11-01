#include <RFM69.h>
#include <SPI.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID     101   // Must be the same for all nodes (0 to 255)
#define MYNODEID      22   // My node ID (0 to 255)
#define TONODEID      0   // Destination node ID (0 to 254, 255 = broadcast)

// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):

#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):

#define USEACK        false // Request ACKs or not

// Packet sent/received indicator LED (optional):

#define LED           9 // LED positive pin
#define GND           8 // LED ground pin

// Create a library object for our RFM69HCW module:

#define NUM_ROBOTS 40
#define NUM_BALLS 20




struct RobotPose
{
  boolean valid;
  int16_t ID;
  int16_t x;
  int16_t y;
  int16_t theta;
  int16_t zombie;
  int16_t healing;
};

struct BallPosition
{
  int16_t x;
  int16_t y; 
  int16_t hue; 
};


char lbuf[1024];
void printRobotPose (RobotPose pose);
void printBallPositions(int num, BallPosition (&pos)[NUM_BALLS]);

const int RF69_CHIPKIT_IRQ = 38;
RFM69 radio(RF69_SPI_CS, RF69_CHIPKIT_IRQ, true);
int numRobots = 0;
RobotPose robotPoses[NUM_ROBOTS];
int numBalls = 0;
BallPosition ballPositions[NUM_BALLS];


void setupRadio(int teamID)
{
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW  
}

int getNumRobots()
{
  return numRobots;
}

RobotPose getRobotPose (int robotID)
{
  RobotPose retval;  
  retval = robotPoses[robotID];
  return retval;
}

int getBallPositions (BallPosition (&pos)[NUM_BALLS])
{  
  for (int i = 0 ; i < numBalls ; i++)
  {
    pos[i] = ballPositions[i];    
  }
  return numBalls;
}

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
}


void printRFMMessage(int len, uint8_t* buff)
{
  Serial.print("RECV: " );
  Serial.print(len);
  Serial.print("    ");
  PrintHex8(buff, len);
  /*for(int i = 0 ; i < len ; i++)
  {
    PrintHex8(buff[i], HEX);
  }*/
  Serial.println("");
}

union {
   int16_t value;
   byte arr[sizeof(int16_t)];
} int16_byte_converter;

int16_t convert_bytes_to_int16(char* buf)
{
  int16_byte_converter.arr[1] = buf[0];
  int16_byte_converter.arr[0] = buf[1];
  return int16_byte_converter.value;
}

void updateRobotPoseAndBallPositions (void)
{
  
  // Read from the radio to get a packet
  //   (1) Read packets until a $ is found
  //   (2) read packets until a * is found
  //   (3) copy the data into the current double-buffer memory locations 
  //   (4) iterate through the buffer and pick off the position and rotation data. It is an integer for each field.
  bool msg_start_found = false;
  bool msg_end_found = false;
  int idx = 0;
  int error = 0;
  while (error == 0)
  {
    if (radio.receiveDone()) // Got one!
    {
      printRFMMessage(radio.DATALEN, (uint8_t*)radio.DATA);

      
      // If the start of a message hasn't been found, look for the '$'
      if (!msg_start_found)
      {
        char firstByte = (char)radio.DATA[0];
        char secondByte = (char)radio.DATA[1];
        if (firstByte == '$' && secondByte == '$') 
        {
          Serial.println("FOUND START BYTES");
          msg_start_found = true;
          idx = 0;
        }       

        char lastByte = (char)radio.DATA[radio.DATALEN-1];
        char secondLastByte = (char)radio.DATA[radio.DATALEN-2];
        if (msg_start_found ==true && lastByte == '*' && secondLastByte == '*')
        {
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          Serial.println("FOUND END BYTE SAME PACKET");
          msg_start_found = false;
          break;
        }
        else if (msg_start_found == true)
        {
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          idx += radio.DATALEN;
          continue;
        }
      }    
      else if (msg_start_found && radio.DATALEN >= 2)
      {
        // Check to see if the message has ended
        char lastByte = (char)radio.DATA[radio.DATALEN-1];
        char secondLastByte = (char)radio.DATA[radio.DATALEN-2];
        if (lastByte == '*' && secondLastByte == '*')
        {
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          Serial.println("FOUND END BYTE OTHER PACKET");
          msg_start_found = false;
          break;
        }
        // If the message hasn't ended, copy all the data into the buffer
        else
        {       
          //Serial.print("MSG RCVD IDX: ");
          //Serial.println(idx);
          memcpy(&(lbuf[idx]), (char*)radio.DATA, radio.DATALEN);
          idx += radio.DATALEN;        
        }
      }
      else
      {
        /*
        Serial.println("NOT A GOOD STATE");
        Serial.print("MSF:"); Serial.print(msg_start_found);
        Serial.print("   RDL:"); Serial.print(radio.DATALEN);
        Serial.print("   IDX:"); Serial.print(idx);
        Serial.println("");
        */
      }
    }
    
  }

  for(int i = 0 ; i < NUM_ROBOTS ; i++)
  {
    robotPoses[i].valid = false;
  }
  
  int start_offset = 2;
  numRobots = convert_bytes_to_int16(&lbuf[start_offset + 0]);
  for(int i = 0 ; i < numRobots ; i++)
  {
    int bidx = start_offset + 2 + i*12;
    //Serial.print("bidx: ");
    //Serial.println(bidx);
    int16_t ID = convert_bytes_to_int16(&lbuf[bidx + 0]); //lbuf[bidx + 0] << 8 | lbuf[bidx + 1];

    //Serial.print("ID: ");
    //Serial.println(ID);
    
    int16_t  X = convert_bytes_to_int16(&lbuf[bidx + 2]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];
    int16_t  Y = convert_bytes_to_int16(&lbuf[bidx + 4]); //lbuf[bidx + 4] << 8 | lbuf[bidx + 5];
    int16_t  R = convert_bytes_to_int16(&lbuf[bidx + 6]);
    int16_t  Z = convert_bytes_to_int16(&lbuf[bidx + 8]);
    int16_t  H = convert_bytes_to_int16(&lbuf[bidx + 10]);

    //note:: changed ID index to i, cause ID seemed redundant
    robotPoses[i].valid = true;
    robotPoses[i].ID = ID;
    robotPoses[i].x = X;
    robotPoses[i].y = Y;
    robotPoses[i].theta = R;    
    robotPoses[i].zombie = Z;    
    robotPoses[i].healing = H;    
  }

  int nidx = start_offset + 2 + numRobots*12;
  numBalls = convert_bytes_to_int16(&lbuf[nidx+0]);
  
  for(int i = 0 ; i < numBalls ; i++)
  {
    int bidx = nidx + 2 + i*6;
    int16_t X = convert_bytes_to_int16(&lbuf[bidx + 0]); //lbuf[bidx + 0] << 8 | lbuf[bidx + 1];
    int16_t Y = convert_bytes_to_int16(&lbuf[bidx + 2]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];
    int16_t H = convert_bytes_to_int16(&lbuf[bidx + 4]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];

    ballPositions[i].x = X;
    ballPositions[i].y = Y;
    ballPositions[i].hue = H;
  }

  /*
  Serial.print("NUM ROBOTS: ");
  Serial.println(numRobots);
  for(int i = 0 ; i < NUM_ROBOTS ; i++)
  {
    if (robotPoses[i].valid == true)
      printRobotPose(robotPoses[i]);
  }

  Serial.print("NUM BALLS: ");
  Serial.println(numBalls);
  printBallPositions(numBalls, ballPositions);
*/
  
}

void printRobotPose (RobotPose pose)
{
  Serial.print("Robot: ");
  Serial.print(pose.ID);
    Serial.print("   X: ");
    Serial.print(pose.x);
    Serial.print("   Y: ");
    Serial.print(pose.y);
    Serial.print("   R: ");
    Serial.print(pose.theta);
    Serial.print("   Z: ");
    Serial.print(pose.zombie);
    Serial.print("   H: ");
    Serial.print(pose.healing);
    
    Serial.println("");
}

void printBallPositions(int num, BallPosition (&pos)[NUM_BALLS])
{
  for (int i = 0 ; i < num ; i++)
  {
    Serial.print("Ball: ");
    Serial.print(i);
    Serial.print("   X: ");
    Serial.print(pos[i].x);
    Serial.print("   Y: ");
    Serial.print(pos[i].y);
    Serial.print("   H: ");
    Serial.print(pos[i].hue);
    Serial.println("");
  }
}

void ME401_Radio_initialize(void)
{
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
}
