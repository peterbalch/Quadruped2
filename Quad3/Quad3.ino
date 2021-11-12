//#define UseEEPROM

#ifdef UseEEPROM
  #include <EEPROM.h>
#else
  #include "gaits.h"
#endif

#include "myServo.h"

const byte maxAngle = 180;
Servo servos[MAX_SERVOS];

const byte cmdAck   = 255;
const byte cmdRecvEEPROM  = maxAngle + MAX_SERVOS + 2;
const byte cmdSendEEPROM  = maxAngle + MAX_SERVOS + 3;
const byte cmdRunGait     = maxAngle + MAX_SERVOS + 4;
const byte cmdMaxSpeed    = maxAngle + MAX_SERVOS + 5;
const byte cmdMinTime     = maxAngle + MAX_SERVOS + 6;
const byte cmdBlendRun    = maxAngle + MAX_SERVOS + 7;
const byte cmdStopNow     = maxAngle + MAX_SERVOS + 8;
const byte cmdStopWhen    = maxAngle + MAX_SERVOS + 9;
const byte cmdQueryPose   = maxAngle + MAX_SERVOS + 10;
const byte cmdAtTargets   = maxAngle + MAX_SERVOS + 11;

const int EEPROMsize  = 1024;
const byte MaxRuns = 10; // max num gaits running simultaneously

const byte ServoPin[MAX_SERVOS] = {
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  A0,
  A1,
  A2,
  A3,
  A4,
  A5,
  11,
  12,
  13
};

int ServoPos[MAX_SERVOS] = {9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000};
int ServoTarg[MAX_SERVOS] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
int ServoSpeed[MAX_SERVOS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const int cent = 100;

const int numGaitRecs = 10; // max simultaneous gaits running
struct TGaitRec { // specifies each running gait 
    int GaitTable; // addr of start of gait table; set to -1 if not in use
    int BlendGaitTable; // addr of start of gait table to blend with; set to -1 if not in use
    byte curPose; // target pose currently being approached or has been reached
    byte finalPose; // table is run until targets of this pose have been reached; 255 if "run forever"
    byte MaxServoSpeed; // max rate of movement of any servo
    byte MinApproachTime; // min time it must take to reach the next pose
    byte share; // if blending with another GaitTable then this tables "share"; other table is 100-share
};
TGaitRec GaitRec[numGaitRecs];

//-----------------------------------------------------------
// setServo
//   sets a servo PWM to a particular position
//-----------------------------------------------------------
void setServo(byte sv, byte pos) {
  if (!servos[sv].attached())
    servos[sv].attach(ServoPin[sv]);
  servos[sv].write(pos);
}

//-----------------------------------------------------------
// GaitTableAddr()
//  address of gait N in gait tables
//-----------------------------------------------------------
int GaitTableAddress(int gait_num) {
  #ifdef UseEEPROM
    int addr = 0;
    while (gait_num) {
      int n = NumPoses(addr);
      if (n == 0)
        return -1;
      addr += 5 + n * CountSet(ServoSet(addr));
      gait_num--;
    }
    return addr;
  #else
    return GaitTableAddr[gait_num];
  #endif
}

//-----------------------------------------------------------
// MoveServo
//   move one servo slowly to target position
//   return true if has reached target
//-----------------------------------------------------------
bool MoveServo(int servo) {
  if (abs(ServoPos[servo] - ServoTarg[servo]*cent) < cent)
  {
    if (ServoPos[servo] != ServoTarg[servo]*cent)
      setServo(servo, ServoPos[servo] / cent);
    ServoPos[servo] = ServoTarg[servo]*cent;
    ServoSpeed[servo] = 0;
    return true;
  }

  if (ServoPos[servo] != ServoTarg[servo]*cent)
  {
    if (ServoSpeed[servo] == 0)
    {
      if (ServoTarg[servo]*cent - ServoPos[servo] > 0)
        ServoPos[servo] = ServoPos[servo] + cent; else 
      if (ServoTarg[servo]*cent - ServoPos[servo] < 0)
        ServoPos[servo] = ServoPos[servo] - cent;
    } else
      ServoPos[servo] = (ServoPos[servo] + ServoSpeed[servo]);

    setServo(servo, ServoPos[servo] / cent);
  }
  return false;
}

//-----------------------------------------------------------
// PollServos()
//   move all servos slowly to target positions
//   return true if all have reached target
//-----------------------------------------------------------
bool PollServos() {
  const int period = 10;
  static unsigned long NextTime = 0;

  if (millis() - NextTime > period) {
    NextTime = millis();

    bool done = true;
    for (int sv = 0; sv < MAX_SERVOS; sv++)
      done = MoveServo(sv) && done;
    return done;
  }

  return false;
}

//-----------------------------------------------------------
// StopNow()
//  stop running gait immediately
//-----------------------------------------------------------
void StopNow(int gait_addr) {
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
  if (GaitRec[iGait].GaitTable == gait_addr) {
    int servo = 0;
    for (long s = ServoSet(gait_addr); s != 0; s = s >> 1) {
      if (s & 1) 
        ServoTarg[servo] = ServoPos[servo] / cent;
      servo++;
    }
    GaitRec[iGait].GaitTable = -1;
  }
}

//-----------------------------------------------------------
// StopWhen()
//  stop running gait when it reaches the targets of poseN
//-----------------------------------------------------------
void StopWhen(int gait_addr, byte poseN) {
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
    if (GaitRec[iGait].GaitTable == gait_addr) 
      GaitRec[iGait].finalPose = poseN;
}

//-----------------------------------------------------------
// QueryPose()
//  returns the current pose of the gait
//  returns 255 if gait not running
//-----------------------------------------------------------
byte QueryPose(int gait_addr) {
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
  if (GaitRec[iGait].GaitTable == gait_addr) 
    return GaitRec[iGait].curPose;
  return 255;  
}

//-----------------------------------------------------------
// AtTargets()
//  returns 1 if servos positions same as target positions 
//  returns 0 if servos positions not same as target positions 
//  returns 255 if gait not running
//-----------------------------------------------------------
byte AtTargets(int gait_addr) {
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
  if (GaitRec[iGait].GaitTable == gait_addr) {
    int ans = 1;
    int servo = 0;
    for (long s = ServoSet(gait_addr); s != 0; s = s >> 1) {
      if ((s & 1) && (abs(ServoPos[servo] - ServoTarg[servo]*cent) >= cent))
        ans = 0;
      servo++;
    }
    return ans;
  }
  return 255;
}

//-----------------------------------------------------------
// recvByte()
//  wait for a byte from the PC
//-----------------------------------------------------------
byte recvByte() {
  while (! Serial.available()) ;
  byte b = Serial.read();
  return b;
}

//-----------------------------------------------------------
// recvByteAck()
//   wait for a byte from the PC
//   send an Ack back
//-----------------------------------------------------------
byte recvByteAck() {
  byte b = recvByte();
  Serial.write(cmdAck);
  return b;
}

//-----------------------------------------------------------
// recvInt()
//   wait for a 2-byte int from the PC
//-----------------------------------------------------------
int recvInt() {
  int i = recvByteAck();
  i = i + (recvByteAck() << 8);
  return i;
}

//-----------------------------------------------------------
// recvLong()
//   wait for a 4-byte long from the PC
//-----------------------------------------------------------
long recvLong() {
  long i = recvByteAck();
  i = i + ((long)recvByteAck() << 8);
  i = i + ((long)recvByteAck() << 16);
  i = i + ((long)recvByteAck() << 24);
  return i;
}

//-----------------------------------------------------------
// CountSet
//    how many bits of a long are 1
//-----------------------------------------------------------
byte CountSet(long set) {
  int n = 0;
  while (set) {
    n += set & 1;
    set >>= 1;
  }
  return n;
}

//-----------------------------------------------------------
// RecvEEPROM()
//   get bytes from the PC and write to EEPROM
//-----------------------------------------------------------
#ifdef UseEEPROM
void RecvEEPROM() {
  Serial.write(cmdAck);
  int n = recvInt();
  for (int addr = 0; addr < n; addr++)
    EEPROM.update(addr, recvByteAck());
}
#endif

//-----------------------------------------------------------
// SendEEPROM()
//   get bytes from the EEPROM and send to PC
//-----------------------------------------------------------
#ifdef UseEEPROM
void SendEEPROM() {
  for (int addr = 0; addr < EEPROMsize; addr++)
    Serial.write(EEPROM.read(addr));
  Serial.write(cmdAck);
}
#endif

//-----------------------------------------------------------
// ReadTableByte
//   get a byte from a gait table PROGMEM
//-----------------------------------------------------------
byte ReadTableByte(int addr) {
  #ifdef UseEEPROM
    return EEPROM.read(addr);
  #else
    return pgm_read_byte_near(addr);
  #endif
}

//-----------------------------------------------------------
// ReadTableLong
//   get a byte from a gait table PROGMEM
//-----------------------------------------------------------
long ReadTableLong(int addr) {
  #ifdef UseEEPROM
    return 
      ((long)EEPROM.read(addr++)) +
      (((long)EEPROM.read(addr++)) << 8)+
      (((long)EEPROM.read(addr++)) << 16)+
      (((long)EEPROM.read(addr++)) << 24);
  #else
    return pgm_read_dword_near(addr);
  #endif
}

//-----------------------------------------------------------
// NumPoses
//   how many Poses in a gait 
//-----------------------------------------------------------
byte NumPoses(int gait) {
  return ReadTableByte(gait);
}

//-----------------------------------------------------------
// ServoSet
//   return the servo set of a gait 
//   long ServoSet; one bit for each servo used in this gait
//-----------------------------------------------------------
long ServoSet(int gait) {
  gait++; // byte NumPoses
  return ReadTableLong(gait);
}

//-----------------------------------------------------------
// GetPose
//   get a pose of a gait in PROGMEM or EEPROM into ServoTarg
//   calculate speed of each servo 
//-----------------------------------------------------------
void GetPose(int gait, int blend_gait, byte pose, byte aMinApproachTime, byte aMaxServoSpeed, byte shareA) {
  if (pose >= NumPoses(gait))
    return;

  if (blend_gait < 0) {
    long s = ServoSet(gait);
    int p = gait + 5 + pose*CountSet(s); // 5 == byte NumPoses + long ServoSet
    for (int sv = 0; sv < MAX_SERVOS; sv++) {
      if (s & 1) {
        ServoTarg[sv] = ReadTableByte(p++);
      }
      s = s >> 1;
    }
  } else {
    long s1 = ServoSet(gait);
    long s2 = ServoSet(blend_gait);
    int p1 = gait + 5 + pose*CountSet(s1); 
    int p2 = blend_gait + 5 + pose*CountSet(s2); 
    for (int sv = 0; sv < MAX_SERVOS; sv++) {
      if (s1 & s2 & 1) 
        ServoTarg[sv] = (ReadTableByte(p1++)*shareA + ReadTableByte(p2++)*(100-shareA)) / 100; 
      else if (s1 & 1) 
        ServoTarg[sv] = ReadTableByte(p1++); 
      else if (s2 & 1) 
        ServoTarg[sv] = ReadTableByte(p2++); 

      s1 = s1 >> 1;
      s2 = s2 >> 1;
    }
  }

  int n = 0;
  long s = ServoSet(gait);
  for (int sv = 0; sv < MAX_SERVOS; sv++) {
    if (s & 1) {
      n = max(n,(abs(ServoPos[sv]-ServoTarg[sv]*cent) / (cent / 50) / aMaxServoSpeed));
    }
    s = s >> 1;
  }

  if (n > 0)
    n = max(n,aMinApproachTime);

  s = ServoSet(gait);
  for (int sv = 0; sv < MAX_SERVOS; sv++) {
    if (s & 1) {
      if (n == 0)
        ServoSpeed[sv] = 0; else
        ServoSpeed[sv] = ((ServoTarg[sv]*cent-ServoPos[sv]) / n);
    }
    s = s >> 1;
  }
}

//-----------------------------------------------------------
// RunGaits
//   runs the gaits tables
//-----------------------------------------------------------
void RunGaits() {
  PollServos();
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
  if ((GaitRec[iGait].GaitTable >= 0) && AtTargets(GaitRec[iGait].GaitTable)) {
    GaitRec[iGait].curPose++;
    if (GaitRec[iGait].curPose >= NumPoses(GaitRec[iGait].GaitTable))
      GaitRec[iGait].curPose = 0;
    GetPose(GaitRec[iGait].GaitTable, GaitRec[iGait].BlendGaitTable, GaitRec[iGait].curPose, 
      GaitRec[iGait].MinApproachTime, GaitRec[iGait].MaxServoSpeed, GaitRec[iGait].share);

    if (GaitRec[iGait].curPose == GaitRec[iGait].finalPose)
      GaitRec[iGait].GaitTable = -1;
  }
}

//-----------------------------------------------------------
// StartRunGait
//   gait_addr: name of array in gait table
//   blend_addr: name of array in gait table to be blended; may be -1
//   poseStart: start at pose N; if 255 then continue from where you are
//   poseEnd: stop when you reach pose M; if 255 then continue forever
//   aMaxServoSpeed: max speed for any servo
//   MinApproachTime: min time any pose transition can take
//   aShare: when blended, share (of main gait) 1..100 (blended gait gets 100-share)
//-----------------------------------------------------------
void StartRunGait(int gait_addr, int blend_addr, byte poseStart, byte poseEnd, byte aMaxServoSpeed, byte aMinApproachTime, byte aShare) {  
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
    if (GaitRec[iGait].GaitTable == gait_addr)
      GaitRec[iGait].GaitTable = -1;


  for (int iGait = 0; iGait < numGaitRecs; iGait++)
  if (GaitRec[iGait].GaitTable < 0) {
    GaitRec[iGait].GaitTable = gait_addr;
    GaitRec[iGait].BlendGaitTable = blend_addr;
    if (poseStart < 255)
      GaitRec[iGait].curPose = poseStart;
    GaitRec[iGait].finalPose = poseEnd;
    GaitRec[iGait].MaxServoSpeed = constrain(aMaxServoSpeed,1,100);
    GaitRec[iGait].MinApproachTime = constrain(aMinApproachTime,0,100);
    GaitRec[iGait].share = constrain(aShare,1,100);
    return;
  }
}

//-----------------------------------------------------------
// recvCmd()
//   get command byte from the PC and act on it
//-----------------------------------------------------------
void recvCmd() {
  byte i,i2,i3,i4,i5;
  static byte pin = 0;  
  if (Serial.available() > 0) {
    byte b = Serial.read();
    if (b <= maxAngle)
      setServo(pin, b);
    else if (b <= maxAngle + MAX_SERVOS)
      pin = b - (maxAngle + 1);
    else switch (b) {
        #ifdef UseEEPROM
          case cmdRecvEEPROM: RecvEEPROM(); break;
          case cmdSendEEPROM: SendEEPROM(); break;
        #endif
        
        case cmdRunGait: 
          StartRunGait(GaitTableAddress(recvByte()), -1, 0, 255, 100, 0, 100);
          Serial.write(cmdAck); 
          break;
        
        case cmdMaxSpeed: 
          i = recvByte();
          Serial.write(cmdAck); 
          for (int iGait = 0; iGait < numGaitRecs; iGait++)
            if (GaitRec[iGait].GaitTable == GaitTableAddress(i))
              GaitRec[iGait].MaxServoSpeed = recvByte(); 
          break;

        case cmdMinTime: 
          i = recvByte();
          Serial.write(cmdAck); 
          for (int iGait = 0; iGait < numGaitRecs; iGait++)
            if (GaitRec[iGait].GaitTable == GaitTableAddress(i))
              GaitRec[iGait].MinApproachTime  = recvByte(); 
          break;

        case cmdBlendRun: 
          i = recvByte(); // gait table A
          i2 = recvByte(); // share of A
          i3 = recvByte(); // gait table B
          i4 = recvByte(); // start pose; may be 255
          i5 = recvByte(); // end pose; may be 255
          StartRunGait(GaitTableAddress(i),GaitTableAddress(i3),i4,i5,100,0,i2);
          Serial.write(cmdAck); 
          break;

        case cmdStopNow: 
          StopNow(GaitTableAddress(recvByte())); 
          Serial.write(cmdAck); 
          break;

        case cmdStopWhen: 
          i = recvByte(); 
          StopWhen(GaitTableAddress(i),recvByte()); 
          Serial.write(cmdAck); 
          break;

        case cmdQueryPose:
          Serial.write(QueryPose(GaitTableAddress(recvByte()))); 
          break;

        case cmdAtTargets:
          Serial.write(AtTargets(GaitTableAddress(recvByte()))); 
          break;
      }
  }
}

//-----------------------------------------------------------
// setup()
//-----------------------------------------------------------
void setup() {
  Serial.begin(57600);
  Serial.println("Quad3");

  for (int sv = 0; sv < MAX_SERVOS; sv++)
    setServo(sv, ServoPos[sv] / cent);
 
  for (int iGait = 0; iGait < numGaitRecs; iGait++)
    GaitRec[iGait].GaitTable = -1; // not in use
}

//-----------------------------------------------------------
// loop()
//   main loop
//-----------------------------------------------------------
void loop() {
  RunGaits();
  recvCmd();
}
