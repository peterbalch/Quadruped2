const byte Stand[] PROGMEM = {
  1, // num poses
  0xFB,0x1F,0x00,0x00, // servo set
  131, 125, 90, 90, 90, 76, 44, 57, 109, 105, 141, 77};

const byte Grip[] PROGMEM = {
  3, // num poses
  0x04,0x00,0x00,0x00, // servo set
  79, 
  98, 
  142};

const byte WaveLegs[] PROGMEM = {
  9, // num poses
  0xFB,0x1F,0x00,0x00, // servo set
  71, 149, 90, 90, 90, 90, 36, 109, 86, 134, 75, 54, 
  71, 149, 90, 90, 90, 90, 36, 109, 86, 134, 134, 120, 
  71, 149, 90, 90, 90, 90, 36, 109, 86, 134, 75, 54, 
  71, 149, 90, 90, 90, 90, 36, 109, 136, 53, 75, 54, 
  71, 149, 90, 90, 90, 90, 36, 109, 86, 134, 75, 54, 
  133, 88, 90, 90, 90, 90, 36, 109, 86, 134, 75, 54, 
  71, 149, 90, 90, 90, 90, 36, 109, 86, 134, 75, 54, 
  71, 149, 90, 90, 90, 90, 84, 41, 86, 134, 75, 54, 
  71, 149, 90, 90, 90, 90, 36, 109, 86, 134, 75, 54};

const byte Wave2[] PROGMEM = {
  9, // num poses
  0xFB,0x1F,0x00,0x00, // servo set
  71, 149, 50, 130, 129, 50, 36, 109, 86, 134, 75, 54, 
  71, 149, 50, 130, 129, 50, 36, 109, 86, 134, 134, 120, 
  71, 149, 50, 130, 129, 50, 36, 109, 86, 134, 75, 54, 
  71, 149, 50, 130, 129, 50, 36, 109, 136, 53, 75, 54, 
  71, 149, 50, 130, 129, 50, 36, 109, 86, 134, 75, 54, 
  133, 88, 50, 130, 129, 50, 36, 109, 86, 134, 75, 54, 
  71, 149, 50, 130, 129, 50, 36, 109, 86, 134, 75, 54, 
  71, 149, 50, 130, 129, 50, 84, 41, 86, 134, 75, 54, 
  71, 149, 50, 130, 129, 50, 36, 109, 86, 134, 75, 54};

const byte Kneel[] PROGMEM = {
  1, // num poses
  0xFB,0x1F,0x00,0x00, // servo set
  63, 109, 90, 90, 90, 55, 86, 124, 135, 108, 157, 62};

const byte WaveBack[] PROGMEM = {
  4, // num poses
  0x18,0x1E,0x00,0x00, // servo set
  90, 90, 78, 122, 85, 57, 
  90, 90, 145, 78, 150, 94, 
  90, 90, 78, 122, 85, 57, 
  90, 90, 145, 78, 150, 94};

const byte WaveFront[] PROGMEM = {
  4, // num poses
  0xE3,0x01,0x00,0x00, // servo set
  78, 148, 90, 76, 35, 91, 
  78, 148, 90, 76, 86, 47, 
  78, 148, 90, 76, 35, 91, 
  126, 108, 90, 76, 35, 91};

const byte Still[] PROGMEM = {
  4, // num poses
  0xFB,0x1F,0x00,0x00, // servo set
  78, 148, 90, 90, 90, 76, 35, 91, 78, 122, 85, 57, 
  78, 148, 90, 90, 90, 76, 35, 91, 78, 122, 85, 57, 
  78, 148, 90, 90, 90, 76, 35, 91, 78, 122, 85, 57, 
  78, 148, 90, 90, 90, 76, 35, 91, 78, 122, 85, 57};

int GaitTableAddr[] = {
  Stand,
  Grip,
  WaveLegs,
  Wave2,
  Kneel,
  WaveBack,
  WaveFront,
  Still};
