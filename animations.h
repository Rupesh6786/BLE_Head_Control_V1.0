// Number of columns: 3 servos + 1 column for step duration in ms
#define NUMBER_OF_ACE (numberOfServos + 1)
#define HEAD_ACE      NUMBER_OF_ACE
#define HEAD_SERVOS   numberOfServos

// ========== HEAD ANIMATIONS ==========
// Convention: { neck, eyes, mouth, duration }

// 0) Neutral Position (Straight, centered eyes, mouth closed)
const int headPrg00step = 1;
const int headPrg00[][HEAD_ACE] PROGMEM = {
  { 90, 90, 0, 300 }   // Neutral pose
};

// 1) Scan Motion (Looking left and right to scan environment)
const int headPrg01step = 3;
const int headPrg01[][HEAD_ACE] PROGMEM = {
  { 130, 90, 0, 400 },  // Look Left
  { 50,  90, 0, 500 },  // Look Right
  { 90,  90, 0, 300 }   // Return to Center
};

// 2) Talk / Expression (Jaw movement combined with gaze shifts)
const int headPrg02step = 4;
const int headPrg02[][HEAD_ACE] PROGMEM = {
  { 90, 105, 60, 200 },  // Gaze Left, Mouth Open
  { 90, 75,  0,  200 },  // Gaze Right, Mouth Closed
  { 90, 90,  60, 200 },  // Center, Mouth Open
  { 90, 90,  0,  300 }   // Center, Mouth Closed (Rest)
};