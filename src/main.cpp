#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <elapsedMillis.h>

// ===== Hardware (MKR Zero + DYNAMIXEL MKR Shield) =====
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = A6;
const int BUT1 = 6, BUT2 = 7;

// ===== DYNAMIXEL setup =====
const uint8_t  DXL_ID = 1;             // <-- deine Servo-ID
const float    DXL_PROTOCOL = 2.0;
const uint32_t DXL_BAUD = 1000000;
elapsedMillis but1Millis;
unsigned long buttonTimer = 150;
bool but1Up, but2Up;
bool but1Press, but2Press;
float cur_pos;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

constexpr float DEG_PER_TICK = 360.0f / 4096.0f;
static inline float snapNearestDeg(float deg) {
  return roundf(deg / DEG_PER_TICK) * DEG_PER_TICK;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUT1, INPUT);
  pinMode(BUT2, INPUT);
  DEBUG_SERIAL.begin(115200);

  // Bus starten
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.writeControlTableItem(DRIVE_MODE,    DXL_ID, 1);
  dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, 0); // kein Offset
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, 100);
  dxl.torqueOn(DXL_ID);
}

bool reachedGoal(uint8_t dxl_id, float target_deg, float err_deg = 0.09f, uint32_t timeout = 8000){
  elapsedMillis polling;
  elapsedMillis t;
  while(t < timeout){
    if(polling >= 10){
      float cur_pos = dxl.getPresentPosition(dxl_id, UNIT_DEGREE);
      if (fabsf(cur_pos - target_deg) <= err_deg) {
        return true;
      }
      polling = 0;
    }
  } 
  return false;
}
  

void sim_movement(){
  static const float sim_target [] = {0, 330, 165, 190, 215, 240, 265, 290, 165, 140, 115, 90, 65, 40, 0};
    for (float deg : sim_target) {
      float cmd = snapNearestDeg(deg);                             // nächster darstellbarer Winkel
      dxl.setGoalPosition(DXL_ID, cmd, UNIT_DEGREE);
      reachedGoal(DXL_ID, cmd, 0.09f, 3000);   
      DEBUG_SERIAL.print("Sollwert: ");
      DEBUG_SERIAL.print(deg);
      DEBUG_SERIAL.print("----Istwert: ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE), 3);
  }
  dxl.ledOff(DXL_ID);
}

void loop() {
  but1Press = digitalRead(BUT1);
  if(but1Up == HIGH && but1Press == LOW && but1Millis > buttonTimer) {
    sim_movement();
    but1Millis = 0;
  }
  but1Up = but1Press;
}
