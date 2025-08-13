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
constexpr float TICK_PER_DEG = 4096.0f / 360.0f;

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
  dxl.writeControlTableItem(DRIVE_MODE,    DXL_ID, 1);  // Drive mode reverse bei 1 bedeutet cw ist + und ccw ist -
  dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, 0); // kein Offset
  dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 40);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 15);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, 100);
  dxl.torqueOn(DXL_ID);
  
}

float getRealPosition(){
  return dxl.getPresentPosition(DXL_ID, UNIT_RAW) * DEG_PER_TICK;
}

bool reachedGoal(uint8_t dxl_id, float target_deg, float err_deg, uint32_t timeout){
  elapsedMillis polling;
  elapsedMillis t;
  int cur_counter = 0;
  while(t < timeout){
    if(polling >= 5){
      float cur_pos = dxl.getPresentPosition(dxl_id, UNIT_RAW);
      float cur_cur = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
      if(fabsf(cur_cur) > 30){
        dxl.setGoalPosition(dxl_id,dxl.getPresentPosition(dxl_id,UNIT_RAW), UNIT_RAW);
        DEBUG_SERIAL.println(dxl.getPresentPosition(dxl_id, UNIT_RAW));
        return false;
      }
      //DEBUG_SERIAL.println(dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE));
      if (fabsf(cur_pos - target_deg) <= err_deg) {
        return true;
      }
      polling = 0;
    }
  } 
  return false;
}
  
void sim_movement(){
  float sim_deg [] = {0, 330, 0, 165, 0, 330, 0};
  int32_t sim_tick[sizeof(sim_deg)/sizeof(sim_deg[0])];
  int32_t roundTick;
  for(int i = 0; i<sizeof(sim_deg)/sizeof(sim_deg[0]); i++){
    roundTick = (int32_t)lroundf(sim_deg[i] * TICK_PER_DEG);
    if(roundTick < 0){
      roundTick = 0;
    }
    else if(roundTick > 4095){
      roundTick = 4095;
    }
    sim_tick[i] = roundTick;
  }
  for (int32_t tick : sim_tick) {                            // nächster darstellbarer Winkel
    dxl.setGoalPosition(DXL_ID, tick, UNIT_RAW);
    if(reachedGoal(DXL_ID, tick, 0.15f, 30000) == false) {break;}
    DEBUG_SERIAL.print("Sollwert: ");
    DEBUG_SERIAL.print(tick*DEG_PER_TICK, 3);
    DEBUG_SERIAL.print("----Istwert: ");
    float cur_deg = getRealPosition();
    DEBUG_SERIAL.println(cur_deg, 3);
  }
  dxl.ledOff(DXL_ID);
}

void loop() {
  but1Press = digitalRead(BUT1);
  if(but1Up == HIGH && but1Press == LOW && but1Millis > buttonTimer) {
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_RAW));
    sim_movement();
    but1Millis = 0;
  }
  but1Up = but1Press;
}
