#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <elapsedMillis.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = A6;
const int BUT1 = 6, BUT2 = 7;

const uint8_t  DXL_ID = 1;
const float    DXL_PROTOCOL = 2.0;
const uint32_t DXL_BAUD = 1000000;
elapsedMillis but1Millis;
unsigned long buttonTimer = 150;
bool but1Up, but2Up;
bool but1Press, but2Press;
float cur_pos;
float tarVolt = 0.0;
float totalAngle = 0.0;
float d11;
float d12;
float d21;
float d22;
float d31;
float d32;

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
  dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 100);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 40);

  dxl.torqueOn(DXL_ID);
}

float getRealDegPosition(){
  return dxl.getPresentPosition(DXL_ID, UNIT_RAW) * DEG_PER_TICK;
}

bool reachedGoal(uint8_t dxl_id, int32_t target_tick, int32_t err_tick = 5 ,uint32_t timeout = 10000){
  elapsedMillis polling;
  elapsedMillis t;
  while(t < timeout){
    if(polling >= 5){
      int32_t cur_pos = dxl.getPresentPosition(dxl_id, UNIT_RAW);
      float cur_cur = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
      if(fabsf(cur_cur) > 61){
        DEBUG_SERIAL.print("Before Hold: ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(dxl_id, UNIT_RAW));
        dxl.setGoalPosition(dxl_id,dxl.getPresentPosition(dxl_id,UNIT_RAW), UNIT_RAW);
        DEBUG_SERIAL.print("After Hold: ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(dxl_id, UNIT_RAW));
        return false;
      }
      //DEBUG_SERIAL.println(dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE));
      if ((fabsf(cur_pos - target_tick) <= err_tick) && cur_pos - target_tick < 0) {
        for(int i = 0; i<5 ; i++){
          dxl.setGoalPosition(dxl_id, dxl.getPresentPosition(dxl_id,UNIT_RAW) + 1, UNIT_RAW);
        } 
        return true;
      }
      else if ((fabsf(cur_pos - target_tick) <= err_tick) && cur_pos - target_tick > 0) {
        for(int i = 0; i<5 ; i++){
          dxl.setGoalPosition(dxl_id, dxl.getPresentPosition(dxl_id,UNIT_RAW) - 1, UNIT_RAW);
        } 
        return true;
      }
      else if (cur_pos == target_tick){
        return true;
      }
      polling = 0;
    }
  } 
  return false;
}
  
  
void sim_movement(){
  dxl.ledOn(DXL_ID);

  dxl.setGoalPosition(DXL_ID, 0, UNIT_RAW);
  reachedGoal(DXL_ID, 0);
  dxl.setGoalPosition(DXL_ID, 4095, UNIT_RAW);
  if(reachedGoal(DXL_ID, 4095) == false){
    totalAngle = getRealDegPosition();
  }
  
  float posMidSteps = 25;
  float negMidSteps = 25;
  float absMid = totalAngle / 2;
  float sim_deg [] = {0, totalAngle, absMid,
     d21, absMid-2*negMidSteps, absMid-3*negMidSteps, absMid-4*negMidSteps, d12,
      d22, absMid+2*posMidSteps, absMid+3*posMidSteps, absMid+4*posMidSteps, d31, 0};
  int32_t sim_tick[sizeof(sim_deg)/sizeof(sim_deg[0])];
  int32_t roundTick;
  for(size_t i = 0; i<sizeof(sim_deg)/sizeof(sim_deg[0]); i++){
    roundTick = (int32_t)lroundf(sim_deg[i] * TICK_PER_DEG);
    if(roundTick < 0){
      roundTick = 0;
    }
    else if(roundTick > 4095){
      roundTick = 4095;
    }
    sim_tick[i] = roundTick;
  }
  for (int32_t tick : sim_tick) {
    DEBUG_SERIAL.print(tick);
    dxl.setGoalPosition(DXL_ID, tick, UNIT_RAW);
    if(reachedGoal(DXL_ID, tick) == false) {
      DEBUG_SERIAL.print("End Hold: ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_RAW));
      break;
    }
    DEBUG_SERIAL.print("Sollwert: ");
    DEBUG_SERIAL.print(tick*DEG_PER_TICK, 3);
    DEBUG_SERIAL.print("----Istwert: ");
    float cur_deg = getRealDegPosition();
    DEBUG_SERIAL.println(cur_deg, 3);
  }
  dxl.ledOff(DXL_ID);
}

void loop() {
  if(Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim();
    bool cancelled = false;

    if(command.startsWith("SETV:")){tarVolt = command.substring(5).toFloat();}
    if(command.startsWith("SETW:")){totalAngle = command.substring(5).toFloat();}
    if(command.startsWith("dead11:")){d11 = command.substring(5).toFloat();}
    if(command.startsWith("dead12:")){d12 = command.substring(5).toFloat();}
    if(command.startsWith("dead21:")){d21 = command.substring(5).toFloat();}
    if(command.startsWith("dead22:")){d22 = command.substring(5).toFloat();}
    if(command.startsWith("dead31:")){d31 = command.substring(5).toFloat();}
    if(command.startsWith("dead32:")){d32 = command.substring(5).toFloat();}

    else if(command == "GO"){
      //but1Press = digitalRead(BUT1);
      //if(but1Up == HIGH && but1Press == LOW && but1Millis > buttonTimer) {
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_RAW));
        sim_movement();
        DEBUG_SERIAL.print("End Hold: ");
          DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_RAW));
      //  but1Millis = 0;
      //}
      //but1Up = but1Press;
    }
  }
}
