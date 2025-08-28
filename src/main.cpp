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
float startcurrent = 40;
float maxcurrent = 0;
bool but1Up, but2Up;
bool but1Press, but2Press;
bool cancelled;
float cur_pos;
float tarVolt = 0.0;
float realAngleTotal = 0.0;
float sollAngleTotal = 0.0;
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
  Serial.begin(115200);

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

float lerpDead(int32_t x, int32_t d1, int32_t d2){
  if(d1 == d2){
    return 0;
  } else {
    return (float)(x - d1) / (float)(d2 - d1);
  }
}

float getRealDegPosition(){
  return dxl.getPresentPosition(DXL_ID, UNIT_RAW) * DEG_PER_TICK;
}

int32_t DegToTick(float degPos){
  return (int32_t)lroundf(degPos * TICK_PER_DEG);
}

bool reachedGoal(uint8_t dxl_id, int32_t target_tick, int32_t err_tick = 2 ,uint32_t timeout = 8000){
  elapsedMillis polling;
  elapsedMillis t;
  while(t < timeout){
    if(polling >= 1){
      int32_t cur_pos = dxl.getPresentPosition(dxl_id, UNIT_RAW);
      float cur_cur = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
      if(fabsf(cur_cur) > maxcurrent+1){
        realAngleTotal = getRealDegPosition();
        dxl.setGoalPosition(dxl_id,dxl.getPresentPosition(dxl_id,UNIT_RAW), UNIT_RAW);
        return false;
      }

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

bool measureMax(uint8_t dxl_id, int32_t target_tick, int32_t err_tick = 2 ,uint32_t timeout = 8000){
  elapsedMillis polling;
  elapsedMillis t;
  while(t < timeout){
    if(polling >= 1){
      int32_t cur_pos = dxl.getPresentPosition(dxl_id, UNIT_RAW);
      float cur_cur = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
      if(fabsf(cur_cur) > startcurrent){
        dxl.setGoalPosition(dxl_id,dxl.getPresentPosition(dxl_id,UNIT_RAW), UNIT_RAW);
        return false;
      }
      if(maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
        maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
      }
      
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


float deadSollVolt(int32_t tick, float tarVolt, 
                  float totalAngle, float d12, 
                  float d21, float d22, float d31){

  const int32_t d12Tick = DegToTick(d12);
  const int32_t d21Tick = DegToTick(d21);
  const int32_t d22Tick = DegToTick(d22);
  const int32_t d31Tick = DegToTick(d31);
  const float halfVolt = tarVolt / 2;

  if(tick <= d12Tick){return 0;}
  if(tick <= d21Tick){
    float alive1 = lerpDead(tick, d12Tick, d21Tick);
    return halfVolt * alive1;
  }
  if(tick <= d22Tick){return halfVolt;}
  if(tick <= d31Tick){
    float alive2 = lerpDead(tick, d22Tick, d31Tick);
    return halfVolt + halfVolt * alive2;
  }
  return tarVolt;
}

void sim_movement(){
  cancelled = false;
  dxl.ledOn(DXL_ID);
  for(int i = 0; i<4 ;i++){
    dxl.setGoalPosition(DXL_ID, 1900, UNIT_RAW);
    measureMax(DXL_ID, 1900);
    dxl.setGoalPosition(DXL_ID, 2200, UNIT_RAW);
    measureMax(DXL_ID, 2200);
  }
  dxl.torqueOff(DXL_ID);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, (int32_t)maxcurrent);
  dxl.torqueOn(DXL_ID);

  dxl.setGoalPosition(DXL_ID, 0, UNIT_RAW);
  reachedGoal(DXL_ID, 0);
  dxl.setGoalPosition(DXL_ID, 4095, UNIT_RAW);
  reachedGoal(DXL_ID, 4095);
  dxl.setGoalPosition(DXL_ID, 0, UNIT_RAW);
  reachedGoal(DXL_ID, 0);
  for(int i = 0; i < 5; i++){ // UNBEDINGT FIXEN
    dxl.ledOff(1);
    delay(150);
     dxl.ledOn(1);
    delay(150);
  } 

  realAngleTotal = 331.2; // DAS AUCH
  sollAngleTotal = 330;// DAS AUCH
  float posMidSteps = 25;
  float negMidSteps = 25;
  float realMid = realAngleTotal / 2;
  float sollMid = sollAngleTotal / 2;

  float sim_deg [] = {realMid,
     d22, d22+negMidSteps, d22+2*negMidSteps, d22+3*negMidSteps, d31, realAngleTotal,
      d21, d21-posMidSteps, d21-2*posMidSteps, d21-3*posMidSteps, d12, 0};
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
    if (Serial.available()) {
      String stopCommand = Serial.readStringUntil('\n');
      stopCommand.trim();
      if(stopCommand == "STOP"){
        Serial.print("Vorgang wurde abgebrochen");
        cancelled = true;
        break;
      }
    }
    //DEBUG_SERIAL.print(tick);
    dxl.setGoalPosition(DXL_ID, tick, UNIT_RAW);
    if (!reachedGoal(DXL_ID, tick)) {
      // Strom-Trip -> sofort raus
      Serial.println("CANCEL");
      cancelled = true;
      break;
    }

    Serial.print("Soll-Winkel:");
    if(tick == DegToTick(realMid)){
      Serial.print(0);
    } else if(tick == DegToTick(realAngleTotal) || tick == 0){
      Serial.print(tick*DEG_PER_TICK-realMid);
    } else{
      Serial.print(tick*DEG_PER_TICK-sollMid);
    }
    Serial.print(";Soll-Spannung:");
    Serial.print(deadSollVolt(tick, tarVolt, realAngleTotal, 
                              d12, d21, d22, d31));

    Serial.print(";Ist-Spannung:");
    Serial.print(tick/500);
    Serial.print(";Ist-Winkel:");
    Serial.print(dxl.getPresentPosition(DXL_ID, UNIT_RAW)*DEG_PER_TICK - realMid);
    Serial.print(";Linearität:");
    Serial.println((tick/500 - deadSollVolt(tick, tarVolt, realAngleTotal, 
                              d12, d21, d22, d31))/tarVolt);
    //DEBUG_SERIAL.print("Sollwert: ");
    //DEBUG_SERIAL.print(tick*DEG_PER_TICK, 3);
    //DEBUG_SERIAL.print("----Istwert: ");
    //float cur_deg = getRealDegPosition();
    //DEBUG_SERIAL.println(cur_deg, 3);
  }
  dxl.setGoalPosition(DXL_ID, 2047, UNIT_RAW);
  reachedGoal(DXL_ID, 2047);

  dxl.torqueOff(DXL_ID);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, 100);
  dxl.torqueOn(DXL_ID);
  dxl.ledOff(DXL_ID);
}

void loop() {
  if(Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim();

    if(command.startsWith("SETV:")){tarVolt = command.substring(5).toFloat();}
    if(command.startsWith("SETW:")){sollAngleTotal = command.substring(5).toFloat();}
    if(command.startsWith("dead11:")){d11 = command.substring(7).toFloat();}
    if(command.startsWith("dead12:")){d12 = command.substring(7).toFloat();}
    if(command.startsWith("dead21:")){d21 = command.substring(7).toFloat();}
    if(command.startsWith("dead22:")){d22 = command.substring(7).toFloat();}
    if(command.startsWith("dead31:")){d31 = command.substring(7).toFloat();}
    if(command.startsWith("dead32:")){d32 = command.substring(7).toFloat();}

    else if(command == "GO"){
      //but1Press = digitalRead(BUT1);
      //if(but1Up == HIGH && but1Press == LOW && but1Millis > buttonTimer) {
        //DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_RAW));
        sim_movement();
        //DEBUG_SERIAL.print("End Hold: ");
        //DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_RAW));
      //  but1Millis = 0;
      //}
      //but1Up = but1Press;
      if(cancelled == false){
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
      }
    }
  }
}
