#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <elapsedMillis.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = A6;
//const int BUT1 = 6, BUT2 = 7;

const uint8_t  DXL_ID = 1;
const float    DXL_PROTOCOL = 2.0;
const uint32_t DXL_BAUD = 1000000;

const int pollTimer = 1;
const int32_t measureZone1 = 1700, measureZone2 = 2400, servoMid = 2047;
const int slowSpeed = 1, medSpeed = 2, fastSpeed = 3;
//elapsedMillis but1Millis;
//unsigned long buttonTimer = 150;
//bool but1Up, but2Up;
//bool but1Press, but2Press;

float startcurrent = 80;
float slow_maxcurrent = 0;
float med_maxcurrent = 0;
float fast_maxcurrent = 0;

bool cancelled;
int32_t cur_pos;
float tarVolt;
int32_t stoppedTick;
int32_t realTickTotal;
int32_t sollTickTotal;
float sollDegTotal;
float d11Deg, d12Deg, d21Deg, d22Deg, d31Deg, d32Deg;
int32_t d11Tick, d12Tick, d21Tick, d22Tick, d31Tick, d32Tick;
float istStartVolt, istEndVolt;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

constexpr float DEG_PER_TICK = 360.0f / 4096.0f;
constexpr float TICK_PER_DEG = 4096.0f / 360.0f;

void setup() {
  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(BUT1, INPUT);
  //pinMode(BUT2, INPUT);
  Serial.begin(115200);

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
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

int32_t getRealTickPosition(){
  return dxl.getPresentPosition(DXL_ID, UNIT_RAW);
}

int32_t DegToTick(float degPos){
  return (int32_t)lroundf(degPos * TICK_PER_DEG);
}

float TickToDeg(int32_t tickPos){
  return (float)lroundf(tickPos * DEG_PER_TICK);
}

float setDynaSpeed(int speed){
  switch(speed){
    case 1:
      dxl.torqueOff(DXL_ID);
      dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 15);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 8);
      dxl.torqueOn(DXL_ID);
    case 2:
      dxl.torqueOff(DXL_ID);
      dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 40);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 15);
      dxl.torqueOn(DXL_ID);
    case 3:
      dxl.torqueOff(DXL_ID);
      dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 100);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 35);
      dxl.torqueOn(DXL_ID);
  }
  
}

bool reachedGoal(uint8_t dxl_id, int32_t target_tick, int measuring = 0, int32_t err_tick = 1 ,uint32_t timeout = 20000){
  elapsedMillis polling;
  elapsedMillis t;
  elapsedMillis LEDMillis;
  while(t < timeout && cancelled == false){
    if(polling >= pollTimer){
      cur_pos = getRealTickPosition();
      float cur_cur = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);

      if (Serial.available()) {
        String stopCommand = Serial.readStringUntil('\n');
        stopCommand.trim();
        if(stopCommand == "STOP"){
          Serial.print("Vorgang wurde abgebrochen");
          cancelled = true;
          break;
        }
      }
      
      switch(measuring){
        case 0:
        if(fabsf(cur_cur) > med_maxcurrent+3){
          stoppedTick = getRealTickPosition();
          for (int i=0;i<3;i++) dxl.setGoalPosition(DXL_ID, getRealTickPosition(), UNIT_RAW);
          return false;
          break;
        }
        
        case 1:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          return false;
        }
        if(slow_maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
          slow_maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
        }

        case 2:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          return false;
        }
        if(med_maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
          med_maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
        }

        case 3:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          return false;
        }
        if(fast_maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
          fast_maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
        }
      }

      if ((fabsf(cur_pos - target_tick) <= err_tick) && cur_pos - target_tick < 0) {
        for(int i = 0; i<5 ; i++){
          dxl.setGoalPosition(dxl_id, getRealTickPosition() + 1, UNIT_RAW);
        } 
        return true;
      }
      else if ((fabsf(cur_pos - target_tick) <= err_tick) && cur_pos - target_tick > 0) {
        for(int i = 0; i<5 ; i++){
          dxl.setGoalPosition(dxl_id, getRealTickPosition() - 1, UNIT_RAW);
        } 
        return true;
      }
      else if (cur_pos == target_tick){
        return true;
      }
      polling = 0;
    }
  } 
  dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
  return false;
}

float deadSollVoltDeg(float deg, float tarVolt,
                      float d12Deg, float d21Deg, float d22Deg, float d31Deg) {
  const float half = tarVolt * 0.5f;
  if (deg <= d12Deg) return 0.0f;
  if (deg <= d21Deg) return half * (deg - d12Deg) / (d21Deg - d12Deg);
  if (deg <= d22Deg) return half;
  if (deg <= d31Deg) return half + half * (deg - d22Deg) / (d31Deg - d22Deg);
  return tarVolt;
}

void correction_movement(){
  // todo
}

void sim_movement(){
  cancelled = false;
  med_maxcurrent = 0;
  
  dxl.ledOn(DXL_ID);
  
  // --------------- MAX. STROM/TORQUE MESSEN
  for(int i = 1; i<=3; i++){
    setDynaSpeed(i);
    for(int j = 0; j <= 1 ;j++){
      dxl.setGoalPosition(DXL_ID, measureZone1, UNIT_RAW);
      reachedGoal(DXL_ID, measureZone1, i);
      dxl.setGoalPosition(DXL_ID, measureZone2, UNIT_RAW);
      reachedGoal(DXL_ID, measureZone2, i);
    }
  }

  dxl.torqueOff(DXL_ID);
  dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 40);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 15);
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, (int32_t)med_maxcurrent+1);
  dxl.torqueOn(DXL_ID);

  dxl.setGoalPosition(DXL_ID, -servoMid, UNIT_RAW);
  reachedGoal(DXL_ID, -servoMid);
  int32_t startTick = stoppedTick;
  Serial.println(":SENS:VOLT:DC:REF:STAT OFF");
  delay(50);
  Serial.println(":SENS:VOLT:DC:REF:ACQ");
  delay(50);
  Serial.println(":SENS:VOLT:DC:REF:STAT ON");
  delay(50);
  Serial.println("VOLTR");
  delay(50);
  for(;;){
    String VCommand = Serial.readStringUntil('\n');
    VCommand.trim();
    if(VCommand.startsWith("ISTV:")){
      istStartVolt = VCommand.substring(5).toFloat();
      break;
    }
  }


  for(int i = 0; i < 5; i++){ // UNBEDINGT FIXEN
    dxl.ledOff(1);
    delay(100);
     dxl.ledOn(1);
    delay(100);
  } 

  dxl.setGoalPosition(DXL_ID, 3*servoMid, UNIT_RAW);
  reachedGoal(DXL_ID, 3*servoMid);
  int32_t endTick = stoppedTick;
  realTickTotal = endTick - startTick;
  Serial.println("VOLTR");
  delay(50);
  for(;;){
    String VCommand = Serial.readStringUntil('\n');
    VCommand.trim();
    if(VCommand.startsWith("ISTV:")){
      istEndVolt = VCommand.substring(5).toFloat();
      break;
    }
  }
  
  for(int i = 0; i < 5; i++){ // UNBEDINGT FIXEN
    dxl.ledOff(1);
    delay(100);
    dxl.ledOn(1);
    delay(100);
  } 
  // 1° = 11.375 ticks
  // 1 Tick = 0.08791208791 °
  float realDegTotal = TickToDeg(realTickTotal);
  // realTickTotal = DegToTick(realDegTotal); // ca. 331.2°
  sollTickTotal = DegToTick(sollDegTotal); // ca. 330°

  float midDegs = 25;
  float realMidDeg = realDegTotal/2;
  float sollMidDeg = sollDegTotal/2;
  int32_t midSteps = DegToTick(25); // ca. 284 Ticks
  int32_t realMid = DegToTick(realDegTotal/2);

  d11Tick = DegToTick(d11Deg);
  d12Tick = DegToTick(d12Deg);
  d21Tick = DegToTick(d21Deg);
  d22Tick = DegToTick(d22Deg);
  d31Tick = DegToTick(d31Deg);
  d32Tick = DegToTick(d32Deg);

  float soll_Deg[] = {realMidDeg,
     d22Deg, d22Deg+midDegs, d22Deg+2*midDegs, d22Deg+3*midDegs, d31Deg, realDegTotal,
      d21Deg, d21Deg-midDegs, d21Deg-2*midDegs, d21Deg-3*midDegs, d12Deg, 0};
  
  int32_t mercyTick = 10;
  int32_t mercyStart = mercyTick;
  int32_t mercyEnd = realTickTotal - mercyTick;
  int32_t sim_tick [] = {realMid,
     d22Tick, d22Tick+midSteps, d22Tick+2*midSteps, d22Tick+3*midSteps, d31Tick, mercyEnd,
      d21Tick, d21Tick-midSteps, d21Tick-2*midSteps, d21Tick-3*midSteps, d12Tick, mercyStart};
  
  int32_t drive_tick[sizeof(sim_tick)/sizeof(sim_tick[0])];
  for(size_t i = 0; i<sizeof(sim_tick)/sizeof(sim_tick[0]); i++){    
    drive_tick[i] = sim_tick[i] + startTick;
  }
  
  float printSollDeg[sizeof(sim_tick)/sizeof(sim_tick[0])];
  float printSollVolt[sizeof(sim_tick)/sizeof(sim_tick[0])];
  float printIstVolt[sizeof(sim_tick)/sizeof(sim_tick[0])];
  float printIstDeg[sizeof(sim_tick)/sizeof(sim_tick[0])];
  float printLinear[sizeof(sim_tick)/sizeof(sim_tick[0])];

  for (size_t i = 0; i<sizeof(sim_tick)/sizeof(sim_tick[0]); i++) {
    int32_t tick = drive_tick[i];
    //DEBUG_SERIAL.print(tick);
    dxl.setGoalPosition(DXL_ID, tick, UNIT_RAW);
    if (!reachedGoal(DXL_ID, tick)) {
      // Strom-Trip -> sofort raus
      Serial.println("CANCEL");
      cancelled = true;
      break;
    } else {
      //Serial.println("VOLTREADY");
    }

    // SOLL-WINKEL
    int32_t relTick = tick - startTick;
    if(relTick == realMid){
      printSollDeg[i] = soll_Deg[i]-realMidDeg;
    } else if(relTick == mercyEnd || relTick == mercyStart){
       printSollDeg[i] = soll_Deg[i]-realMidDeg;
    } else{
       printSollDeg[i] = soll_Deg[i]-sollMidDeg;
    }

    // SOLL-SPANNUNG
    printSollVolt[i] = deadSollVoltDeg(soll_Deg[i], tarVolt, d12Deg, d21Deg, d22Deg, d31Deg);

    // IST-SPANNUNG
    if(relTick == mercyEnd){
      printIstVolt[i] = istEndVolt;
    } else if(relTick == mercyStart) {
      printIstVolt[i] = istStartVolt;
    } else {
      Serial.println("VOLTR");
      delay(50);
      for(;;){
        String VCommand = Serial.readStringUntil('\n');
        VCommand.trim();
        if(VCommand.startsWith("ISTV:")){
          printIstVolt[i] = VCommand.substring(5).toFloat();
          break;
        }
      }
    }
    
    // IST-WINKEL
    if(relTick == realMid){
      printIstDeg[i] = getRealDegPosition() - realMidDeg - TickToDeg(startTick);
    } else if(relTick == mercyEnd){
      printIstDeg[i] = realMidDeg;
    } else if(relTick == mercyStart){
      printIstDeg[i] = -realMidDeg;
    } else{
      printIstDeg[i] = getRealDegPosition() - sollMidDeg - TickToDeg(startTick);
    }

    // LINEARITÄT
    printLinear[i] = (printIstVolt[i] - deadSollVoltDeg(soll_Deg[i], tarVolt, d12Deg, d21Deg, d22Deg, d31Deg))/tarVolt;


    /*
    Serial.print("Soll-Winkel:");
    if(relTick == realMid){
      Serial.print(soll_Deg[0]-realMidDeg);
    } else if(relTick == realTickTotal || relTick == 0){
      Serial.print((relTick - realMid)*DEG_PER_TICK);
    } else{
      Serial.print(soll_Deg[i]-sollMidDeg);
    }
    Serial.print(";Soll-Spannung:");
    Serial.print(deadSollVoltDeg(soll_Deg[i], tarVolt, d12Deg, d21Deg, d22Deg, d31Deg));

    Serial.print(";Ist-Spannung:");
    Serial.print(tick/500);
    Serial.print(";Ist-Winkel:");
     if(relTick == realMid){
      Serial.print(getRealDegPosition() - realMidDeg - startTick*DEG_PER_TICK);
    } else if(relTick == realTickTotal || relTick == 0){
      Serial.print((getRealDegPosition() - realMidDeg - startTick*DEG_PER_TICK));
    } else{
      Serial.print(getRealDegPosition() - sollMidDeg - startTick*DEG_PER_TICK);
    }
    Serial.print(";Linearität:");
    Serial.println((tick/500 - deadSollVoltDeg(soll_Deg[i], tarVolt, d12Deg, d21Deg, d22Deg, d31Deg))/tarVolt);
    //DEBUG_SERIAL.print("Sollwert: ");
    //DEBUG_SERIAL.print(tick*DEG_PER_TICK, 3);
    //DEBUG_SERIAL.print("----Istwert: ");
    //float cur_deg = getRealDegPosition();
    //DEBUG_SERIAL.println(cur_deg, 3);
    
    */
  }
  for (size_t i = 0; i<sizeof(sim_tick)/sizeof(sim_tick[0]); i++) {
    Serial.print("Soll-Winkel:");
    Serial.print(printSollDeg[i]);
    Serial.print(";Soll-Spannung:");
    Serial.print(printSollVolt[i]);
    Serial.print(";Ist-Spannung:");
    Serial.print(printIstVolt[i]);
    Serial.print(";Ist-Winkel:");
    Serial.print(printIstDeg[i]);
    Serial.print(";Linearität:");
    Serial.println(printLinear[i]);
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
    if(command.startsWith("SETW:")){sollDegTotal = command.substring(5).toFloat();}
    if(command.startsWith("dead11:")){d11Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead12:")){d12Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead21:")){d21Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead22:")){d22Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead31:")){d31Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead32:")){d32Deg = command.substring(7).toFloat();}

    else if(command == "GO"){
      sim_movement();
      if(cancelled == false){
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
      }
    }
  }
}
