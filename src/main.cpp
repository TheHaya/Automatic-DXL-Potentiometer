#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <elapsedMillis.h>
#include <array>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = A6;

const uint8_t  DXL_ID = 1;
const float    DXL_PROTOCOL = 2.0;
const uint32_t DXL_BAUD = 1000000;

const int pollTimer = 1;
const int32_t measureZone1 = 1850, measureZone2 = 2250, servoMid = 2047;
const int slowSpeed = 1, medSpeed = 2, fastSpeed = 3;

const size_t printArraySize = 13;
float printIstDeg[printArraySize];
float printSollDeg[printArraySize];
float printSollVolt[printArraySize];
float printIstVolt[printArraySize];
float printRealDiffMid[printArraySize];
float printSollVoltReal[printArraySize];
float printLinearReal[printArraySize];
//float printSollVoltReal[printArraySize];
//float printLinear[printArraySize];
bool emptyCells[printArraySize];

bool error_lin;
bool error_mech;
bool error_midDead;
bool error_lin_index[printArraySize];

float startcurrent = 150;
float slow_maxcurrent;
float med_maxcurrent;
float fast_maxcurrent;
float tarVolt = 10;

bool cancelled;
int32_t cur_pos;
int32_t stoppedTick;
int32_t realTickTotal, sollTickTotal, startTick;
float sollDegTotal;
float d11Deg, d12Deg, d21Deg, d22Deg, d31Deg, d32Deg;
int32_t d11Tick, d12Tick, d21Tick, d22Tick, d31Tick, d32Tick;
int32_t userGoto;
float istStartVolt, istEndVolt, istMidVolt;
float ccwLinks, ccwRechts, cwLinks, cwRechts, aktivCCW, aktivCW, gesAktiv;
float realMidDeg;
float lin_min, lin_max;

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
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, 150);
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
  return (float)(tickPos * DEG_PER_TICK);
}

void setDynaSpeed(int speed){
  switch(speed){
    case 1:
      dxl.torqueOff(DXL_ID);
      dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 15);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 8);
      dxl.torqueOn(DXL_ID);
      break;
    case 2:
      dxl.torqueOff(DXL_ID);
      dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 40);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 15);
      dxl.torqueOn(DXL_ID);
      break;
    case 3:
      dxl.torqueOff(DXL_ID);
      dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_ID, 100);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 35);
      dxl.torqueOn(DXL_ID);
      break;

    default:
      break;
  }
}

bool reachedGoal(uint8_t dxl_id, int32_t target_tick, int measuring = 0, int32_t err_tick = 1 ,uint32_t timeout = 20000){
  elapsedMillis polling;
  elapsedMillis t;

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
        if(fabsf(cur_cur) > med_maxcurrent+6){
          stoppedTick = getRealTickPosition();
          for (int i=0;i<3;i++) dxl.setGoalPosition(DXL_ID, getRealTickPosition(), UNIT_RAW);
          Serial.println("Strom MED Fehler");
          return false;
        }
        break;
        
        case 1:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          Serial.println("Kalibrierung SLOW Fehler");
          return false;
        }
        if(slow_maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
          slow_maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
        }
        break;

        case 2:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          Serial.println("Kalibrierung MEDIUM Fehler");
          return false;
        }
        if(med_maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
          med_maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
        }
        break;

        case 3:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          Serial.println("Kalibrierung FAST Fehler");
          return false;
        }
        if(fast_maxcurrent < dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE)){
          fast_maxcurrent = dxl.getPresentCurrent(dxl_id, UNIT_MILLI_AMPERE);
        }
        break;

        case 4:
        if(fabsf(cur_cur) >= startcurrent){
          dxl.setGoalPosition(dxl_id, getRealTickPosition(), UNIT_RAW);
          return false;
        }
        break;

        case 5:
        if(fabsf(cur_cur) > slow_maxcurrent+2){
          stoppedTick = getRealTickPosition();
          for (int i=0;i<3;i++) dxl.setGoalPosition(DXL_ID, getRealTickPosition(), UNIT_RAW);
          Serial.println("Strom SLOW Fehler");
          return false;
        }
        break;

        case 6:
        if(fabsf(cur_cur) > fast_maxcurrent+10){
          stoppedTick = getRealTickPosition();
          for (int i=0;i<3;i++) dxl.setGoalPosition(DXL_ID, getRealTickPosition(), UNIT_RAW);
          Serial.println("Strom FAST Fehler");
          return false;
        }
        break;
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

float corr_measure(float currentVolt){
  cancelled = false;
  Serial.println("VOLTR");
  delay(50);
  if(cancelled == false){
    for(;;){
    String VCommand = Serial.readStringUntil('\n');
    VCommand.trim();
      if(VCommand.startsWith("ISTV:")){
        currentVolt = VCommand.substring(5).toFloat();
        break;
      }
    }
  }
  return currentVolt;
}

// deadDirection 0 -> deadzone links von position //////--- ;;; 1 -> rechts von position ---//////
float correction_movement(float &currentVolt, float goalVolt, int deadDirection = 0, int timeout = 8000){
const float v_tol = 0.001f;        
const int32_t tick_tol = 1;
int32_t low, high;
currentVolt = corr_measure(currentVolt);

while (!cancelled && fabsf(currentVolt - goalVolt) <= v_tol) {
  int32_t t = 0;
  if(deadDirection == 0){ // dead links
    t = getRealTickPosition() + 20;
  } else if(deadDirection == 1){ // dead rechts
    t = getRealTickPosition() - 20;
  }
  dxl.setGoalPosition(DXL_ID, t, UNIT_RAW);
  reachedGoal(DXL_ID, t, 6);
  currentVolt = corr_measure(currentVolt);
}

high = getRealTickPosition();

while (!cancelled && fabsf(currentVolt - goalVolt) > v_tol) {
  int32_t t = 0;
  if(deadDirection == 0){
    t = getRealTickPosition() - 10;
  } else if(deadDirection == 1){
    t = getRealTickPosition() + 10;
  }
  dxl.setGoalPosition(DXL_ID, t, UNIT_RAW);
  reachedGoal(DXL_ID, t, 6);
  currentVolt = corr_measure(currentVolt);
}

low = getRealTickPosition();

while (!cancelled && abs(high - low) > tick_tol) {
  int32_t mid = (high + low) / 2;
  dxl.setGoalPosition(DXL_ID, mid, UNIT_RAW);
  reachedGoal(DXL_ID, mid, 6);

  currentVolt = corr_measure(currentVolt);
  if (fabsf(currentVolt - goalVolt) > v_tol) {
    high = mid;
  } else {
    low = mid;
  }
}
float edgeDeg = 0;
  if(deadDirection == 0){
    edgeDeg = TickToDeg(low);
  } else if(deadDirection == 1){
    edgeDeg = TickToDeg(high);
  }
return edgeDeg;
}

// ------------------ Random GUI buttons für debugging
void goLeft(){
  cancelled = false;
  setDynaSpeed(fastSpeed);
  dxl.setGoalPosition(DXL_ID, getRealTickPosition()-10, UNIT_RAW);
  reachedGoal(DXL_ID, getRealTickPosition()-10, 4);
}

void goRight(){
  cancelled = false;
  setDynaSpeed(fastSpeed);
  dxl.setGoalPosition(DXL_ID, getRealTickPosition()+5, UNIT_RAW);
  reachedGoal(DXL_ID, getRealTickPosition()+5, 4);
}

void goTo(){
  cancelled = false;
  setDynaSpeed(fastSpeed);
  dxl.setGoalPosition(DXL_ID, userGoto, UNIT_RAW);
  reachedGoal(DXL_ID, userGoto, 4);
}

void zero_movement(){
  cancelled = false;
  setDynaSpeed(fastSpeed);
  dxl.setGoalPosition(DXL_ID, 2047, UNIT_RAW);
  reachedGoal(DXL_ID, 2047, 4);
}

// ----------------- MESSVORGANG INITIALISIEREN
void calc_init(){
  error_lin = false;
  error_mech = false;
  error_midDead = false;
  for (size_t i = 0; i < printArraySize; ++i) {
    error_lin_index[i] = false;
  }
  lin_max = 0;
  lin_min = 0;
  cancelled = false;
  slow_maxcurrent = 0;
  med_maxcurrent = 0;
  fast_maxcurrent = 0;
}

// ----------------- MESSVORGANG BEWEGUNG UND AUFNAHME
void sim_movement(){
  dxl.ledOn(DXL_ID);
  
  // --------------- MAX. STROM/TORQUE MESSEN
  setDynaSpeed(fastSpeed);
  dxl.setGoalPosition(DXL_ID, measureZone1, UNIT_RAW);
  reachedGoal(DXL_ID, measureZone1, 4);

  for(int i = 1; i<=3; i++){
    setDynaSpeed(i);
    dxl.setGoalPosition(DXL_ID, measureZone2, UNIT_RAW);
    reachedGoal(DXL_ID, measureZone2, i);
    dxl.setGoalPosition(DXL_ID, measureZone1, UNIT_RAW);
    reachedGoal(DXL_ID, measureZone1, i);
  }

  // --------------- MECHANISCHE ENDEN ERMITTELN
  setDynaSpeed(medSpeed);
  dxl.setGoalPosition(DXL_ID, DegToTick(35), UNIT_RAW);
  reachedGoal(DXL_ID, DegToTick(35));
  setDynaSpeed(slowSpeed);
  dxl.setGoalPosition(DXL_ID, -servoMid, UNIT_RAW);
  reachedGoal(DXL_ID, -servoMid, 5);
  startTick = stoppedTick;
  istStartVolt = corr_measure(istStartVolt);

  for(int i = 0; i < 5; i++){ // UNBEDINGT FIXEN (maybe)
    dxl.ledOff(1);
    delay(100);
     dxl.ledOn(1);
    delay(100);
  } 

  setDynaSpeed(fastSpeed);
  dxl.setGoalPosition(DXL_ID, DegToTick(325), UNIT_RAW);
  reachedGoal(DXL_ID, DegToTick(325), 6);
  setDynaSpeed(slowSpeed);
  dxl.setGoalPosition(DXL_ID, 3*servoMid, UNIT_RAW);
  reachedGoal(DXL_ID, 3*servoMid, 5);
  int32_t endTick = stoppedTick;
  realTickTotal = endTick - startTick;
  istEndVolt = corr_measure(istEndVolt);
  
  for(int i = 0; i < 5; i++){ // UNBEDINGT FIXEN (maybe)
    dxl.ledOff(1);
    delay(100);
    dxl.ledOn(1);
    delay(100);
  } 

  
  // --------------- ALLE PUNKTE DAZWISCHEN ERMITTELN
  // 1° = 11.375 ticks
  // 1 Tick = 0.08791208791 °
  float realDegTotal = TickToDeg(realTickTotal);
  if(realDegTotal < 298.0 || realDegTotal > 332.0) error_mech = true;
  // realTickTotal = DegToTick(realDegTotal); // ca. 331.2°
  sollTickTotal = DegToTick(sollDegTotal); // ca. 330°

  float midDegs = 25;
  realMidDeg = realDegTotal/2;
  float sollMidDeg = sollDegTotal/2;
  int32_t midSteps = DegToTick(25); // ca. 284 Ticks
  int32_t realMid = realTickTotal/2;
  int32_t offsetVonSoll = (realTickTotal-sollTickTotal) / 2;
  
  for(int i = 0; i < 5; i++){ // UNBEDINGT FIXEN (maybe)
    dxl.ledOff(1);
    delay(100);
    dxl.ledOn(1);
    delay(100);
  } 

  d11Tick = DegToTick(d11Deg);
  d12Tick = DegToTick(d12Deg);
  d21Tick = DegToTick(d21Deg);
  d22Tick = DegToTick(d22Deg);
  d31Tick = DegToTick(d31Deg);
  d32Tick = DegToTick(d32Deg);

  float soll_Deg[] = {realMidDeg,
     d22Deg, d22Deg+midDegs, d22Deg+2*midDegs, d22Deg+3*midDegs, d31Deg, realDegTotal,
      d21Deg, d21Deg-midDegs, d21Deg-2*midDegs, d21Deg-3*midDegs, d12Deg, 0};
  
  int32_t mercyTick = 10; // 10 Ticks vor jeweiligem Ende
  int32_t mercyStart = mercyTick;
  int32_t mercyEnd = realTickTotal - mercyTick;

  int32_t sim_tick [] = {realMid,
     d22Tick, d22Tick+midSteps, d22Tick+2*midSteps, d22Tick+3*midSteps, d31Tick, mercyEnd,
      d21Tick, d21Tick-midSteps, d21Tick-2*midSteps, d21Tick-3*midSteps, d12Tick, mercyStart};
  
  int32_t drive_tick[printArraySize];
  for(size_t i = 0; i<printArraySize; i++){    
    if(sim_tick[i] == realMid || sim_tick[i] == mercyEnd || sim_tick[i] == mercyStart){
      drive_tick[i] = sim_tick[i] + startTick;
    } else {
      drive_tick[i] = sim_tick[i] + startTick + offsetVonSoll;
    }
  }

  setDynaSpeed(fastSpeed);
  for (size_t i = 0; i<printArraySize; i++) {
    int32_t tick = drive_tick[i];
    //DEBUG_SERIAL.print(tick);
    dxl.setGoalPosition(DXL_ID, tick, UNIT_RAW);
    if (!reachedGoal(DXL_ID, tick, 6)) {
      // Strom-Trip -> sofort raus
      Serial.println("CANCEL");
      cancelled = true;
      break;
    }
    
    // LEERE ZELLEN
    int32_t relTick = tick - startTick;
    emptyCells[i] = false;
    if(relTick == mercyStart || relTick == mercyEnd ||
       relTick == d21Tick + offsetVonSoll || relTick == d22Tick + offsetVonSoll){
        emptyCells[i] = true;
       }

    // SOLL-WINKEL
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
      printIstVolt[i] = istEndVolt - istStartVolt;
    } else if(relTick == mercyStart) {
      printIstVolt[i] = 0;
    } else {
      printIstVolt[i] = corr_measure(printIstVolt[i]);
      if(relTick == realMid){
        istMidVolt = printIstVolt[i];
      }
    }
    
    // IST-WINKEL
    if(relTick == realMid){
      printIstDeg[i] = getRealDegPosition() - realMidDeg - TickToDeg(startTick);
    } else if(relTick == mercyEnd){
      printIstDeg[i] = realMidDeg;
    } else if(relTick == mercyStart){
      printIstDeg[i] = -realMidDeg;
    } else if(relTick == d12Tick + offsetVonSoll){
      printIstDeg[i] = correction_movement(printIstVolt[i], istStartVolt, 0) - TickToDeg(startTick) - realMidDeg;
      ccwLinks = printIstDeg[i];
    } else if(relTick == d21Tick + offsetVonSoll){
      printIstDeg[i] = correction_movement(printIstVolt[i], istMidVolt, 1) - TickToDeg(startTick) - realMidDeg;
      ccwRechts = printIstDeg[i];
    } else if(relTick == d22Tick + offsetVonSoll){
      printIstDeg[i] = correction_movement(printIstVolt[i], istMidVolt, 0) - TickToDeg(startTick) - realMidDeg;
      cwLinks = printIstDeg[i];
    } else if(relTick == d31Tick + offsetVonSoll){
      printIstDeg[i] = correction_movement(printIstVolt[i], istEndVolt, 1) - TickToDeg(startTick) - realMidDeg;
      cwRechts = printIstDeg[i];
    }
    else{
      printIstDeg[i] = getRealDegPosition() - realMidDeg - TickToDeg(startTick);
    }
    
    // REALER WINKEL ZUR MITTE
    if(printSollDeg[i] == 0){
      printRealDiffMid[i] = 0;
    }
    else if(printIstDeg[i] > 0){
      printRealDiffMid[i] = printIstDeg[i] - cwLinks;
    } else if(printIstDeg[i] < 0){
      printRealDiffMid[i] = printIstDeg[i] - ccwRechts;
    } 
  }
  // --------------- ERROR MITTELANZAPFUNG
  if(fabsf((cwLinks + ccwRechts) - (d21Deg + d22Deg)) > 1.5){
    error_midDead = true;
  }
  // --------------- ÜBERGABE AN PYTHON
  for (size_t i = 0; i<printArraySize; i++) {
    Serial.print("Soll-Winkel:");
    Serial.print(printSollDeg[i],1);
    Serial.print(";Soll-Spannung:");
    Serial.print(printSollVolt[i],2);
    Serial.print(";Ist-Spannung:");
    Serial.print(printIstVolt[i],3);
    Serial.print(";Ist-Winkel:");
    Serial.print(printIstDeg[i],1);
    Serial.print(";DiffMid-Winkel:");
    Serial.println(printRealDiffMid[i],1);
  }

  dxl.setGoalPosition(DXL_ID, 2047, UNIT_RAW);
  reachedGoal(DXL_ID, 2047, 6);
  dxl.ledOff(DXL_ID);
}

// ----------------- LINEARITÄT
void calc_linearity(){
  const float realVoltPerDegree = tarVolt/gesAktiv;
  const float midSollVoltReal = aktivCCW*realVoltPerDegree;

  for(size_t i = 0; i<printArraySize ; i++){
    // LEERE ZELLEN ÜBERSPRINGEN
    if (emptyCells[i] == true) {
      printSollVoltReal[i] = NAN;
      printLinearReal[i] = NAN;
      continue;
    }

    // SOLLSPANNUNG REAL
    printSollVoltReal[i] = printRealDiffMid[i] * realVoltPerDegree + midSollVoltReal;
    
    // LINEARITÄT
    printLinearReal[i] = (printIstVolt[i] - printSollVoltReal[i])/tarVolt;
    if(fabsf(printLinearReal[i]) > 0.005) {
      error_lin = true;
      error_lin_index[i] = true;
    } 

    if(printLinearReal[i] > lin_max) {
      lin_max = printLinearReal[i];
    }
    if(printLinearReal[i] < lin_min) {
      lin_min = printLinearReal[i];
    }
  }

  
  for(size_t i = 0; i<printArraySize ; i++){
    Serial.print("LINEAR;");
    Serial.print("idx:");   
    Serial.print(i);
    Serial.print(";Soll-Spannung Real:");   
    Serial.print(printSollVoltReal[i], 3);
    Serial.print(";Linearität:"); 
    Serial.println(printLinearReal[i], 6);
  }
}

void calc_summary(){
  float totzone = cwLinks - ccwRechts;
  aktivCCW = fabsf(ccwLinks - ccwRechts);
  aktivCW = (cwRechts - cwLinks);
  gesAktiv = aktivCCW + aktivCW;

  Serial.print("SUMMARY;");
  Serial.print("Totzone:");   
  Serial.print(totzone, 1);
  Serial.print(";AktivCW:"); 
  Serial.print(aktivCW, 1);
  Serial.print(";AktivCCW:");
  Serial.print(aktivCCW, 1);
  Serial.print(";AktivSumme:");
  Serial.println(gesAktiv, 1);
}

// ERROR AUSGABE EXCEL
void calc_errors(){
  if (error_lin == true) {
    Serial.print("ERROR_LIN;");
    Serial.print("IDX:");
    bool first = true;
    for (size_t i = 0; i < printArraySize; i++) {
      if (error_lin_index[i]) {
        if (first) {
          Serial.print(i);
          first = false;
        } else {
          Serial.print(",");
          Serial.print(i);
        }
      }
    }
    Serial.print(";LIN_MAX:");
    Serial.print(lin_max, 6);
    Serial.print(";LIN_MIN:");
    Serial.println(lin_min, 6);
  }
}

void loop() {
  if(Serial.available()){
    String command = Serial.readStringUntil('\n');
    command.trim();

    // EINGABE VON PYTHON
    if(command.startsWith("SETV:")){tarVolt = command.substring(5).toFloat();}
    if(command.startsWith("SETW:")){sollDegTotal = command.substring(5).toFloat();}
    if(command.startsWith("dead11:")){d11Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead12:")){d12Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead21:")){d21Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead22:")){d22Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead31:")){d31Deg = command.substring(7).toFloat();}
    if(command.startsWith("dead32:")){d32Deg = command.substring(7).toFloat();}

    if(command.startsWith("goto:")){userGoto = command.substring(5).toFloat();}

    else if(command == "GO"){
      calc_init();
      sim_movement();
      calc_summary();
      calc_linearity();
      calc_errors();
      if(cancelled == false){
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
        cancelled = false;
      }
    }
    else if(command =="ZERO"){
      zero_movement();
      if(cancelled == false){
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
        cancelled = false;
      }
    } // -------------- AB HIER RANDOM DEBUG BUTTONS
    else if(command =="LEFT"){
      goLeft();
      if(cancelled == false){
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
        cancelled = false;
      }
    }
    else if(command =="RIGHT"){
      goRight();
      if(cancelled == false){
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
        cancelled = false;
      }
    }
    else if(command =="POS"){
      if(cancelled == false){
        delay(0.25);
        Serial.println(getRealDegPosition()-15.7-165.7);
        delay(0.25);
        Serial.println(getRealTickPosition());
        delay(0.25);
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
        cancelled = false;
      }
    }
    else if(command =="GOTO"){
      if(cancelled == false){
        goTo();
        Serial.println("READY");
      } else{
        Serial.println("CANCEL");
        cancelled = false;
      }
    }
  }
}
