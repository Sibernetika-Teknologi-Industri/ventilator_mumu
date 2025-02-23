// Low-Cost Ventilator
// ----
// Code including serial comm with mega

#include <SoftwareSerial.h>

SoftwareSerial SerialM(11,12); //RX, TX

/////////////////////////////////// BREATHING PART //////////////////////////////////

//-- KONFIGURASI PIN DRIVER =========================================================
// ENA- NC
// ENA+ NC
// DIR- GND
// DIR+ dirPin
// PUL- GND
// PUL+ stepPin

//-- TWEAKABLES =====================================================================
// dirPin = pin DIR+
// stepPin = pin PUL+
// microstepping = settingan microstepping (1 / 2 / 4 / 8 / 16)
// dirInhale = arah untuk inhale (HIGH / LOW)
#define microstepping 4
#define dirInhale LOW

//-- PIN FISIK =======================================================================
#define enaPin 2
#define dirPin 4
#define stepPin 3
#define limitSwitchIn 5
#define limitSwitchEx 6
#define calManMaju 7
#define calManMundur 8
#define LEDCallibrate 13

bool callibrated = false;
/////////////////////////////////////////////////////////////////////////////////////

//-- Global Variables ===============================================================
String bufferq[7];
bool statusOn = 0;
bool warnVol = 0;
bool warnPres = 0;
int Vtidal = 0;
float IRat = 1;
float ERat = 0;
int RR = 0;
bool triggerInhale = 0;
int stateNow = 9;

String lastData = "<0,0,0,0,0,0,0>";
bool spontaneousPrev = false;

unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath, slopeFactor, initDelay;

//-- SETUP =========================================================================
void setup() {
  Serial.begin(115200);
  SerialM.begin(38400);

  slopeFactor = 0.5;
  delayInhale = 220; // dalam microseconds
  delayExhale = delayInhale; // dalam microseconds
  initDelay = 500;

  //////////// BREATHING PART //////////////////
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limitSwitchIn, INPUT_PULLUP);
  pinMode(limitSwitchEx, INPUT_PULLUP);

  pinMode(calManMaju, INPUT_PULLUP);
  pinMode(calManMundur, INPUT_PULLUP);

  Serial.println("==> CALLIBRATING"); Serial.flush();
  Callibrate();
  Serial.println("==> CALLIBRATION DONE"); Serial.flush();
}

void loop() {
  unsigned long timeInhaleReal;
  
  updateAllGlobalVars();
//  delayMicroseconds(500);

  if (statusOn) {
//      Serial.println(Vtidal);
      stepTidal = cekTidal(Vtidal);
      timeBreath = (60000 / float(RR)) * 1000;
      timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)) * 1000; // dalam microseconds
      timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)) * 1000; // dalam microseconds
      
      if(stateNow == 0){
        Serial.println("==> STATUS: ON");
      }
 
      if(spontaneousPrev){ stateNow = 2; Serial.println("==> STATE 2");}
      else{stateNow = 1;Serial.println("==> STATE 1");}

      spontaneousPrev = false;

      Serial.println("==================");
      Serial.println("Vol Tidal = " + String(Vtidal));
      Serial.println("Step Tidal = " + String(stepTidal));
      Serial.println("Slope Tidal = " + String(slopeFactor));
      Serial.println("----");
      Serial.println("DELAY Awal = " + String(initDelay));
      Serial.println("DELAY Inhale = " + String(delayInhale));
      Serial.println("DELAY Exhale = " + String(delayExhale));
      Serial.println("----");
      Serial.println("WAKTU BREATH = " + String(timeBreath));
      Serial.println("WAKTU IDEAL Inhale = " + String(timeInhale));
      Serial.println("WAKTU IDEAL Exhale = " + String(timeExhale));
      Serial.println("----");

      unsigned long now = micros();
      
      if(stateNow == 1){
        Inhale();
    
        while((micros()-now) < timeInhale){delayMicroseconds(1);}
        timeInhaleReal = micros()-now;
        Serial.println("==> TIME INHALE : " + String(timeInhaleReal));
          
        Exhale(stepTidal);
      } else if(stateNow == 2){
        int stepTidal2 = Inhale2();

        while((micros()-now) < timeInhale){delayMicroseconds(1);}
        timeInhaleReal = micros()-now;
        Serial.println("==> TIME INHALE : " + String(timeInhaleReal));
          
        Exhale(stepTidal2);
      }
      
  
      while((micros()-now) < timeBreath){delayMicroseconds(1);}
      Serial.println("==> TIME EXHALE : " + String(micros()-now - timeInhaleReal));
      
      Serial.println("TIME TAKEN : " + String(micros() - now));
      Serial.println("----");
      
  } else {
    if(stateNow !=0){
      Serial.println("==> STATUS: OFF");Serial.flush();
      stateNow = 0;
    }
    
    if (!callibrated) {
      Callibrate();
    }
  }
}

//== FUNCTION STATES ==================================================
//-- Sekuens Inhale ====================================================================
void Inhale() {
  // 0. Init Variables
  unsigned long now = micros();
  float delayInhale2 = initDelay;
  int stepCount = 0;

  // 1. Set Arah
  digitalWrite(dirPin, dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) { // UNTUK DIPERIKSA
    if(i < slopeFactor*stepTidal){
      delayInhale2 -= (initDelay-delayInhale) / (slopeFactor*stepTidal);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayInhale2 += (initDelay-delayInhale) / (slopeFactor*stepTidal);
    }

    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, HIGH);
    }

    delayMicroseconds(delayInhale2);

    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, LOW);
    }

    delayMicroseconds(delayInhale2);

    stepCount += 1;
  }

  Serial.println(stepCount);

  // 3. Tampil Waktu
  Serial.print("Waktu Inhale = ");
  Serial.println(micros() - now);
}

//-- Sekuens Inhale ====================================================================
int Inhale2() {
  // 0. Init Variables
  unsigned long now = micros();
  float delayInhale2 = initDelay;
  int stepCount = 0;

  // 1. Set Arah
  digitalWrite(dirPin, dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal && !checkVolume(); i++) {
    if(i < slopeFactor*stepTidal){
      delayInhale2 -= (initDelay-delayInhale) / (slopeFactor*stepTidal);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayInhale2 += (initDelay-delayInhale) / (slopeFactor*stepTidal);
    }

    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, HIGH);
    }

    delayMicroseconds(delayInhale2);

    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, LOW);
    }

    delayMicroseconds(delayInhale2);

    stepCount += 1;
  }

  Serial.println(stepCount);

  // 3. Tampil Waktu
  Serial.print("Waktu Inhale = ");
  Serial.println(micros() - now);
  return stepCount;
}

//-- Sekuens Exhale ====================================================================
void Exhale(int stepTidalE) {
  // 0. Init Variables
  unsigned long now = micros();
  float delayExhale2 = initDelay;

  // 1. Set Arah
  digitalWrite(dirPin, !dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidalE; i++) {
    if(i < slopeFactor*stepTidalE){
      delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidalE);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidalE);
    }

    if(checkInhale()){
      spontaneousPrev = true;
    }

    if(digitalRead(limitSwitchEx)){
      digitalWrite(stepPin, HIGH);
    }

    delayMicroseconds(delayExhale2);

    if(digitalRead(limitSwitchEx)){
      digitalWrite(stepPin, LOW);
    }

    delayMicroseconds(delayExhale2);
  }

  // 3. Tampil Waktu
  Serial.print("Waktu Exhale = ");
  Serial.println(micros() - now);
}


//-- FUNCTION PLUS PLUS =============================================
//-- Fungsi Kalibrasi
void Callibrate() {
  digitalWrite(LEDCallibrate, HIGH);
  digitalWrite(dirPin, !dirInhale);
  unsigned long now = millis();
  while(digitalRead(limitSwitchEx)){
    if (millis() - now > 2000){break;} // 2 detik maks kalibrasi
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  callibrated = true;
  digitalWrite(LEDCallibrate, LOW);
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
float cekTidal(float vol_Tidal){
  float lookup_vol[] = {500, 750, 1000, 1250};
  float lookup_step[] = {480, 550, 660, 700};

  float stepTidal = 0;
  int arraySize = sizeof(lookup_vol) / sizeof(lookup_vol[0]);

  // Extrapolasi Bawah
  if(vol_Tidal < cariMin(lookup_vol, arraySize)){
    float m = float(lookup_step[1] - lookup_step[0]) / (lookup_vol[1] - lookup_vol[0]);
    float c = float(lookup_step[0]) - lookup_vol[0] * m;
    stepTidal = m * vol_Tidal + c;
  }
  // Extrapolasi Atas
  else if(vol_Tidal > cariMax(lookup_vol, arraySize)){
    float m = float(lookup_step[arraySize-1] - lookup_step[arraySize-2]) / (lookup_vol[arraySize-1] - lookup_vol[arraySize-2]);
    float c = float(lookup_step[arraySize-1]) - lookup_vol[arraySize-1] * m;
    stepTidal = m * vol_Tidal + c;
  }
  // Normal + Interpolasi
  else {
    for(int i = 0; i< arraySize; i++) {
      if (vol_Tidal == lookup_vol[i]) {
        stepTidal = lookup_step[i];
      } else {
        if(vol_Tidal >= lookup_vol[i] && vol_Tidal < lookup_vol[i+1]) {
          stepTidal = lookup_step[i] + float(lookup_step[i+1] - lookup_step[i]) * float(vol_Tidal - lookup_vol[i]) / float(lookup_vol[i+1]-lookup_vol[i]);
          break;
        }
      }
    }
  }
  return stepTidal;
}

//-- fungsi tambahan lookup table
float cariMin(float list[], int arraySize){
  float mini = 99999;
  for(int i=0; i< arraySize; i++){
    if(list[i] <= mini) {
      mini = list[i];
    }
  }
  return mini;
}

float cariMax(float list[], int arraySize){
  float maxi = 0;
  for(int i=0; i< arraySize; i++){
    if(list[i] >= maxi) {
      maxi = list[i];
    }
  }
  return maxi;
}


//== FUNCTIONS SERIAL ==============================================
// Update all Global Variable
void updateAllGlobalVars(){
  String received = listeningMega();
//  Serial.print("Received: ");
//  Serial.println(received);
//  Serial.flush();

  int indexStart = 0;
  int indexEnd = 0;

  for(int i = 0; i<7; i++){
    indexEnd = received.indexOf("," , indexStart);
    bufferq[i] = received.substring(indexStart, indexEnd);
    indexStart = indexEnd+1;
//    Serial.println(String(i) + ": " + bufferq[i]);
    
  }

  statusOn = bufferq[0].toInt();
  warnVol = bufferq[1].toInt();
  warnPres = bufferq[2].toInt();
  Vtidal = bufferq[3].toInt();
  ERat = bufferq[4].toFloat();
  RR = bufferq[5].toInt();
  triggerInhale = bufferq[6].toInt();
//  Serial.println("StatusOn: " + String(statusOn));
//  Serial.println("warnVol: " + String(warnVol));
//  Serial.println("warnPres: " + String(warnPres));
//  Serial.println("Vtidal: " + String(Vtidal));
//  Serial.println("ERat: " + String(ERat));
//  Serial.println("RR: " + String(RR));
//  Serial.println("triggerInhale: " + String(triggerInhale));
//  Serial.println("--------");
//  delay(1000);
}

// Update Buffer from serial
String listeningMega(){
  bool quit = false;
  String seriesData = "";

//  Serial.println(F("Waiting data from Mega...")); Serial.flush();
  while (!quit) {
    if (SerialM.available() > 0) {
      char x = SerialM.read();
      if (x == '>'){
        seriesData += x;
        quit = true;
      } else {
        if (x == '<') {seriesData = "";}
        seriesData += x;
      }
    } else {
      seriesData = lastData;
      quit = true;
    }
  }
  lastData = seriesData;

  //!! Dummy Data !!
//  seriesData = "<0,0,0,1250,2,14,1>";
  
  String seriesData2 = seriesData.substring(1,seriesData.length()-1);

  return seriesData2;
}

bool checkInhale(){
  String received = listeningMega();
  triggerInhale = received.substring(received.length()-1, received.length()).toInt();
  return triggerInhale;
}

bool checkVolume(){
  String received = listeningMega();
  warnVol = received.substring(1,2).toInt();
  return warnVol;
}
