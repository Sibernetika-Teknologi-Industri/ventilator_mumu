// Perlu data kalibrasi jml step vs volume tidal

// -- KONFIGURASI PIN ===============================================================
// ENA- NC
// ENA+ NC
// DIR- GND
// DIR+ DIR
// PUL- GND
// PUL+ PUL

//-- TWEAKABLES =====================================================================
// DIR = pin DIR+
// PUL = pin PUL+
// microPULg = settingan microPULg (1 / 2 / 4 / 8 / 16)
// dirInhale = arah untuk inhale (HIGH / LOW)
#define ENA 4
#define PUL 5
#define DIR 6

#define offsetq 8

#define Bt1 A0
#define Bt2 A1
#define Bt3 A2
#define Bt4 A4

#define dirInhale HIGH
int stateq = 2;
int stepq = 0;
int delayq = 2000;

// Slope2an
//#define initDelay 750
#define endDelay 1200

//-- Input HMI ======================================================================
// volTidal = Volume Tidal (cc)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
float volTidal = 300;
int IRat = 1;
float ERat = 1;
int RR = 10;

//-- GLOBAL VARIABLEs ===============================================================
unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath, slopeFactor;
float initDelay = 25;
unsigned long p_vol, p_RR;
float p_IE;

int num_buf = 4;
String bufferq[4];
String lastData = "<0,0,0,0>";
bool updated = false;
bool p_ntab = false;

//-- SETUP ==========================================================================
void setup() {
  Serial.begin(115200);
  
  slopeFactor = 0.35;
  updateParam(volTidal, RR, ERat);
  
//  timeInEx = stepTidal * delayInhale/; 

  pinMode(ENA, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  pinMode(Bt1, INPUT_PULLUP);
  pinMode(Bt2, INPUT_PULLUP);
  pinMode(Bt3, INPUT_PULLUP);
  pinMode(Bt4, INPUT_PULLUP);
}

//-- LOOP ============================================================================
void loop() {
  if(digitalRead(Bt4) == LOW){
//    digitalWrite(2, HIGH);
//    Serial.println("BREATHING ====================="); Serial.flush();
    if (stepTidal > 0) {
      unsigned long now = micros();
      
      Inhale();

//      while((micros()-now) < timeInhale){
////        delayMicroseconds(1);
//      }
  
//      unsigned long timeInhaleReal = micros()-now;
      
      Serial.println("==> TIME INHALE : " + String(micros()-now));Serial.flush();
      delayMicroseconds(100);
      Exhale();
      
      while((micros()-now) < timeBreath){
//        delayMicroseconds(1);
      }
//      Serial.println("==> TIME EXHALE : " + String(micros()-now - timeInhaleReal));
      Serial.println("TIME TAKEN : " + String(micros() - now));Serial.flush();
//      Serial.println("----");
    } else {
      //throw error message
 
    }
  }else if(digitalRead(Bt3) == LOW) {
    while(digitalRead(Bt3) == LOW){
      cekNewParam();
    }
    updateParam(p_vol, p_RR, p_IE);
  }else if(digitalRead(Bt1) == LOW){
    digitalWrite(DIR, dirInhale);
    Serial.println("MAJU!");
//    if(stateq != 1){
//      stepq = 0;
//      stateq = 1;
//    }
    while(digitalRead(Bt1) == LOW){
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delayq);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delayq);
//      stepq +=1;
//      Serial.println("Step = " + String(stepq));
    }
  } else if(digitalRead(Bt2) == LOW){
    digitalWrite(DIR, !dirInhale);
    Serial.println("MUNDUR!");
//    if(stateq != 0){
//      stepq = 0;
//      stateq = 0;
//    }
    while(digitalRead(Bt2) == LOW){
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delayq);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delayq);
//      stepq +=1;
//      Serial.println("Step = " + String(stepq));
    }
  }
}

void cekNewParam(){
  String received = listeningMega();
  if(!updated) {
    Serial.print("Received: ");
    Serial.println(received);
    Serial.flush();


    int indexStart = 0;
    int indexEnd = 0;

    for(int i = 0; i<num_buf; i++) {
      indexEnd = received.indexOf(",", indexStart);
      bufferq[i] = received.substring(indexStart, indexEnd);
      indexStart = indexEnd+1;
      //    Serial.println(String(i) + ": " + bufferq[i]);
    }

    p_vol = bufferq[0].toInt();
    p_RR = bufferq[1].toInt();
    p_IE = bufferq[2].toFloat();
    p_ntab = bufferq[3].toInt();

    updated = true;
  }
}

String listeningMega(){
  bool quit = false;
  String seriesData = "";

//  Serial.println(F("Waiting data from Mega...")); Serial.flush();
  while (!quit) {
    if (Serial.available() > 0) {
      updated = false;
      char x = Serial.read();
      if (x == '>') {
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

  if(seriesData == lastData) {
    updated = true;
  }

  lastData = seriesData;

  //!! Dummy Data !!
//  seriesData = "<1,0,0,350,2,14,0>";

  return seriesData.substring(1,seriesData.length()-1);
}

void updateParam(float vol, int RRq, float ERatq){
//  float offsetVolq;
//  if(vol==300){
//    offsetVolq = -9.4*ERatq*ERatq-0.393838979*RRq*RRq+12.74840452*RRq+44.4*ERatq-88.24752086;
//  } else{
//    offsetVolq = 0;
//  }
//  vol -= offsetVolq;
  stepTidal = cekTidal(vol);
  timeBreath = (60000 / float(RRq)) * 1000;
  timeInhale = (60000 / float(RRq)) * (float(IRat) / float(IRat + ERatq)) * 1000 - 500; // dalam microseconds

  float offsetInhaleq= 0;
//  if(vol == 300){
//    offsetInhaleq = -0.012412439*RRq-0.157731744*ERatq+0.296506979;
//  }
//  else if(vol==400){
//    offsetInhaleq = 0.134687073-0.00701688*RRq-0.084229897*ERatq;
//  } else if(vol==500){
//    offsetInhaleq = -0.009570213*RRq-0.169627944*ERatq+0.348851729;
//    offsetInhaleq = offsetInhaleq*0.5;
//  }else if(vol==600){
//    offsetInhaleq = -0.011453781*RRq-0.189504356*ERatq+0.38605728;
//    offsetInhaleq = offsetInhaleq*0.7;
//  }
//  Serial.println(offsetInhaleq);
//  Serial.println(offsetVolq);
//  timeInhale += offsetInhaleq*1000000/2;
  timeExhale = (60000 / float(RRq)) * (float(ERatq) / float(IRat + ERatq)) * 1000; // dalam microseconds
  delayInhale = float(timeInhale) / float(stepTidal) / 2; //endDelay; // dalam microseconds
  delayExhale = 20; //delayInhale; // dalam microseconds
// if(vol == 650){
//    delayInhale = delayInhale/0.69*0.5;
//  }
 

  Serial.println("----");
  Serial.println("RR = " + String(RRq));
  Serial.println("IE = " + String(ERatq));
  Serial.println("Vol Tidal = " + String(vol));
  Serial.println("Step Tidal = " + String(stepTidal));
  Serial.println("Slope Tidal = " + String(slopeFactor));
  Serial.println("----");
  Serial.println("DELAY Awal = " + String(initDelay));
  Serial.println("DELAY Inhale = " + String(delayInhale));
  Serial.println("DELAY Exhale = " + String(delayExhale));
  Serial.println("----");
  Serial.println("WAKTU BREATH = " + String(timeBreath));
  Serial.println("WAKTU IDEAL Inhale = " + String(timeInhale-offsetInhaleq*1000000));
  Serial.println("WAKTU IDEAL Exhale = " + String(timeExhale));
  Serial.println("----");
  
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
float cekTidal(float vol_Tidal){
  float lookup_vol[] =  { 235,  250,   279,  308,  322,  352,  382,  410,  442,  476,  510,  546,  581,  599,  616,  646,  673};
  float lookup_step[] = {3500, 3600,  3850, 3900, 4000, 4200, 4400, 4600, 4800, 5000, 5200, 5400, 5600, 5700, 5800, 6000, 6200};

//  float lookup_vol[] = {  216,  261,  295,  311,  327,  347,  364,  380,  394,  402,  411,  427,  444,  462,  495,  510,  544,  561,  595,  613,  630,  659,  672};
//  float lookup_step[] = {3500, 3800, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4650, 4700, 4800, 4900, 5000, 5200, 5300, 5500, 5600, 5800, 5900, 6000, 6200, 6300};

//  float lookup_vol[] = {100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750};
//  float lookup_step[] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500};
  
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
//  return vol_Tidal*10;
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



//-- Sekuens Inhale ====================================================================
void Inhale() {
  // 0. Hitung Waktu
//  unsigned long now = micros();
//  float delayInhale2 = delayInhale-offsetq;
  bool ntab = true;
  float axq;
  float delayInhale2 = delayInhale-offsetq;
  
  if(!p_ntab){
    ntab = false;
    Serial.println("NotNtab");Serial.flush();
  } else {
    delayInhale2 = 0.7*delayInhale;
    axq = 1/(0.4*stepTidal-1) * (1.5)*delayInhale; //1.38
    Serial.println("Ntab");Serial.flush();
    Serial.println(axq);
  }
  
//  if(ntab){}/
  
  // 1. Set Arah
  digitalWrite(DIR, dirInhale);
  delayMicroseconds(5);
  
  // 2. Set Gerakan Stepper
  if(ntab){
    for(int i = 0; i < stepTidal; i++) {
      if(i>0.6*stepTidal){
        delayInhale2 += axq;
      }
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delayInhale2);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delayInhale2);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(delayInhale2);
    digitalWrite(PUL, LOW);
    delayMicroseconds(delayInhale2);
  }
  }
  // 3. Tampil Waktu
//  Serial.print("Waktu Inhale = ");
//  Serial.println(micros() - now);
}

//-- Sekuens Exhale ====================================================================
void Exhale() {
  // 0. Hitung Waktu
  unsigned long now = micros();
  float delayExhale2 = initDelay;
  
  // 1. Set Arah
  digitalWrite(DIR, !dirInhale);
  delayMicroseconds(5); 

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    if(i < slopeFactor*stepTidal){
      delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidal);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidal);
    }
//    if(digitalRead(limitSwitchEx)){
    digitalWrite(PUL, HIGH);
//    }

    delayMicroseconds(delayExhale2);
    
//    if(digitalRead(limitSwitchEx)){
    digitalWrite(PUL, LOW);
//    }

    delayMicroseconds(delayExhale2);
  }
  
  // 3. Tampil Waktu
//  Serial.print("Waktu Exhale = ");
//  Serial.println(micros() - now);
}
