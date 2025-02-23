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

//-- Input HMI ======================================================================
// volTidal = Volume Tidal (cc)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
float volTidal = 0;
int IRat = 1;
float ERat = 1;
int RR = 10;

//-- GLOBAL VARIABLEs ===============================================================
unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath, slopeFactor;
float initDelay = 25;
unsigned long p_vol, p_RR;
float p_IE;
int p_mode = 0;

int num_buf = 4;
String bufferq[4];
String lastData = "<0,0,0,0>";
bool updated = false;


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
  updateAllGlobal();
  
  if(p_mode!=0){
    if (stepTidal > 0) {
      unsigned long now = micros();
      
      Inhale();
      
      Serial.println("{ei}"); Serial.flush();
      delayMicroseconds(100);
      
      Exhale();
      
      while((micros()-now) < timeBreath){}
      
      Serial.println("{ec}"); Serial.flush();
    } else {
      //throw error message
    }
  } else {
    Serial.println("Waiting.. (mode 0 )");
    Serial.flush();
    delay(1000);
    if(digitalRead(Bt1) == LOW){
      digitalWrite(DIR, dirInhale);
      Serial.println("MAJU!");
      while(digitalRead(Bt1) == LOW){
        digitalWrite(PUL, HIGH);
        delayMicroseconds(delayq);
        digitalWrite(PUL, LOW);
        delayMicroseconds(delayq);
      }
    } else if(digitalRead(Bt2) == LOW){
      digitalWrite(DIR, !dirInhale);
      Serial.println("MUNDUR!");
      while(digitalRead(Bt2) == LOW){
        digitalWrite(PUL, HIGH);
        delayMicroseconds(delayq);
        digitalWrite(PUL, LOW);
        delayMicroseconds(delayq);
      }
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
    }

    p_mode = bufferq[0].toInt();
    p_vol = bufferq[1].toInt();
    p_RR = bufferq[2].toInt();
    p_IE = bufferq[3].toFloat();

    updateParam(p_vol, p_RR, p_IE);
    
    updated = true;
  }
}

void updateAllGlobal(){
  cekNewParam();
}

String listeningMega(){
  bool quit = false;
  String seriesData = "";

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
  
//  if (Serial.available() > 0) {
//    updated = false;
//    seriesData = Serial.readString();
//    quit = true;
//  } else {
//    seriesData = lastData;
//    quit = true;
//  }
//    
//  if(seriesData == lastData) {
//    updated = true;
//  }

  lastData = seriesData;

  return seriesData.substring(1,seriesData.length()-1);
}

void updateParam(float vol, int RRq, float ERatq){
  stepTidal = 0;
  timeBreath = (60000 / float(RRq)) * 1000;
  timeInhale = (60000 / float(RRq)) * (float(IRat) / float(IRat + ERatq)) * 1000 - 500; // dalam microseconds

  float offsetInhaleq= 0;
  float a,b,c;

  if(vol == 300){
    stepTidal = 4000;
    if(ERatq ==1){
      c = 0.000650547;
      b = -0.028619167;
      a = 0.354092703;
      if(RRq == 10){stepTidal=4500;}
      if(RRq == 12){stepTidal=4300;}
      if(RRq == 14){stepTidal=4150;}
    } else if(ERatq == 2){
      c = 0.000273647;
      b = -0.018056717;
      a = 0.190179239;
    } else if(ERatq == 3){
      c = 0.000177588;
      b = -0.016726554;
      a = 0.14036616;
      if(RRq == 25){stepTidal=4200;}
      if(RRq == 30){stepTidal=4200;}
    }
  } 
  
  else if(vol == 400){
    stepTidal = 4750;
    if(ERatq ==1){
      c = 2.80764E-05;
      b = 0.002991068;
      a = -0.023063682;
      if(RRq == 10){stepTidal=5000;}
      if(RRq == 12){stepTidal=4850;}
    } else if(ERatq == 2){
      c = 0.000264859;
      b = -0.016527119;
      a = 0.152748413;
      if(RRq == 20){stepTidal=4800;}
      if(RRq == 25){stepTidal=4900;}
      if(RRq == 30){stepTidal=4900;}
    } else if(ERatq == 3){
      c = 0.000154945;
      b = -0.020846611;
      a = 0.178253482;
      if(RRq == 20 || RRq == 25 || RRq == 30){stepTidal=5200;}
      if(RRq == 18 || RRq == 16){stepTidal=4900;}
    }
  } else if(vol == 500){
    stepTidal = 5430;
    if(ERatq ==1){
      c = -6.55388E-06;
      b =  0.004163627;
      a = -0.051711605;
    } else if(ERatq == 2){
      c = 0.000361423;
      b = -0.020693201;
      a = 0.159466747;
      if(RRq == 25){stepTidal=5600;}
      if(RRq == 30){stepTidal=5800;}
    } else if(ERatq == 3){
      c = 4.48447E-05;
      b = -0.014660329;
      a = 0.06594389;
      if(RRq == 15){stepTidal=5500;}
      if(RRq == 16){stepTidal=5600;}
      if(RRq == 18){stepTidal=5800;}
      if(RRq == 20){stepTidal=5900;}
      if(RRq == 25){stepTidal=6000;}
      if(RRq == 30){stepTidal=6100;}
    }
  } else if(vol == 600){
    stepTidal = 6060;
    if(ERatq ==1){
      c = -0.000252947;
      b = 0.008377475;
      a = -0.000950374;
      if(RRq == 30){stepTidal=6200;}
    } else if(ERatq == 2){
      c = -0.000345664;
      b = 0.002713639;
      a = -0.034269684;
      if(RRq == 25){stepTidal=6300;}
      if(RRq == 30){stepTidal=6400;}
    } else if(ERatq == 3){
      c = 0.000479443;
      b = -0.031616575;
      a = 0.18875765;
      if(RRq == 12){stepTidal=6100;}
      if(RRq == 14){stepTidal=6100;}
      if(RRq == 15){stepTidal=6100;}
      if(RRq == 16){stepTidal=6100;}
      if(RRq == 18){stepTidal=6200;}
      if(RRq == 20){stepTidal=6350;}
      if(RRq == 25){stepTidal=6400;}
      if(RRq == 30){stepTidal=6400;}
    }
  }

  offsetInhaleq = a + b*RRq + c*RRq*RRq;
  
  timeInhale += offsetInhaleq/2*1000000;
  
  timeExhale = (60000 / float(RRq)) * (float(ERatq) / float(IRat + ERatq)) * 1000; // dalam microseconds
  delayInhale = float(timeInhale) / float(stepTidal) / 2; //endDelay; // dalam microseconds
  delayExhale = 20; //delayInhale; // dalam microseconds

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
  Serial.println("WAKTU IDEAL Inhale = " + String(timeInhale));
  Serial.println("WAKTU IDEAL Exhale = " + String(timeExhale));
  Serial.println("----");
  
  Serial.println("updateParam()");
  Serial.flush();
}


//-- Sekuens Inhale ====================================================================
void Inhale() {
  // 0. Hitung Waktu
//  unsigned long now = micros();
//  float delayInhale2 = delayInhale-offsetq;
  bool ntab = false;
  float axq;
  float delayInhale2 = delayInhale-offsetq;
  
//  if(!p_ntab){
//    ntab = false;
//    Serial.println("NotNtab");Serial.flush();
//  } else {
//    delayInhale2 = 0.7*delayInhale;
//    axq = 1/(0.4*stepTidal-1) * (1.5)*delayInhale; //1.38
//    Serial.println("Ntab");Serial.flush();
//    Serial.println(axq);
//  }
  
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
    if(digitalRead(CCWL) == HIGH){ // NEW , kalau limitswitch aktif
      break;
     }
  }
  
  // 3. Tampil Waktu
//  Serial.print("Waktu Exhale = ");
//  Serial.println(micros() - now);
}
