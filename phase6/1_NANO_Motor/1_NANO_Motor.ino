/*
**************************************
* Kode Untuk menggerakkan servo
* Bagian dari project Ventilator MUMU
* PT Sibernetika Teknologi Industri
*
* Device: Arduino Nano
* Author:
*      - Prasetyo Wibowo LS. (prasetyowls12@yahoo.com)
***************************************
*/


/* KONFIGURASI PIN *****************************
* ENA     (DO)    PIN ENABLE
* PUL     (DO)    PIN PULSE
* DIR     (DO)    PIN ARAH
* ARST    (DO)    PIN RESET
* CWL     (DI)    LIMIT SWITCH CLOCKWISE
* CCWL    (DI)    LIMIT SWITCH COUNTER-CLOCKWISE
* EMGS    (DI)    EMERGENCY BUTTON
* WARNN   (DI)    WARN TRIGGER (INTERRUPT)
* Bt1     (DI)    TOMBOL UNTUK MENGGERAKAN MANUAL (MAJU)
* Bt2     (DI)    TOMBOL UNTUK MENGGERAKAN MANUAL (MUNDUR)
*************************************************/
#define ENA 8
#define PUL 10
#define DIR 9
#define ARST 7
#define CWL 4
#define CCWL 5
#define EMGS 3
#define WARNN 2
#define Bt1 A4
#define Bt2 A5

/* DEFINISI VARIABEL ****************************
* slopeFactor = Faktor Sloping untuk akselerasi
* Offsetq   =   Offset untuk
* dirInhale =   Tegangan yang membuat motor gerak ke arah inhale (HIGH)
* warnq     =   Status warning
* stateq    =
* delayq    =

*************************************************/
#define slopeFactor 0.35
#define offsetq 11
#define dirInhale HIGH

bool warnq = false;
int stateq = 2;
int stepq = 0;
int delayq = 2000;

/* Input HMI ******************************
// volTidal = Volume Tidal (cc)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
*******************************************/
float volTidal = 0;
int IRat = 1;
float ERat = 1;
int RR = 10;

//-- GLOBAL VARIABLEs ===============================================================
unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath;
float initDelay = 25;
unsigned long p_vol, p_RR;
float p_IE;
int p_mode = 0;

int num_buf = 5;
String bufferq[5];
float p_del=0;
String lastData = "<0,0,0,0,0>";
bool updated = false;
unsigned long stepstop = 0;

/* SETUP ***************************************
- Set Serial BAUD Rate
- Update Parameter dengan parameter awal
- Set mode pin-pin & interrupt
- Lakukan kalibrasi (maju 1000 step kemudian mundur 8 detik)
************************************************/
void setup() {
  Serial.begin(115200);

  updateParam(volTidal, RR, ERat);

  pinMode(ENA, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ARST, OUTPUT);
  digitalWrite(ARST, HIGH);
  Serial.println("ON");
  delay(10);
  Serial.println("OFF");
  digitalWrite(ARST, LOW);

  pinMode(Bt1, INPUT_PULLUP);
  pinMode(Bt2, INPUT_PULLUP);

  pinMode(CWL, INPUT_PULLUP);
  pinMode(CCWL, INPUT_PULLUP);

  pinMode(EMGS, INPUT_PULLUP);
  pinMode(WARNN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EMGS), updateEMGS, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WARNN), triggerWarn, FALLING);

  //-- KALIBRASI
  Serial.println("Callibrating...");
  digitalWrite(ENA, HIGH);

  //Maju 1000 Step
  digitalWrite(DIR, dirInhale);
  delayMicroseconds(5);
  for(int i = 0; i < 1000; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1500);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1500);
    if(digitalRead(CWL) == HIGH){
      break;
    }
  }

  //Mundur 8 detik
  digitalWrite(DIR, !dirInhale);
  delayMicroseconds(5);

  unsigned long now = millis();
  while(digitalRead(CCWL) == LOW) {
    //if (millis() - now > 8000) {break;} // 2 detik maks kalibrasi
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1500);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1500);
  }

  digitalWrite(ENA, LOW);
  Serial.println("DONE, READY!");

}

/* LOOP ***************************************
- Listen for Serial Comm Input
- Update Global Variables
- Do Inhale and Exhale if Mode on & EMGS off
- If Mode off:
  -- Check for Button input
  -- Check for Limit Switches
***********************************************/
void loop() {
  updateAllGlobal();

  if(p_mode!=0 && !digitalRead(EMGS)){
    digitalWrite(ENA, HIGH);
    if (stepTidal > 0) {
        unsigned long now = micros();

        stepstop = Inhale2();

        delayMicroseconds(100);
        Serial.println("{ei}"); Serial.flush();

        Exhale(stepstop);

        while((micros()-now) < timeBreath){}

        Serial.println("{ec}"); Serial.flush();
//      }
    } else {
      //throw error message
    }
  } else {
    digitalWrite(ENA, LOW);
    if(digitalRead(CCWL) == HIGH){
      Serial.println("CCWL");
    }
    if(digitalRead(CWL) == HIGH){
      Serial.println("CWL");
    }
    if(digitalRead(EMGS)){
      Serial.println("{ee}");
    } else {
      Serial.println("Waiting.. (mode 0 )");
    }
    Serial.flush();
    delay(1000);
    if(digitalRead(Bt1) == LOW){
      digitalWrite(ENA, HIGH);
      digitalWrite(DIR, dirInhale);
      Serial.println("MAJU!");
      while(digitalRead(Bt1) == LOW){
        digitalWrite(PUL, HIGH);
        delayMicroseconds(delayq);
        digitalWrite(PUL, LOW);
        delayMicroseconds(delayq);
      }
    } else if(digitalRead(Bt2) == LOW){
      digitalWrite(ENA, HIGH);
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

void updateEMGS(){
  if(digitalRead(EMGS) == HIGH) { // ON
    //kirim perintah ke raspi
    Serial.println("{ee}");
  } else { //EMGS OFF
    Serial.println("EE off");
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
    p_del = bufferq[4].toFloat();

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
      seriesData = Serial.readString();
      quit = true;
    } else {
      seriesData = lastData;
      quit = true;
    }
  }

  lastData = seriesData;

  return seriesData.substring(1,seriesData.length()-1);
}

void updateParam(float vol, int RRq, float ERatq){
  stepTidal = vol;
  timeBreath = (60000 / float(RRq)) * 1000;
  timeInhale = (60000 / float(RRq)) * (float(IRat) / float(IRat + ERatq)) * 1000 - 500; // dalam microseconds

  float offsetInhaleq= 0;
  float a,b,c;

  timeInhale += p_del/2*1000000;

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

int Inhale2() { //assisted inhale
  // 0. Hitung Waktu
//  unsigned long now = micros();
//  float delayInhale2 = delayInhale-offsetq;
  int stepstop = stepTidal;
  float axq;
  float delayInhale2 = delayInhale-offsetq;

  // 1. Set Arah
  digitalWrite(DIR, dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(delayInhale2);
    digitalWrite(PUL, LOW);
    delayMicroseconds(delayInhale2);
    if(warnq == true) {
      stepstop = i;
      break;
    }
  }
  warnq = false;
  // 3. Tampil Waktu
//  Serial.print("Waktu Inhale = ");
//  Serial.println(micros() - now);
  return stepstop;
}

void triggerWarn(){
  warnq = true;
}

//-- Sekuens Exhale ====================================================================
void Exhale(int stepTidalq) {
  // 0. Hitung Waktu
  unsigned long now = micros();
  float delayExhale2 = initDelay;

  // 1. Set Arah
  digitalWrite(DIR, !dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidalq; i++) {
    if(i < slopeFactor*stepTidalq){
      delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidalq);
    }
    if(i>(1-slopeFactor)*stepTidalq){
      delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidalq);
    }
//    if(digitalRead(limitSwitchEx)){
    digitalWrite(PUL, HIGH);
//    }

    delayMicroseconds(delayExhale2);

//    if(digitalRead(limitSwitchEx)){
    digitalWrite(PUL, LOW);
//    }

    delayMicroseconds(delayExhale2);

    if(digitalRead(CCWL) == HIGH){
      Serial.println("CCWL");
      break;
    }
  }

  // 3. Tampil Waktu
//  Serial.print("Waktu Exhale = ");
//  Serial.println(micros() - now);
}
