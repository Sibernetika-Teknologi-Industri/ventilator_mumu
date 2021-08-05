/*
********************************************************
* Kode Untuk menggerakkan servo
* Bagian dari project Ventilator MUMU
* PT Sibernetika Teknologi Industri
*
* Device: Arduino Nano
* Author:
*      - Prasetyo Wibowo LS. (prasetyowls12 (at) yahoo (dot) com)
*********************************************************
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
*  slopeFactor  =   Faktor Sloping untuk akselerasi
*  Offsetq      =   Offset untuk
*  dirInhale    =   Tegangan yang membuat motor gerak ke arah inhale (HIGH)
*  delayq       =   Delay pulse untuk gerakan santai (input manual dari button)
*  IRat         =   Ratio Inhale
*************************************************/
#define slopeFactor 0.35
#define offsetq 11
#define dirInhale HIGH
#define delayq 2000
#define IRat 1

/* GLOBAL VARIABLEs ******************************/
unsigned long stepTidal, delayInhale, delayExhale;
float timeInhale, timeExhale, timeBreath;
float initDelay = 25;

unsigned long p_vol, p_RR;
float p_IE;
int p_mode = 0;
float p_del = 0;
bool warnq = false;

int num_buf = 5;
String bufferq[5];
String lastData = "<0,0,0,0,0>";
bool updated = false;

unsigned long stepstop = 0;
int stepq = 0;


/* FUNCTION PROMISES ****************************/
// Comm Functions
void updateAllGlobal();
String readSerial();
void updateParam(float, int, float);

// Movement Functions
int Inhale2();
void Exhale(int);

// Interrupt Functions
void updateEMGS();
void triggerWarn();

/********************************************* VARIABEL I:E DAN RR (Riyan) ****************************/
unsigned long waktuMillis = 0; //Waktu milisecond
unsigned long waktuInspirasi = 0; //Waktu ketika Inspirasi
unsigned long waktuEkspirasi = 0; //Waktu ketika Expirasi (relase motor servo)
unsigned long waktuEkspirasi2 = 0; //Waktu ketika Expirasi (relase setelah Limit switch tersentuh)
float pen_i = 0; // variabel penampung inspirasi
float pen_e = 0; // variabel penampung Ekspirasi
float pen_cycle = 0; //variabel penampung Cycle
int pen_rr = 0; // variabel penampung RR
int pen_ratio = 0; // Variabel penampung ratio 
/********************************************* VARIABEL I:E DAN RR (Riyan) ****************************/


/* SETUP ***************************************
- Set Serial BAUD Rate
- Update Parameter dengan parameter awal
- Set mode pin-pin & interrupt
- Lakukan kalibrasi (maju 1000 step kemudian mundur 8 detik)
************************************************/
void setup() {
  Serial.begin(115200);

  // updateParam(0, 10, 1);

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

  // KALIBRASI ---
  Serial.println("Callibrating...");
  digitalWrite(ENA, HIGH);

  // Maju 1000 Step
  digitalWrite(DIR, dirInhale);
  delayMicroseconds(5);
  for(int i = 0; i < 1000; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1500);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1500);
    if(digitalRead(CWL) == HIGH) break;
  }

  // Mundur 8 detik
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
  // 1. Get Command from Serial
  updateAllGlobal();

  // 2a. If Mode ON
  if(p_mode!=0 && !digitalRead(EMGS)){
    // 2a.1. Enable the Actuator
    digitalWrite(ENA, HIGH);

    if (stepTidal > 0) {
        unsigned long now = micros();

        // 2a.2. Do Inhale
        stepstop = Inhale2();
        // Added Delay for Plateau Pressure
        delayMicroseconds(100);
        // Send Serial untuk menandakan inhale selesai (end inhale)
        // Serial.println("{ei}"); 
// ************************************* Inhale Proses (Riyan)********************************************
            pen_i = waktuInspirasi; // Menampung waktu inspirasi ke dalam variabel pen_i
            Serial.println('{' + String("{ei}") + ',' + String(pen_rr) + ',' +  String(pen_ratio) + '}'); // Menampilkan data ketika Inhale selesai
            waktuEkspirasi = 0 ; // Mereset Expirasi (dari motor)
            waktuEkspirasi2 = 0 ; // Mereset Expirasi (setelah Limitswitch Luar tertekan)
// ************************************* Inhale Proses (Riyan)********************************************
        Serial.flush();

        // 2a.3. Do Exhale
        Exhale(stepstop);
        // Idling
        while((micros()-now) < timeBreath){
//***********************************Hitung waktu Ekspirasi setelah limitswitch tertekan (Riyan)********************************************
            if (millis()-waktuMillis>=1){
            waktuEkspirasi2  = waktuEkspirasi2 + 1;
            waktuMillis = millis();
              }
//***********************************Hitung waktu Ekspirasi setelah limitswitch tertekan (Riyan)********************************************
          }
        // Send Serial untuk menandakan exhale selesai (end exhale)
        // Serial.println("{ec}"); 
// *************************************Perhitungan RR, I:E, dan Exhale Proses (Riyan)*************************************
              pen_e = waktuEkspirasi + waktuEkspirasi2; // Penggabungan waktu Expirasi motor dan expirasi setelah limitswitch tertekan
              pen_cycle = (pen_i) + (pen_e); //penggabungan waktu Inpirasi dan Ekspirasi
              pen_rr = round(60000/pen_cycle); //perhitungan waktu Respiration Rate (RR)
              pen_ratio = round(pen_e / pen_i); //Perhitungan I:E 
              Serial.println('{' + String("{ec}") + ',' + String(pen_rr) + ',' +  String(pen_ratio) + '}'); // Menampilkan data ketika Exhale selesai
              waktuInspirasi = 0 ; // Mereset waktu Inspirasi
// *************************************Perhitungan RR, I:E, dan Exhale Proses (Riyan)*************************************     
        Serial.flush();

    } else {
      //throw error message
    }
  }

  // 2b. If Mode OFF
  else {
    // 2b.1. Disable the Actuator
    digitalWrite(ENA, LOW);

    // 2b.2. Check For Limit Switches
    if(digitalRead(CCWL) == HIGH){
      Serial.println("CCWL");
    }
    if(digitalRead(CWL) == HIGH){
      Serial.println("CWL");
    }

    // 2b.3. Check For Emergency Button
    if(digitalRead(EMGS)){
      Serial.println("{ee}");
    } else {
      Serial.println("Waiting.. (mode 0 )");
    }

    Serial.flush();
    delay(1000);

    // 2b.4. Check for Manual Input
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

/**************************
*      COMM FUNCTIONS     *
***************************/

/* Update Global Variables **********************
*  Update most global values dari data Serial
*  return: None
*************************************************/
void updateAllGlobal(){
  String received = readSerial();
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

/* Read Serial **************************
*  Fungsi untuk membaca string yang masuk ke Serial
*  return: String
********************************************/
String readSerial(){
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

/* Update Param **********************
*  Update parameter gerakan berdasar variable global baru
*  return: None
*************************************************/
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


/**************************
*   MOVEMENT FUNCTIONS    *
***************************/

/* Inhale Sequence ****************************/
int Inhale2() { //assisted inhale
  // 0. Hitung Waktu
  int stepstop = stepTidal;
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

//***********************************Hitung waktu Inspirasi (Riyan)********************************************
      if (millis()-waktuMillis>=1){
      waktuInspirasi = waktuInspirasi + 1;
      waktuMillis =millis();
      }
//***********************************Hitung waktu Inspirasi (Riyan)********************************************

    
    if(warnq == true) {
      stepstop = i;
      break;
    }
  }
  warnq = false;

  // Return jumlah step yang telah dilakukan
  return stepstop;
}


/* Exhale Sequence **********************/
void Exhale(int stepTidalq) {
  // 0. Hitung Waktu
  unsigned long now = micros();
  float delayExhale2 = initDelay;

  // 1. Set Arah
  digitalWrite(DIR, !dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidalq; i++) {


//***********************************Hitung waktu Ekspirasi (Riyan)********************************************
        if (millis()-waktuMillis>=1){
        waktuEkspirasi  = waktuEkspirasi + 1;
        waktuMillis = millis();
        }
//***********************************Hitung waktu Ekspirasi (Riyan)********************************************


    
    if(i < slopeFactor*stepTidalq){
      delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidalq);
    }
    if(i>(1-slopeFactor)*stepTidalq){
      delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidalq);
    }

    digitalWrite(PUL, HIGH);
    delayMicroseconds(delayExhale2);
    digitalWrite(PUL, LOW);
    delayMicroseconds(delayExhale2);   

    if(digitalRead(CCWL) == HIGH){
      Serial.println("CCWL");
      break;
    }
  }
}



/**************************
*   INTERRUPT FUNCTIONS   *
***************************/

/* Update Status Warning ***************************/
void triggerWarn(){
  warnq = true;
}

/* Update Status EMGS ***************************/
void updateEMGS(){
  if(digitalRead(EMGS) == HIGH) {
    //kirim perintah ke raspi
    Serial.println("{ee}");
  } else { //EMGS OFF
    Serial.println("EE off");
  }
}
