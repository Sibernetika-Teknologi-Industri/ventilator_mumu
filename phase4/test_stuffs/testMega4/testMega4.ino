/* Low Cost Ventilator - ITB
 * This is the program for the main microcontroller (Arduino  \
 * Mega). Read sensors, signaling motor controller (Arduino   \
 * Nano), and communicate with HMI (Nextion).
 */

//== LIBRARIES =============================================
#include <SoftwareSerial.h>
#include <Nextion.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Servo.h>

SoftwareSerial SerialFl(62, 63); //A8 A9
//== GLOBAL VARIABLES ======================================
// Servos
Servo servoPEEP;
Servo servoOxigen;
#define enaServoPEEP 38
#define sigServoPEEP 36
#define enaServoOx 42
#define sigServoOx 40

// Warning-Warning
#define warningVolume_PIN 31
#define warningPressure_PIN 30
#define triggerInhalePin 31
boolean warningVolume = 0;
boolean warningPressure = 0;
boolean triggerInhale = 0;

// Pressure States
float pip_value = 0;
float ipp_value = 0;
bool exhaleStage = false;

// Nextion variables
uint32_t state = 0;
uint32_t IE = 20;
uint32_t RR = 10;
uint32_t PEEP = 5;
uint32_t Vti = 300;
uint32_t Ox = 20;

// buffers
uint32_t prev_Vti = 0;
uint32_t prev_Ox = 0;
float ERat = 2;

int CurrentPage;
int mode = 0;

// new variables
int PEEP_LIMIT = 5;
int PIP_LIMIT = 100;
bool spuriousDetected = false;
#define warningPEEP_PIN 39
#define ButtonResetAlarm_PIN 40
byte alarms[9] = {0,0,0,0,0,0,0,0,0};

NexNumber n7 = NexNumber(1, 19, "n7");
NexNumber n8 = NexNumber(1, 20, "n8");
NexNumber n9 = NexNumber(1, 21, "n9");
NexNumber n10 = NexNumber(1, 22, "n10"); //Preserved for IPP
NexNumber n11 = NexNumber(1, 23, "n11"); //Preserved for PEEP
NexText t1 = NexText(1, 9, "t1");//Preserved for Alarm (send t1.pco = 63488), normally 12678
NexDSButton bt0 = NexDSButton(1, 3, "bt0");
NexButton b7 = NexButton(4, 20, "b7");
NexButton b8 = NexButton(4, 19, "b8");
NexButton b9 = NexButton(4, 21, "b9");
NexButton b10 = NexButton(4, 22, "b10");
NexButton b11 = NexButton(4, 26, "b11");
NexButton b12 = NexButton(4, 25, "b12");
NexButton b13 = NexButton(4, 24, "b13");
NexButton b14 = NexButton(4, 23, "b14");
NexButton b15 = NexButton(4, 27, "b15");
NexButton b16 = NexButton(4, 28, "b16");
NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");
NexPage page4 = NexPage(4, 0, "page4");
NexPage page6 = NexPage(6, 0, "page6");

NexTouch *nex_listen_list[] = {
	&n7,&n8,&n9,&n10,&n11,
	&b7,&b8,&b9,&b10,&b11,&b12,&b13,&b14,&b15,&b16,
	&bt0, &page0, &page1, &page2, &page4, &page6, NULL
};

// Pressure and Flow variables
const int PIN_MPX5010DP_pressure = A1;
const int PIN_MPX5010DP_flow = A0;
uint8_t pressure_int8;
uint8_t flow_int8;
int pressure_raw;
int flow_raw;
float pressure_float;
float pressure_float2;
float flow_float;

// KE-25 Oxygen Sensor + ADS1115 ADC board
Adafruit_ADS1115 ads;
float scalefactor = 0.1875F;
float volts = 0.0;
int16_t oxygen_raw = 0;
float oxygen_float;

bool readPEEP = false;
bool readIPP = false;

double volumeAcc = 0;
double dt;
unsigned long now = 0;
//== MAIN SETUP ============================================
void setup() {
	Serial.begin(115200);   // for debugging
	Serial1.begin(57600);   // from/to Nano
	Serial2.begin(115200);   // from/to Nextion #Updated to 115200
	Serial3.begin(115200); // NANO alarm
	SerialFl.begin(57600); // NANO FLOW

	ads.begin();      // from/to ADS115 + Oxygen
//  ads.setGain(GAIN_SIXTEEN);

	pinMode(3, INPUT_PULLUP);
	pinMode(9, OUTPUT);
	pinMode(warningPressure_PIN, OUTPUT);
	pinMode(warningVolume_PIN, OUTPUT);
	attachInterrupt(digitalPinToInterrupt(3), readPEEPQ, FALLING);
	attachInterrupt(digitalPinToInterrupt(2), readIPPQ, FALLING);

	servoPEEP.attach(sigServoPEEP);
	servoOxigen.attach(sigServoOx);
	pinMode(enaServoPEEP, OUTPUT);
	pinMode(enaServoOx, OUTPUT);

  // Tombol
  pinMode(ButtonResetAlarm_PIN, INPUT_PULLUP);

	nexInit();
	bt0.attachPush(bt0PushCallback, &bt0);
	b7.attachPush(b7PushCallback, &b7);
	b8.attachPush(b8PushCallback, &b8);
	b9.attachPush(b9PushCallback, &b9);
	b10.attachPush(b10PushCallback, &b10);
	b11.attachPush(b11PushCallback, &b11);
	b12.attachPush(b12PushCallback, &b12);
	b13.attachPush(b13PushCallback, &b13);
	b14.attachPush(b14PushCallback, &b14);
	b15.attachPush(b15PushCallback, &b15);
	b16.attachPush(b16PushCallback, &b16);
	page0.attachPush(page0PushCallback, &page0);
	page1.attachPush(page1PushCallback, &page1);
	page2.attachPush(page2PushCallback, &page2);
	page4.attachPush(page4PushCallback, &page4);
	page6.attachPush(page6PushCallback, &page6);
//  dbSerialPrintln(CurrentPage);
}


//== MAIN LOOP =============================================
void loop() {
	update2Nano();
	Serial.println("state" + String(state));

  // state = 0, artinya mesin berhenti
  // state = 1, mesin jalan

  // mode untuk loop nya
  // mode 5 = routine stuck ketika alarm level = HIGH

  // Page 0 welcome
  // Page 1 main display (yg ada grafik)
  // Page 2 pemilihan mode (pemilihan assist/mandatory)
  // Page 4 Setting/config (IE, RR, Vti, PEEP, O2)
  // Page 6 Setting alarm (over PIP, under IPP, O2_tolerance)

	if (state == 0 && CurrentPage == 1) {mode = 0;}
	if (state == 1 && CurrentPage == 1) {mode = 1;}
	if (CurrentPage == 2) {mode = 2;}     // page 2
	if (CurrentPage == 4) {mode = 3;}
	if (CurrentPage == 6) {mode = 4;}

	while (mode == 0) {     // page 1 tapi nggak nyala/stop
		nexLoop(nex_listen_list);
		oxygenUpdate();

		if (CurrentPage != 1) {break;}
		if (state == 1) {break;}
	}

	while (mode == 1) {   // page 1 tapi nyala

		// // Set Servos ------
		// if(prev_Vti != Vti){
		//  setServoPEEP(Vti);
		//  prev_Vti = Vti;
		// }
		// if(prev_Ox != Ox){
		//  setServoOx(Ox);
		//  prev_Ox = Ox;
		// }

		nexLoop(nex_listen_list);

		// PEEP and IPP Check ----
		if(readPEEP) {
			PEEPUpdate();
			pip_value = 0;
			readPEEP = false;
			digitalWrite(triggerInhalePin, HIGH); //reset trigger inhale
			exhaleStage = false;
		}
		if (readIPP) {
			IPPUpdate();
			exhaleStage = true;
			readIPP = false;
		}
		pressureUpdate1();
		oxygenUpdate();
		if(exhaleStage) {
      Serial.println("EXHALING");
			//0. Cek Fighting
			if(fighting()) {
				digitalWrite(triggerInhalePin,LOW);
        setAlarm("02_HIGH"); mode = 5;
			}

			//1. PEEP Pressure HOLD (CPAP)
			if(pressure_float < PEEP_LIMIT && spuriousDetected){
				digitalWrite(warningPEEP_PIN, LOW);
        setAlarm("04_HIGH");
			} else {
				digitalWrite(warningPEEP_PIN, HIGH);
			}

		} else {  //when exhaleStage == false, or in other word, between PEEP to IPP, or in simple, when inhalation
			Serial.println("INHALING");
			if (pressure_float > PIP_LIMIT) { // warning pressure to nano through digital pin
				Serial.println("WARN!!");
				digitalWrite(warningPressure_PIN,LOW);
				delay(1);
				digitalWrite(warningPressure_PIN,HIGH);
        setAlarm("00_HIGH");
        mode = 5;
			}
		}
    nexLoop(nex_listen_list);
		if (CurrentPage != 1) {break;}
		if (state == 0) {break;}
	}

	while (mode == 3) {   // page 4
		nexLoop(nex_listen_list);
//    dbSerialPrintln(mode);
//    Serial.print(IE);Serial.print("\t");Serial.print(RR);Serial.print("\t");
//    Serial.print(PEEP);Serial.print("\t");Serial.print(Ox);Serial.print("\t");
//    Serial.println(Vti);
		if (CurrentPage != 4) {break;}
	}

	while (mode == 4) {   // mode nerima bacaan pengaturan trigger (belum disetel)
		nexLoop(nex_listen_list);
//    dbSerialPrintln(mode);
		if (CurrentPage != 6) {break;}
	}

  while (mode == 5) {   // mode utk routine stuck setelah alarm
    state = 0;
    if (digitalRead(ButtonResetAlarm_PIN == LOW)) {
      mode = 0;
      setAlarm("99_LOW"); // Reset and send all alarms[] off
    }
//    Serial.println("!!! ALARM HIGH. MACHINE STOPPED. FIX THE SETUP THEN PRESS ALARM RESET BUTTON !!!"); Serial.fulsh();
  }
}


//== FUNCTIONS =============================================

//-- Interrupts
void readPEEPQ(){
	readPEEP = true;
}
void readIPPQ(){
	readIPP = true;
}

//-- Sending necessary information to Arduino Nano ---------
//-- (motor controller) through Serial2 port ---------------
void update2Nano() {
	String message = '<' + String(state) + ','
	                 + String(Vti) + ','
	                 + String(ERat) + ','
	                 + String(RR) + '>';
	Serial1.print(message); Serial1.flush();
  Serial.println(message);
	String message2 = '<' + String(state) + ','
						    + String(Vti) + '>';
	SerialFl.print(message2); SerialFl.flush();

// Debug message
//  Serial.print(F("Message sent to Nano:\n\t")); Serial.print(message); Serial.flush();
}

//-- Float type mapper, for linear regression calibration --
float mapFloat(float rawX, float rawA, float rawB, float realA, float realB) {
	float realX = ( (realB - realA) * float((rawX - rawA) / (rawB - rawA)) ) + realA;
	return realX;
}

//-- Nextion button callback -------------------------------
void b8PushCallback(void *ptr) {  // Press event for button b8
	IE++;
	if(IE>=60)
	{IE=60;}
	ERat = float(IE)/10;
	update2Nano();
}

void b7PushCallback(void *ptr) {  // Press event for button b7
	IE--;
	if(IE==4294967295) {IE=0;}
	ERat = IE/10;
	update2Nano();
}

void b10PushCallback(void *ptr) {  // Press event for button b10
	RR++;
	if(RR>=60)
	{RR=60;}
	update2Nano();
}

void b9PushCallback(void *ptr) {  // Press event for button b9
	RR--;
	if(RR<=10)
	{RR=10;}
	update2Nano();
}

void b14PushCallback(void *ptr) {  // Press event for button b14
	PEEP++;
	if(PEEP>=80)
	{PEEP=80;}
	update2Nano();
}

void b13PushCallback(void *ptr) {  // Press event for button b13
	PEEP--;
	if(PEEP==4294967295)
	{PEEP=0;}
	update2Nano();
}

void b16PushCallback(void *ptr) {  // Press event for button b16
	Vti=Vti+10;
	if(Vti>=800)
	{Vti=800;}
	update2Nano();
}

void b15PushCallback(void *ptr) {  // Press event for button b15
	Vti=Vti-10;
	if(Vti<=100)
	{Vti=100;}
	update2Nano();
}

void b12PushCallback(void *ptr) {  // Press event for button b12
	Ox++;
	if(Ox>=100)
	{Ox=100;}
//  Serial.println(Ox);
}

void b11PushCallback(void *ptr) {  // Press event for button b11
	Ox--;
	if(Ox<=20)
	{Ox=20;}
//  Serial.println(Ox);
}

void bt0PushCallback(void *ptr) {
//uint32_t state;
	bt0.getValue(&state);
	dbSerialPrintln(state);
	update2Nano();
}

void page0PushCallback(void *ptr) {
	CurrentPage = 0;
//  dbSerialPrintln(CurrentPage);
}

void page1PushCallback(void *ptr){
	CurrentPage = 1;
//  dbSerialPrintln(CurrentPage);
}

void page2PushCallback(void *ptr) {
	CurrentPage = 2;
//  dbSerialPrintln(CurrentPage);
}

void page4PushCallback(void *ptr) {
	CurrentPage = 4;
//  dbSerialPrintln(CurrentPage);
}

void page6PushCallback(void *ptr) {
	CurrentPage = 6;
//  dbSerialPrintln(CurrentPage);
}

//-- MPX5010DP pressure sensor ----------------
float calcDatasheetPressure(int x_adc) {
/* Function to return differential pressure value (in cmH2O)
 * from raw ADC value by
 * following typical characteristic in the datasheet
 * ( http://contrails.free.fr/temp/MPX5010.pdf )
 */
	const int abr = 1023;   // ADC bit range
	const int avr = 5;   // ADC voltage range (in volt)
	const float dvo = 0.2;   // Datasheet voltage offset (in volt)
	const float dpg = 2222.22;   // Datasheet pressure gradient (in pascal/volt)
	const float pa_cmh2o = 0.0102;   // 1 Pa = 0.0102 cmH2O
	float pds = (((float(x_adc) / abr * avr) - dvo) * dpg);   // Pressure based on datasheet (in pascal)
	float pds_cmh2o = pds * pa_cmh2o;   // Pressure (in cmH2O)
	return 0.1095*x_adc-4.6591;
}

void pressureUpdate() {
// Read sensor output
	pressure_float = 0; //0.3135*ads.readADC_SingleEnded(3)-1163.1143-153.43;
	pressure_int8 = map(int(pressure_float), -10, 20, 0, 255);

// Update to Nextion waveform graph
	Serial2.print("add 1,0,");
	Serial2.print(pressure_int8 - 80);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

double PIP_raw, PIP_float;
double IPP_raw, IPP_float;
double PEEP_raw, PEEP_float;

void pressureUpdate1() {
	pressure_float = 0; //0.3135*ads.readADC_SingleEnded(3)-1163.1143-153.43;
	pressure_int8 = map(int(pressure_float), -10, 20, 0, 255);
	if(pip_value < pressure_float) {
		pip_value = pressure_float;
	}

// Update to Nextion waveform graph
	Serial2.print("add 1,0,");
	Serial2.print(pressure_int8 - 80);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

void PEEPUpdate() {
//  PEEP_raw = 0.3135*ads.readADC_SingleEnded(0)-1163.1143-153.43;
	pressure_float = 0; //0.3135*ads.readADC_SingleEnded(3)-1163.1143-153.43;

	Serial2.print("n11.val=");
	Serial2.print(round(pressure_float));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial2.print("n10.val=");
	Serial2.print(round(IPP_float));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial2.print("n9.val=");
	Serial2.print(round(pip_value));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

void IPPUpdate() {
//  IPP_raw = analogRead(PIN_MPX5010DP_pressure);
	IPP_float = 0; //0.3135*ads.readADC_SingleEnded(3)-1163.1143-153.43;
	pressure_float = IPP_float;
}

void oxygenUpdate() {
	oxygen_raw = ads.readADC_Differential_0_1();
	volts = (oxygen_raw * scalefactor);
	oxygen_float = 10*volts/6;
	int oxygen_int = int(oxygen_float);
	if (oxygen_int==0) oxygen_int = 100;

///update///
	Serial2.print("n7.val=");
	Serial2.print(oxygen_int);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
////////////
}

//-- Check if patient fight (spurious breath)
bool fighting(){
	bool fight = false;
	// if(pressure_float < 0 && flow_float > 0){
	//  fight = true;
	// }
	return fight;
}

//-- Send alarm mode to buzzer/alarm microcontroller ----
void setAlarm(String key) {   // Key example: 01_ON   ;   09_OFF
  //Define array of alarm triggers
  //alarmzz[0] = High pressure exceeded PIP (HIGH)
  //alarmzz[1] = Pressure too low (HIGH)
  //alarmzz[2] = Patient is fighting (HIGH)
  //alarmzz[3] = Overcurrent fault (HIGH)
  //alarmzz[4] = Sporious breath (MEDIUM)
  //alarmzz[5] = Overtidal volume (MEDIUM)
  //alarmzz[6] = Low PEEP (MEDIUM)
  //alarmzz[7] =
  //alarmzz[8] = Low/Oversupply of Oxygen (LOW)

  int key_index = key.substring(0,2).toInt();
//  Serial.println(key_index);  // debugging
  if (key.substring(3) == "ON") {alarms[key_index] = 1;}
  else if (key_index == 99) {
    for (int j = 0; j<9; j++) {alarms[j] = 0;}
  }
  else {alarms[key_index] = 0;}

  String msg = "<";
  msg = msg + String(alarms[0]);
  for(int i = 1;i<9;i++) {
    msg = msg + "," + String(alarms[i]);
  }
  msg = msg + ">";

  Serial3.println(msg); Serial3.flush();
  Serial.println(msg);
}

//-- Servo to set Oxygen
void setServoOx(uint32_t Oxq){
	int sudut = map(Oxq, 0, 100, 0, 65);

	digitalWrite(enaServoOx, HIGH);
	servoOxigen.write(sudut);
	digitalWrite(enaServoOx, LOW);
}

//-- Servo to set PEEP opening
void setServoPEEP(uint32_t Vol){
	int sudut = round(cariBukaanPEEP(Vol));

	digitalWrite(enaServoPEEP, HIGH);
	servoPEEP.write(sudut);
	digitalWrite(enaServoPEEP, LOW);
}

//-- Lookup Table for Servo PEEP vs VOLUME
float cariBukaanPEEP(float vol_Tidal){
	float lookup_vol[] = {223.75, 289.53, 355.72, 410.89, 475.67, 550.83, 606.44, 653.11, 704.17, 748.33, 771.95};
	float lookup_step[] = {450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950};

	float stepTidal = 0;
	int arraySize = sizeof(lookup_vol) / sizeof(lookup_vol[0]);

	// Extrapolasi Bawah
	if(vol_Tidal < cariMin(lookup_vol, arraySize)) {
		float m = float(lookup_step[1] - lookup_step[0]) / (lookup_vol[1] - lookup_vol[0]);
		float c = float(lookup_step[0]) - lookup_vol[0] * m;
		stepTidal = m * vol_Tidal + c;
	}
	// Extrapolasi Atas
	else if(vol_Tidal > cariMax(lookup_vol, arraySize)) {
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
	for(int i=0; i< arraySize; i++) {
		if(list[i] <= mini) {
			mini = list[i];
		}
	}
	return mini;
}

float cariMax(float list[], int arraySize){
	float maxi = 0;
	for(int i=0; i< arraySize; i++) {
		if(list[i] >= maxi) {
			maxi = list[i];
		}
	}
	return maxi;
}
