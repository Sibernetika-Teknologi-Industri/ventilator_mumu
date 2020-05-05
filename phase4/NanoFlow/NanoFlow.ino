#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

SoftwareSerial SerialM(11,12);
Adafruit_ADS1115 ads;

// PIN LIST
#define pinPEEP 2
#define pinIPP 3

#define pinVolWarn 6
#define pinSpur 7
#define pinFight 8

// GLOBAL VARIABLES
float flow_raw, flow_val;
int vol_lim;
unsigned long now;
double vol_acc = 0;

bool readPEEP = false;
bool readIPP = false;
bool exhaleStage = false;
bool updated = false;
String bufferq[2];
String lastData = "<0, 0>";
bool statusON;

unsigned long nowq = 0;
unsigned long dt = 0;
bool lastState = 0; //0 Inhale, 1 Exhale

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialM.begin(57600);

  ads.begin();
  ads.setGain(GAIN_SIXTEEN);

  pinMode(pinPEEP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinPEEP), readPEEPQ, FALLING);
  pinMode(pinIPP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinIPP), readIPPQ, FALLING);

  nowq = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  readDataFromMega();
  statusON = 1;
  vol_lim = 300;

  flow_raw = ads.readADC_Differential_0_1();
  flow_val = calcFlow(flow_raw);

   if(abs(flow_val) <= 1.7 || abs(roundf(flow_val*100.0)/100.0) == 2.39 || abs(roundf(flow_val*100.0)/100.0) == 3.20 || abs(roundf(flow_val*100.0)/100.0) == 4.01){flow_val=0;}
///  //Serial.println(flow_val);
  if(statusON){
    if(readPEEP){
      //reset stuffs
      digitalWrite(pinSpur, HIGH);
      digitalWrite(pinFight, HIGH);
      vol_acc = 0;

      // indicate INHALE
      exhaleStage = false;

      // reset time and var
      nowq = micros();
      readPEEP = false;
    }

    if(readIPP){
      exhaleStage = true;
      readIPP = false;
    }

    if(exhaleStage){ //fasa exhale
      if(lastState == 0){
//        //Serial.println("EXHALE STAGE");
        lastState = 1;
      }

      //0. Cek Spurious
      if(spuriousDetect()){
        digitalWrite(pinSpur, LOW);
      }
    } else { //fasa inhale
      if(lastState == 1){
//        //Serial.println("INHALE STAGE");
        lastState = 0;
      }
      //0. Cek Fighting
      if(fightingDetect()){
        digitalWrite(pinFight,LOW);
      }

      //1. Jaga Volume
      dt = micros()-nowq;
      vol_acc += flow_val/60 * dt/1000;
      nowq = micros();

      // ambil data
      Serial.print(flow_raw);
      Serial.print("\t");
      Serial.print(flow_val);
      Serial.print("\t");
      Serial.println(vol_acc);
//      delay(9); // agar pas 500 an data
      
//      Serial.println("====> TIME: " + String(dt));
      //Serial.println("VOL: " + String(vol_acc));
      if(vol_acc> vol_lim){
//        //Serial.println("WARNING VOLUME");
        digitalWrite(pinVolWarn, LOW);
        delayMicroseconds(10);
        digitalWrite(pinVolWarn, HIGH);
      }
    }
  }
}

//== FUNCTIONS ------------------------------------------------------

//- Interrupts
void readPEEPQ(){readPEEP = true;}
void readIPPQ(){readIPP = true;}

//- Calc Flow from Callibration
float calcFlow(float flow_rawq){
  float calc = (90.1479*sqrt(flow_rawq)-4852.4818-141.58 + 0.405 +2.85 - 9 + 43.38- 25.42-2.44-20.30);
  
  return calc;
}

//- Detect Spurious
bool spuriousDetect(){
  bool spurious = false;
  // if(){
  //   spurious = true;
  // }
  return spurious;
}

//- Detect Fighting
bool fightingDetect(){
  bool fight = false;
  // if(){
  //   spurious = true;
  // }
  return fight;
}


//- From/to Mega
void readDataFromMega(){
  String received = listeningMega();

//  //Serial.println(updated);
	if(updated == false) {
		//Serial.print("Received: ");
		//Serial.println(received);
		Serial.flush();

		int indexStart = 0;
		int indexEnd = 0;
    //Serial.println("CHECK");Serial.flush();
		for(int i = 0; i<2; i++) {
      //Serial.println("CHECK" + String(i));Serial.flush();
			indexEnd = received.indexOf(",", indexStart);
			bufferq[i] = received.substring(indexStart, indexEnd);
			indexStart = indexEnd+1;
			//    //Serial.println(String(i) + ": " + bufferq[i]);
		}

		statusON = bufferq[0].toInt();
		vol_lim = bufferq[1].toInt();

		updated = true;
   //Serial.println("UPDATED");Serial.flush();
	}
}

String listeningMega(){
	bool quit = false;
	String seriesData = "";

	while (!quit) {
		if (SerialM.available() > 0) {
			updated = false;
			char x = SerialM.read();
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
	lastData = seriesData;

	//!! Dummy Data !!
	// seriesData = "<1, 300>";

//  String seriesData2 = ;

	return seriesData.substring(1,seriesData.length()-1);
}
