#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"

#define relay 	 	6
#define TFT_RST 	7
#define TFT_CS		8
#define TFT_DC		9
#define MAX_CS  	10

#define textsize 		3
#define screenwidth 	320
#define screenheight 	240

#define temperatureY 		1*8*textsize
#define dutyY				2*8*textsize
#define valuesX				8*5*textsize
#define unitsX				18*5*textsize

#define graphmax 	165


Adafruit_MAX31855 thermocouple(MAX_CS);    									// hardware SPI
Adafruit_ILI9340 tft = Adafruit_ILI9340(TFT_CS, TFT_DC, TFT_RST);			// hardware SPI

PID heatPID(&input,&output,&setpoint,Kp,Ki,Kd,DIRECT);
PID_ATune aTune(&input, &output);

double setpoint,input,output,prevsetpoint;									// PID parameters
double Kp = 3566.68;
double Ki = 132.23;
double Kd = 24051.92;

unsigned int prevtime = 0;

unsigned int xpos = 0;														// graphing parameters
int graphoffset;
int scaletemp = 240;
byte buffer[screenwidth+2];
boolean bufferfull = 0;
boolean monitormode = 0;
boolean fastmode = 0;
boolean forceshowtemp = 0;
boolean unitSI = 1;
boolean displaygraph = 1;

double aTuneStep=33265, aTuneNoise=2, aTuneStartValue=33265;				// autotune parameters
unsigned int aTuneLookBack=25;
byte ATuneModeRemember;
boolean tuning = 0;

void callback();
void buildtable();
void cleartft();

void AutoTuneHelper(boolean start);
void ChangeAutoTune();
void SerialSend();

int GetYVal(double input);
float fahrenheit(double celsius);

void setup() {
	pinMode(relay,OUTPUT);
	digitalWrite(relay,LOW);			// ensure relay is off
	
	tft.begin();						// initialize TFT display
	tft.setRotation(3);
	tft.fillScreen(ILI9340_BLACK);
	tft.setTextSize(textsize);
	
	Serial.begin(9600);
	delay(500); 						// wait for MAX31855 to stabilize
	
	setpoint = 0;						// temperature in celsius

	heatPID.SetSampleTime(4194);		// match PID calculation with timer1 frequency
	heatPID.SetOutputLimits(0,65535);	// limit PID output to be within limits of TCNT1
	heatPID.SetMode(AUTOMATIC);
	
	TCCR1A = 0;
	TCCR1B =  1<<CS12 | 1<<CS10;		// prescale clock freq by 1024 	-- 4194.2ms period
//	TCCR1B =  1<<CS12;					// prescale clock freq by 256 	-- 1048.6ms period 	-- too short for relay
	
	OCR1B = 0;							// set compare value to zero 	-- 0% duty cycle

	TIMSK1 |=  1<<TOIE1;				// enable timer1 overflow interrupt
	TIMSK1 |=  1<<OCIE1B;				// enable timer1 output compare register B interrupt
	
	graphoffset = (int)thermocouple.readInternal() - 5;		//set minimum value to be 5degC less than MAX31855 cold junction
	
	buildtable();						// initialize static text display	
}

SIGNAL(TIMER1_OVF_vect) {;				// timer1 overflow interrupt: turn on relay
	PORTD |= (1<<PIND6);
}
SIGNAL(TIMER1_COMPB_vect) {				// timer1 output compare interrupt B: turn off relay
	PORTD &= ~(1<<PIND6);
}

void loop() {
	unsigned int currenttime = millis();					// record loop execution start time to determine loop duration
	int setyval;
	byte tuneval = 0;
	input = thermocouple.readCelsius();						// read thermocouple value into PID input
	heatPID.Compute();										// run PID computation
	if((PORTD >> PIND6) & 0x01) {
		if(output < TCNT1) PORTD &= ~(1<<PIND6);			// if new output is less than current counter position AND relay is currently on (TCNT1 < old output): turn off relay
	} else {
		 if(output > TCNT1) PORTD |= (1<<PIND6);			// if new output is greater t0han current counter position AND relay is currently off (TCNT1 > old output): turn on relay
	}
	OCR1B = output;											// set the new output value to the compare register
	if(tuning) {	
		tuneval = aTune.Runtime();		// if tuning, run autotune computation
		SerialSend();					// print tuning information to serial
		if (tuneval != 0) {				// if tuning is complete
			tuning = false;
			Kp = aTune.GetKp();
			Ki = aTune.GetKi();
			Kd = aTune.GetKd();
			heatPID.SetTunings(Kp,Ki,Kd);
			Serial.print(F("Kp:"));
			Serial.print(Kp);
			Serial.print(F(" Ki:"));
			Serial.print(Ki);
			Serial.print(F(" Kd:"));
			Serial.print(Kd);
			AutoTuneHelper(false);
		}           
	} else {							// if not tuning
		if(Serial.available()) {		// check serial data for commands
			char command = Serial.read();
			if(command == 'S') {													// set setpoint
				float newsetpoint = Serial.parseFloat();
				setyval = GetYVal(setpoint);
				tft.drawLine(0,setyval,screenwidth,setyval,ILI9340_BLACK);
				setpoint = (double)newsetpoint;
				buildtable();
			} else if(command == 'P') {												// set Kp
				Kp = (double)Serial.parseFloat();
				heatPID.SetTunings(Kp,Ki,Kd);
				SerialSend();
			} else if(command == 'I') {												// set Ki
				Ki = (double)Serial.parseFloat();
				heatPID.SetTunings(Kp,Ki,Kd);
				SerialSend();
			} else if(command == 'D') {												// set Kd
				Kd = (double)Serial.parseFloat();
				heatPID.SetTunings(Kp,Ki,Kd);
				SerialSend();
			} else if(command == 'M') {												// toggle monitor mode
				monitormode ^= 0x01;
				tft.fillScreen(ILI9340_BLACK);
				if(!monitormode) buildtable();
				else {
					if(fastmode) {
						fastmode = 0;
						xpos = 0;
						bufferfull = 0;
					}
				}
				forceshowtemp = 0;
			} else if(command == 'T') ChangeAutoTune();								// enable autotune
			 else if(command == 'C') {												// set graph scale
				scaletemp = Serial.parseInt();			
				if(monitormode || fastmode) tft.fillScreen(ILI9340_BLACK);
				else cleartft();
			} else if(command == 'O') {												// set graph offset
				graphoffset = Serial.parseInt();			
				if(monitormode || fastmode) tft.fillScreen(ILI9340_BLACK);
				else cleartft();	
			} else if(command == 'B')	cleartft();									// clear graph buffer
			else if(command == 'F') {												// toggle fast mode
				fastmode ^= 0x01;
				xpos = 0;
				tft.fillScreen(ILI9340_BLACK);
				if(!fastmode) {
					buildtable();
					bufferfull = 0;
					xpos = 0;
				}
			} else if(command == 'E') {												// show temp in monitor or fast mode
				forceshowtemp ^= 0x01;
				if(forceshowtemp) {
					tft.setTextColor(ILI9340_WHITE,ILI9340_BLACK); 
					tft.setCursor(0, 1*8*textsize);
					tft.print(F("T:"));
					tft.setCursor(unitsX, temperatureY);
					tft.setTextColor(ILI9340_RED,ILI9340_BLACK);  
					tft.print(String((char)247)+"C");
				} else tft.fillRect(0,0,screenwidth,screenheight-graphmax,ILI9340_BLACK);
			} else if(command == 'L') tft.fillScreen(ILI9340_BLACK);				// clear (refresh) screen
			else if(command == 'U') {												// toggle units
				unitSI ^= 0x01;
				buildtable();
			} else if(command == 'A') displaygraph ^= 0x01;							// toggle graphing
			else if(command == 'X') SerialSend();									// show PID tunings
            else if(command == 'R') output = Serial.parseInt();						// force OCR1B value
			Serial.read();															// clear extra character from buffer if present
		}
		if(!monitormode && !fastmode) {												// if not monitor mode or fast mode, display dynamic text
			tft.setCursor(valuesX, temperatureY);
			tft.setTextColor(ILI9340_RED,ILI9340_BLACK);  
			if(unitSI) tft.println(input);
			else tft.println(fahrenheit(input));
			tft.setCursor(valuesX, dutyY);
			tft.setTextColor(0x0400,ILI9340_BLACK);
			tft.print(100*output/65535);
		}
		if(forceshowtemp) {															// if command 'E' is enabled force show temperature
			tft.setCursor(valuesX, temperatureY);
			tft.setTextColor(ILI9340_RED,ILI9340_BLACK);  
			tft.println(input);
		}
		
		//tft.drawLine(0,screenheight-graphmax,screenwidth,screenheight-graphmax,ILI9340_YELLOW);

		Serial.print(F("setpoint:"));
		Serial.print(setpoint);
		Serial.print(F("; input:"));
		Serial.print(input);
		Serial.print(F("; output:"));
		Serial.print(output);
		Serial.print(F("; duty:"));
		Serial.print(100*output/65535);
		Serial.print(F("%; dt:"));
		Serial.print(currenttime-prevtime);
		Serial.println(F("ms"));
		
		if(displaygraph) {																				// if graphing is enabled
			setyval = GetYVal(setpoint);
			tft.drawLine(0,setyval,screenwidth,setyval,ILI9340_BLUE);
			int yval = GetYVal(input);
			
			if(!fastmode) {		// if fast mode enabled
				if(bufferfull == 0) {
					for(int i=0;i<screenwidth*2;i++) tft.drawPixel(xpos,yval,ILI9340_WHITE);			// replicate the same delay as when the buffer is full
					buffer[xpos] = yval;
				}
				else {
					for(int i=0;i<screenwidth+1;i++) {
						tft.drawPixel(i,buffer[i],ILI9340_BLACK);
						tft.drawPixel(i,buffer[i+1],ILI9340_WHITE);
					}
					buffer[screenwidth+1] = yval;
					for(int i=0;i<screenwidth+1;i++) buffer[i] = buffer[i+1];
				}
				if(bufferfull == 0) {
					if(++xpos > screenwidth) {
						xpos = 0;
						tft.fillRect(0,screenheight/2,10,170,ILI9340_BLACK);
						bufferfull = 1;
					}
				}
			} else {		// fast mode not enabled
				tft.drawPixel(xpos,yval,ILI9340_WHITE);
				tft.fillRect(xpos+1,screenheight-graphmax,10,graphmax,ILI9340_BLACK);
				if(++xpos > screenwidth) {
					xpos = 0;
					tft.drawLine(0,screenheight,0,screenheight-graphmax,ILI9340_BLACK);
				}
			}
		}
		prevtime = currenttime;	
	}

}

void buildtable() {								// (re)initialize static text display	
	String tempstr;
	if(unitSI) tempstr = String((char)247)+"C";
	else tempstr = String((char)247)+"F";
	
	tft.setCursor(0, 0);
	tft.setTextColor(ILI9340_WHITE,ILI9340_BLACK);  
	tft.print(F("S:"));
	tft.setTextColor(0x0410,ILI9340_BLACK);  
	tft.setCursor(valuesX, 0);
	if(unitSI) tft.print(setpoint);
	else tft.print(fahrenheit(setpoint));
	tft.setCursor(unitsX, 0);
	tft.print(tempstr);
	
	tft.setTextColor(ILI9340_WHITE,ILI9340_BLACK); 
	tft.setCursor(0, 1*8*textsize);
	tft.print(F("T:"));
	tft.setCursor(unitsX, temperatureY);
	tft.setTextColor(ILI9340_RED,ILI9340_BLACK);  
	tft.print(tempstr);
	
	tft.setCursor(0, 2*8*textsize);
	tft.setTextColor(ILI9340_WHITE,ILI9340_BLACK);
	tft.print(F("duty:"));
	tft.setCursor(unitsX, dutyY);
	tft.setTextColor(0x0400,ILI9340_BLACK);
	tft.print(F("%"));
}

float fahrenheit(double celsius) {					// return celsius to fahrenheit
	return (celsius*9/5)+32;
}

void cleartft() {									// refresh TFT, redraw static text, clear buffer
	tft.fillScreen(ILI9340_BLACK);
	buildtable();
	bufferfull = 0;
	xpos = 0;
}

int GetYVal(double input) {							// return TFT y value at temperature for given scale and offset
	return screenheight-(int)(graphmax*((input-graphoffset)/(scaletemp-graphoffset)));
}

void AutoTuneHelper(boolean start) {
	if(start) ATuneModeRemember = heatPID.GetMode();
	else heatPID.SetMode(ATuneModeRemember);
}

void ChangeAutoTune() {
	if(!tuning) {
		output = aTuneStartValue;					// set the output to the desired starting frequency.
		aTune.SetNoiseBand(aTuneNoise);				// set autotune noise value
		aTune.SetOutputStep(aTuneStep);				// set autotune maximum step from start frequency
		aTune.SetControlType(1);					// set autotune for PID (as opposed to PI)
		aTune.SetLookbackSec((int)aTuneLookBack);	// set window for peak detection period
		AutoTuneHelper(true);						// remember current PID mode
		tuning = true;								// enable tuning
	} else {
		aTune.Cancel();								// cancel autotune
		tuning = false;								// disable tuning
		AutoTuneHelper(false);						// restore PID mode
	}
}

void SerialSend() {
	char space = ' ';
	Serial.print(F("setpoint: "));Serial.print(setpoint); Serial.print(space);
	Serial.print(F("input: "));Serial.print(input); Serial.print(space);
	Serial.print(F("output: "));Serial.print(output); Serial.print(space);
	if(tuning)Serial.println(F("tuning mode"));
	else {
		Serial.print(F("Kp: "));Serial.print(heatPID.GetKp());Serial.print(space);
		Serial.print(F("Ki: "));Serial.print(heatPID.GetKi());Serial.print(space);
		Serial.print(F("Kd: "));Serial.print(heatPID.GetKd());Serial.println();
	}
}