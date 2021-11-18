#include "grbl.h"
#include "serial.h"
//#include <Arduino.h>

//#define FA_PIN_ENABLE		8

//#define FA_PIN_JOY_X		A5
//#define FA_PIN_JOY_Y		A4
//#define FA_PIN_JOY_Z		A1


bool mSpindleRunning = false;
bool mSpindleButtonSwitchBusy = false;


#define FA_HIGH 0x1
#define FA_LOW  0x0

#define FA_INPUT 0x0
#define FA_OUTPUT 0x1
#define FA_INPUT_PULLUP 0x2

#define FA_NOT_A_PIN		0
#define FA_NOT_A_PORT		0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


int mJoyCenters[3];

typedef struct 
{
	uint8_t mPinNumber;
	volatile uint8_t *mPin;
	volatile uint8_t *mPort;
	uint8_t mBit;
	volatile uint8_t *mDdr;
//	volatile uint8_t *mOut;
//	volatile uint8_t *mIn;
}FA_PIN;

FA_PIN FA_PIN_A0;
FA_PIN FA_PIN_A1;
FA_PIN FA_PIN_A2;
FA_PIN FA_PIN_A3;
FA_PIN FA_PIN_A4;
FA_PIN FA_PIN_A5;

FA_PIN FA_PIN_D2;
FA_PIN FA_PIN_D3;
FA_PIN FA_PIN_D4;
FA_PIN FA_PIN_D5;
FA_PIN FA_PIN_D6;
FA_PIN FA_PIN_D7;
FA_PIN FA_PIN_D8;
FA_PIN FA_PIN_D9;
FA_PIN FA_PIN_D10;
FA_PIN FA_PIN_D11;
FA_PIN FA_PIN_D12;


typedef struct 
{
	FA_PIN *mPinDir;
	FA_PIN *mPinStep;
	FA_PIN *mPinLimit;

	int mPulse;
	volatile int mSpeed;
	bool mds;
	int mLimitHit;

}FA_STEPPER;


FA_STEPPER stepper_x = {
	.mPinDir = &FA_PIN_D5,
	.mPinStep = &FA_PIN_D2,
	.mPinLimit = &FA_PIN_D9,
	.mPulse = 0,
	.mSpeed = 0,
	.mds = 0,
	.mLimitHit = 0
};
FA_STEPPER stepper_y = {
	.mPinDir = &FA_PIN_D6,
	.mPinStep = &FA_PIN_D3,
	.mPinLimit = &FA_PIN_D10,
	.mPulse = 0,
	.mSpeed = 0,
	.mds = 0,
	.mLimitHit = 0
};
FA_STEPPER stepper_z = {
	.mPinDir = &FA_PIN_D7,
	.mPinStep = &FA_PIN_D4,
	.mPinLimit = &FA_PIN_D12,
	.mPulse = 0,
	.mSpeed = 0,
	.mds = 0,
	.mLimitHit = 0
};



#define FA_PIN_SWMN 		FA_PIN_A0
#define FA_PIN_ENABLE		FA_PIN_D8
#define FA_PIN_SPINDLE		FA_PIN_D11

#define FA_PIN_JOY_X		FA_PIN_A5
#define FA_PIN_JOY_Y		FA_PIN_A4
#define FA_PIN_JOY_Z		FA_PIN_A1
#define FA_PIN_JOY_XY_SW	FA_PIN_A3


FA_STEPPER *steppers[] = {&stepper_x,&stepper_y,&stepper_z};
/*
const uint16_t PROGMEM port_to_mode_PGM[] = {
    FA_NOT_A_PORT,
    FA_NOT_A_PORT,
    (uint16_t) &DDRB,
    (uint16_t) &DDRC,
    (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
    FA_NOT_A_PORT,
    FA_NOT_A_PORT,
    (uint16_t) &PORTB,
    (uint16_t) &PORTC,
    (uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
    FA_NOT_A_PORT,
    FA_NOT_A_PORT,
    (uint16_t) &PINB,
    (uint16_t) &PINC,
    (uint16_t) &PIND,
};
*/


int readAxis(int thisAxis, int delta, int vmin, int vmax);
void setupTimer2();
bool setSpeed(int ax, int speed);
void famc_loop ();
long fa_map(long x, long in_min, long in_max, long out_min, long out_max);
void faPinMode(FA_PIN *pin, uint8_t mode);
//bool pinRead(volatile uint8_t *pinIn, uint8_t pinBit);
bool pinRead(FA_PIN *pin);
void initPin(FA_PIN *pin, uint8_t pinNumber);
int faAnalogRead(uint8_t pin);
void faDigitalWrite(FA_PIN *pin, bool High);

/*
#define fa_portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define fa_portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define fa_portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )
*/

void famc_setup ()
{
	serial_init();
	//printString("123");

	initPin(&FA_PIN_A0, 14);
/*
	FA_PIN_SWMN.mPin = &PINC;
	FA_PIN_SWMN.mPort = &PORTC;
	FA_PIN_SWMN.mDdr = &DDRC;
	FA_PIN_SWMN.mBit = 3;
*/	


	
/*
	*FA_PIN_SWMN.mDdr &= ~(1<<FA_PIN_SWMN.mBit);
	*FA_PIN_SWMN.mPort |= (1<<FA_PIN_SWMN.mBit);
*/
	faPinMode(&FA_PIN_SWMN, FA_INPUT_PULLUP);

//	if(PINC & (1<<FA_PIN_SWMN.mBit))
//	if(*FA_PIN_SWMN.mPin & (1<<FA_PIN_SWMN.mBit))
	if(pinRead(&FA_PIN_SWMN))
	{
		return;
	}
	
	/*
	if(pinRead(FA_PIN_SWMN.mIn, FA_PIN_SWMN.mBit))
	{
		mManualControlEnabled = false;
		return;
	}
	*/

	//initPin(&FA_PIN_A0, 14);
	initPin(&FA_PIN_A1, 15);
	initPin(&FA_PIN_A2, 16);
	initPin(&FA_PIN_A3, 17);
	initPin(&FA_PIN_A4, 18);
	initPin(&FA_PIN_A5, 19);

	initPin(&FA_PIN_D2, 2);
	initPin(&FA_PIN_D3, 3);
	initPin(&FA_PIN_D4, 4);
	initPin(&FA_PIN_D5, 5);
	initPin(&FA_PIN_D6, 6);
	initPin(&FA_PIN_D7, 7);
	initPin(&FA_PIN_D8, 8);
	initPin(&FA_PIN_D9, 9);
	initPin(&FA_PIN_D10, 10);
	initPin(&FA_PIN_D11, 11);
	initPin(&FA_PIN_D12, 12);


	// initialize ADC for analog ports
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
	sbi(ADCSRA, ADEN);


	for(int i=0;i<3;i++)
	{
		faPinMode(steppers[i]->mPinDir, FA_OUTPUT);
		faPinMode(steppers[i]->mPinStep, FA_OUTPUT);
		faPinMode(steppers[i]->mPinLimit, FA_INPUT_PULLUP);
	}

	faPinMode(&FA_PIN_ENABLE, FA_OUTPUT);
	faDigitalWrite(&FA_PIN_ENABLE, FA_HIGH);

	faPinMode(&FA_PIN_JOY_XY_SW, FA_INPUT_PULLUP);
	faPinMode(&FA_PIN_SPINDLE, FA_OUTPUT);

	faDigitalWrite(&FA_PIN_SPINDLE, FA_LOW);
	mSpindleRunning = false;

	mJoyCenters[0] = faAnalogRead(FA_PIN_JOY_X.mPinNumber);
	mJoyCenters[1] = faAnalogRead(FA_PIN_JOY_Y.mPinNumber);
	mJoyCenters[2] = faAnalogRead(FA_PIN_JOY_Z.mPinNumber);

	setupTimer2();

/*
	for(int i=0;i<3;i++)
	{
		pinMode(steppers[i]->mPinDir.mPin, OUTPUT);
		pinMode(steppers[i]->mPinStep.mPin, OUTPUT);
		pinMode(steppers[i]->mPinLimit.mPin, INPUT_PULLUP);

		steppers[i]->mPinDir.mPort = digitalPinToPort(steppers[i]->mPinDir.mPin);
		steppers[i]->mPinDir.mBit = digitalPinToBitMask(steppers[i]->mPinDir.mPin);
		steppers[i]->mPinDir.mOut = portOutputRegister(steppers[i]->mPinDir.mPort);
		steppers[i]->mPinDir.mIn = portInputRegister(steppers[i]->mPinDir.mPort);

		steppers[i]->mPinStep.mPort = digitalPinToPort(steppers[i]->mPinStep.mPin);
		steppers[i]->mPinStep.mBit = digitalPinToBitMask(steppers[i]->mPinStep.mPin);
		steppers[i]->mPinStep.mOut = portOutputRegister(steppers[i]->mPinStep.mPort);
		steppers[i]->mPinStep.mIn = portInputRegister(steppers[i]->mPinStep.mPort);

		steppers[i]->mPinLimit.mPort = digitalPinToPort(steppers[i]->mPinLimit.mPin);
		steppers[i]->mPinLimit.mBit = digitalPinToBitMask(steppers[i]->mPinLimit.mPin);
		steppers[i]->mPinLimit.mOut = portOutputRegister(steppers[i]->mPinLimit.mPort);
		steppers[i]->mPinLimit.mIn = portInputRegister(steppers[i]->mPinLimit.mPort);



	}
	pinMode(FA_PIN_ENABLE, OUTPUT);
	digitalWrite(FA_PIN_ENABLE, HIGH);

	mJoyCenters[0] = analogRead(FA_PIN_JOY_X);
	mJoyCenters[1] = analogRead(FA_PIN_JOY_Y);
	mJoyCenters[2] = analogRead(FA_PIN_JOY_Z);
	

	setupTimer2();
*/

	for(;;) 
	{
		/*
		if(pinRead(&FA_PIN_SWMN))
		{
			printString("M\n");
		}
		else
		{
			printString("-\n");
		}
		*/

		famc_loop();
		//yield();
		//delay(500);
	}

}

void famc_loop () 
{


	int x = readAxis(FA_PIN_JOY_X.mPinNumber, mJoyCenters[0], -10, 10);
	int y = readAxis(FA_PIN_JOY_Y.mPinNumber, mJoyCenters[1], -10, 10);
	int z = readAxis(FA_PIN_JOY_Z.mPinNumber, mJoyCenters[2], -20, 20);
/*
printString("X: ");
printInteger(x);
printString(", Y:");
printInteger(y);
printString(", Z:");
printInteger(z);
printString("\n");
*/
	bool sx = setSpeed(0, x);
	bool sy = setSpeed(1, y);
	bool sz = setSpeed(2, -1*z);

	if(sx || sy || sz)
	{
		faDigitalWrite(&FA_PIN_ENABLE, FA_LOW);
	}
	else
	{
		faDigitalWrite(&FA_PIN_ENABLE, FA_HIGH);
	}
	
/*
	printInteger(faAnalogRead(FA_PIN_JOY_XY_SW.mPinNumber));
	printPgmString(PSTR("\n"));
*/	

	if(pinRead(&FA_PIN_JOY_XY_SW)==FA_LOW)
	{
		if(!mSpindleButtonSwitchBusy)
		{
			mSpindleButtonSwitchBusy = true;
			mSpindleRunning = !mSpindleRunning;
			faDigitalWrite(&FA_PIN_SPINDLE, (mSpindleRunning)?FA_HIGH:FA_LOW);
		}
	}
	else
	{
		mSpindleButtonSwitchBusy = false;
	}
}

int readAxis(int thisAxis, int delta, int vmin, int vmax)
{
	
  	int reading = faAnalogRead(thisAxis);


  	int d = delta-512;

	reading = fa_map(reading-d, 0, 1023, -1100, 1100);
	reading = -1* max(min(reading, 1000), -1000);

	if(reading>vmin && reading<vmax)
		reading = 0;

	return reading;
	
return 0;
}

void faPinMode(FA_PIN *pin, uint8_t mode)
{
	uint8_t bit = pin->mBit;
	//volatile uint8_t *port = pin->mPort;
	//volatile uint8_t *reg, *out;

	//if (*port == FA_NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	//reg = fa_portModeRegister(*port);
	//out = fa_portOutputRegister(*port);

	if (mode == FA_INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*pin->mDdr &= ~(1<<bit);
		*pin->mPort &= ~(1<<bit);
		SREG = oldSREG;
	} else if (mode == FA_INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*pin->mDdr &= ~(1<<bit);
		*pin->mPort |= (1<<bit);
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*pin->mDdr |= (1<<bit);
		SREG = oldSREG;
	}
}

void setupTimer2()
{
//	noInterrupts();
	cli();
	// Clear registers
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;

	// 5000 Hz (16000000/((24+1)*128))
	OCR2A = 24;
	// CTC
	TCCR2A |= (1 << WGM21);
	// Prescaler 128
	TCCR2B |= (1 << CS22) | (1 << CS20);
	// Output Compare Match A Interrupt Enable
	TIMSK2 |= (1 << OCIE2A);
//	interrupts();
	sei();

}

/*
void pinChange(volatile uint8_t *pinOut, uint8_t pinBit, bool High)
{
	if(High == 0)
	{
		*pinOut &= ~pinBit;
	}
	else
	{
		*pinOut |= pinBit;
	}
}
*/
void faDigitalWrite(FA_PIN *pin, bool High)
{
	if(High == 0)
	{
		*pin->mPort &= ~(1 << pin->mBit);
	}
	else
	{
		*pin->mPort |= (1 << pin->mBit);
	}
}
/*
bool pinRead(volatile uint8_t *pinIn, uint8_t pinBit)
{
	return (*pinIn & pinBit);
}
*/
bool pinRead(FA_PIN *pin)
{
	return (*pin->mPin & (1<<pin->mBit));
}

long fa_map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ISR(TIMER2_COMPA_vect) 
{
	for(int i=0;i<3;i++)
	{
		FA_STEPPER *st = steppers[i];
		if(st->mSpeed==0 || (st->mLimitHit>0 && st->mSpeed>0) || (st->mLimitHit<0 && st->mSpeed<0))
		{
			st->mPulse = 0;
			continue;
		}
		if(pinRead(st->mPinLimit)==FA_HIGH)
		//if(pinRead(st->mPinLimit.mIn, st->mPinLimit.mBit)==HIGH)
		{
			st->mLimitHit = 0;
		}
		//pinChange(st->mPinDir.mOut, st->mPinDir.mBit, (st->mSpeed>0)?HIGH:LOW);
		faDigitalWrite(st->mPinDir, (st->mSpeed>0)?FA_HIGH:FA_LOW);
		st->mPulse--;
		if(st->mPulse<=0)
		{
			st->mPulse = (1000-abs(st->mSpeed))/100;
			//pinChange(st->mPinStep.mOut, st->mPinStep.mBit, (st->mds)?HIGH:LOW);
			faDigitalWrite(st->mPinStep, (st->mds)?FA_HIGH:FA_LOW);
			st->mds = !st->mds;
		}
//		if(st->mLimitHit==0 && pinRead(st->mPinLimit.mIn, st->mPinLimit.mBit)==LOW)
		if(st->mLimitHit==0 && pinRead(st->mPinLimit)==FA_LOW)
		{
			st->mLimitHit = (st->mSpeed>0)?2:-2;
		}
	}
}

bool setSpeed(int ax, int speed)
{
	steppers[ax]->mSpeed = speed;
	return (speed!=0);
}


void initPin(FA_PIN *pin, uint8_t pinNumber)
{
	pin->mPinNumber = pinNumber;
	switch (pinNumber)
	{
		case 14:	// A0
			pin->mPin = &PINC;	pin->mPort = &PORTC;	pin->mDdr = &DDRC;		pin->mBit = 0;
			break;
		case 15:	// A1
			pin->mPin = &PINC;	pin->mPort = &PORTC;	pin->mDdr = &DDRC;		pin->mBit = 1;
			break;
		case 16:	// A2
			pin->mPin = &PINC;	pin->mPort = &PORTC;	pin->mDdr = &DDRC;		pin->mBit = 2;
			break;
		case 17:	// A3
			pin->mPin = &PINC;	pin->mPort = &PORTC;	pin->mDdr = &DDRC;		pin->mBit = 3;
			break;
		case 18:	// A4
			pin->mPin = &PINC;	pin->mPort = &PORTC;	pin->mDdr = &DDRC;		pin->mBit = 4;
			break;
		case 19:	// A5
			pin->mPin = &PINC;	pin->mPort = &PORTC;	pin->mDdr = &DDRC;		pin->mBit = 5;
			break;

		case 2:	// D2
			pin->mPin = &PIND;	pin->mPort = &PORTD;	pin->mDdr = &DDRD;		pin->mBit = 2;
			break;
		case 3:	// D3
			pin->mPin = &PIND;	pin->mPort = &PORTD;	pin->mDdr = &DDRD;		pin->mBit = 3;
			break;
		case 4:	// D4
			pin->mPin = &PIND;	pin->mPort = &PORTD;	pin->mDdr = &DDRD;		pin->mBit = 4;
			break;
		case 5:	// D5
			pin->mPin = &PIND;	pin->mPort = &PORTD;	pin->mDdr = &DDRD;		pin->mBit = 5;
			break;
		case 6:	// D6
			pin->mPin = &PIND;	pin->mPort = &PORTD;	pin->mDdr = &DDRD;		pin->mBit = 6;
			break;
		case 7:	// D7
			pin->mPin = &PIND;	pin->mPort = &PORTD;	pin->mDdr = &DDRD;		pin->mBit = 7;
			break;
		case 8:	// D8
			pin->mPin = &PINB;	pin->mPort = &PORTB;	pin->mDdr = &DDRB;		pin->mBit = 0;
			break;
		case 9:	// D9
			pin->mPin = &PINB;	pin->mPort = &PORTB;	pin->mDdr = &DDRB;		pin->mBit = 1;
			break;
		case 10:	// D10
			pin->mPin = &PINB;	pin->mPort = &PORTB;	pin->mDdr = &DDRB;		pin->mBit = 2;
			break;
		case 11:	// D11
			pin->mPin = &PINB;	pin->mPort = &PORTB;	pin->mDdr = &DDRB;		pin->mBit = 3;
			break;
		case 12:	// D12
			pin->mPin = &PINB;	pin->mPort = &PORTB;	pin->mDdr = &DDRB;		pin->mBit = 4;
			break;
	
	}
}

uint8_t analog_reference = 1;
int faAnalogRead(uint8_t pin)
{
	uint8_t low, high;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#endif
	pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
	if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
  
	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
	ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif
#endif

	// without a delay, we seem to read from the wrong channel
	//delay(1);

#if defined(ADCSRA) && defined(ADCL)
	// start the conversion
	
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
	high = ADCH;
#else
	// we dont have an ADC, return 0
	low  = 0;
	high = 0;
#endif

	// combine the two bytes
	return (high << 8) | low;
}


