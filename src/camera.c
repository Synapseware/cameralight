#include "camera.h"

volatile uint8_t	_tick			= 0;
volatile uint16_t	_sense_val		= 0;
volatile uint16_t 	_sample_skip	= 0;
volatile uint8_t	_process		= 0;
volatile uint8_t	_brightness		= 0;
volatile uint8_t	_target			= 0;
volatile uint8_t 	_sleep			= 0;


//------------------------------------------------------------------------------------------
// Prepares the CPU for deep sleep
void shutdownCpu(void)
{
	// shutdown comparator
	ACSR |= (1<<ACD);

	// shutdown adc
	ADCSRA	= 0;

	// shutdown timer0
	TIMSK0	= 0;
	TCCR0B	= 0;
	TCCR0A	= 0;
	OCR0B	= 0;
	OCR0A	= 0;

	// disable I/O pins
	MCUCR	|= (1<<PUD);
	DDRB	= 0;
	PORTB	= 0xFF;
}


//------------------------------------------------------------------------------------------
// Prepare ADC to read sensor output
void configureSensor(void)
{
	// configure ADC
	ADMUX	=	(0<<REFS0)	|
				(0<<ADLAR)	|
				SENSE_MUX;
	ADCSRA	=	(0<<ADEN)	|
				(0<<ADSC)	|
				(0<<ADATE)	|
				(0<<ADIF)	|
				(1<<ADIE)	|
				(1<<ADPS2)	|		// prescaler = 64 (9.6mHz/64 = 150kHz ADC clock)
				(1<<ADPS1)	|
				(0<<ADPS0);
	ADCSRB	=	(0<<ADTS2)	|
				(0<<ADTS1)	|
				(0<<ADTS0);
	DIDR0	=	(1<<SENSE_DIDR);

	PORTB	&=	~(1<<SENSE_INPUT);
}


//------------------------------------------------------------------------------------------
// Prepare timer0
void configureTimer(void)
{
	TCCR0A	=	(0<<COM0A1)	|
				(0<<COM0A0)	|
				(0<<COM0B1)	|	// Enable output on COM0B1 (PB1)
				(0<<COM0B0)	|
				(1<<WGM01)	|	// Fast PWM
				(1<<WGM00);

	TCCR0B	=	(0<<FOC0A)	|
				(0<<FOC0B)	|
				(1<<WGM02)	|
				(0<<CS02)	|	//clk/64
				(1<<CS01)	|
				(1<<CS00);

	TIMSK0	=	(0<<OCIE0B)	|
				(1<<OCIE0A)	|	// interrupt on A
				(0<<TOIE0);

	OCR0A	=	F_CPU/64/1000-1;
	OCR0B	=	0;				// start with minimum brightness value

	disablePWM();
}


//------------------------------------------------------------------------------------------
// Prepare output LED
void configureLED(void)
{
	PORTB	&=	~(1<<LED_PIN);
	DDRB	|=	(1<<LED_PIN);
}


//------------------------------------------------------------------------------------------
// Prepare system
void init(void)
{
	shutdownCpu();

	configureSensor();
	configureTimer();
	configureLED();

	set_sleep_mode(SLEEP_MODE_ADC);
	sleep_enable();

	WD_SET(WD_RST, WDTO_120MS);

	sei();
}


//------------------------------------------------------------------------------------------
// Configure timer0 for PWM output
void enablePWM(void)
{
	DDRB	|=	(1<<PWM_PIN);	// enable output PWM pin
	TCCR0A	|=	PWM_MASK;
}


//------------------------------------------------------------------------------------------
// Disable timer0 PWM output
void disablePWM(void)
{
	TCCR0A	&=	~PWM_MASK;
	OCR0B	=	0;
	DDRB	&=	~(1<<PWM_PIN);
	PORTB	&=	~(1<<PWM_PIN);
}


//------------------------------------------------------------------------------------------
// 
void enableSensor(void)
{
	// configure sensor driver enable (1 = enable/0 = disable)
	DDRB	|= (1<<SENSE_EN);
	PORTB	|= (1<<SENSE_EN);
}


//------------------------------------------------------------------------------------------
// 
void disableSensor(void)
{
	// configure sensor driver enable (1 = enable/0 = disable)
	PORTB	&= ~(1<<SENSE_EN);
	DDRB	&= ~(1<<SENSE_EN);
}


//------------------------------------------------------------------------------------------
// Deep sleep
void deepSleep(void)
{
	cli();

	shutdownCpu();

	// wait for any pending conversions to complete
	while ((ADCSRA & (1<<ADSC)) != 0);

	// configure WDT to wake up every 500ms to check the proximity sensor
	WD_SET(WD_IRQ, WDTO_1S);
	sei();

	// enter power down
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	while (1)
	{
		sleep_cpu();
		wdt_reset();
		checkSensorForWakeup();
		if (checkSensorForWakeup())
			break;
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	}

	init();
}


//------------------------------------------------------------------------------------------
// Performs a proximity sensor reading and checks the result to see if we should wake up
uint8_t checkSensorForWakeup(void)
{
	_sample_skip = 0;
	_process = 0;
	_sense_val = 0xffff;

	// warm up hardware & get a sensor reading
	configureSensor();
	enableSensor();
	startSensorSample();

	// shutdown again
	shutdownCpu();

	if (_sense_val < 200)
		return 1;

	return 0;
}


//------------------------------------------------------------------------------------------
// Enables the optical sensor and starts a conversion
void startSensorSample(void)
{
	// don't start any samples if we are skipping readings
	if (_sample_skip > 0)
	{
		_sample_skip--;
		return;
	}

	// enable sensor and ADC
	enableSensor();
	ADCSRA	|=	(1<<ADEN);

	// ADC sleep mode will trigger an ADC conversion when sleep mode is entered
	set_sleep_mode(SLEEP_MODE_ADC);
	sleep_enable();
	sleep_cpu();
}


//------------------------------------------------------------------------------------------
// Processes the optical sensor data
void processSensorSample(void)
{
	// don't do anything if our data isn't ready
	if (!_process)
		return;

	// take action based on sensor reading
	if (_sense_val < 200)
	{
		// turn on proximity notification LED
		PORTB |= (1<<LED_PIN);
		_sample_skip = 750;

		switch (_brightness)
		{
			case 0:
				enablePWM();
				_target = OCR0A;
				break;
			case 1:
				enablePWM();
				_target = PWM_MAX/2;
				break;
			case 2:
				enablePWM();
				_target = PWM_MAX/5;
				break;
			case 3:
				_target = 0;
				break;
		}

		if (++_brightness > 3)
			_brightness = 0;
	}
	else
	{
		// turn off proximity notification LED
		PORTB &= ~(1<<LED_PIN);
		_sample_skip = 100;
	}

	_process = 0;
}


//------------------------------------------------------------------------------------------
// Main
int main(void)
{
	MCUSR &= ~(1<<WDRF);

	deepSleep();

	while (1)
	{
		// process code here that needs to run continuously
		wdt_reset();

		if (_sleep)
		{
			disablePWM();
			_sleep = 0;
			deepSleep();
			continue;
		}

		if (!_tick)
			continue;

		// do work

		startSensorSample();
		processSensorSample();

		_tick = 0;
	}

	return 0;
}


//------------------------------------------------------------------------------------------
// Capture timer0 compareA interrupt
ISR(TIM0_COMPA_vect)
{
	// flag tick event
	_tick = 1;

	static uint8_t delay = 4;
	if (delay > 0)
	{
		delay--;
		return;
	}

	delay = 4;

	if (OCR0B > _target)
		OCR0B--;
	else if (OCR0B < _target)
		OCR0B++;

	if (0 == OCR0B)
		_sleep = 1;
}


//------------------------------------------------------------------------------------------
// Capture ADC reading
ISR(ADC_vect)
{
	disableSensor();
	ADCSRA	&=	~(1<<ADEN);

	// get sensor value
	_sense_val = (ADCL) | (8<<ADCH);

	// flag sample ready state
	_process = 1;
}


//------------------------------------------------------------------------------------------
// Watchdog timer interrupt vector
ISR(WDT_vect)
{
	// no-op
	//PINB |= (1<<LED_PIN);
}
