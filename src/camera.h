#ifndef __CAMERA_H__
#define __CAMERA_H__


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <wdt.h>


//------------------------------------------------------------------------------------------
// Optical sensor
#define SENSE_INPUT		PB3
#define SENSE_MUX		((1<<MUX1) | (1<<MUX0))
#define SENSE_DIDR		ADC3D
#define SENSE_EN		PB0


//------------------------------------------------------------------------------------------
// Sensor notification LED
#define LED_PIN			PB4


//------------------------------------------------------------------------------------------
// Brightness output driver
#define PWM_PIN			PB1
#define PWM_MASK		((1<<COM0B1) | (0<<COM0B0))


#define PWM_MAX			144


//------------------------------------------------------------------------------------------
// Functions
void configureSensor(void);
void configureTimer(void);
void configureLED(void);
void init(void);
void enablePWM(void);
void disablePWM(void);
void enableSensor(void);
void disableSensor(void);
void startSensorSample(void);
void processSensorSample(void);
uint8_t checkSensorForWakeup(void);

#endif

