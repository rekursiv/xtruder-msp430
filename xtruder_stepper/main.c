
#include "inc/hw_regaccess.h"
#include "driverlib.h"
#include "hal.h"

#include "usb_handler.h"

#include "iodef_test.h"  //   TEST
//#include "iodef_prod.h"

#define SPICLK 1000000


uint8_t timerWakeUp=0;

volatile int16_t curMotorSpeed = 0;
uint8_t prevTorque=1;


////////////  CONFIG   /////////

uint8_t accelDiv = 1;
uint8_t accelStep = 8;
uint8_t holdingTorque = 0;
uint8_t minTorque = 127;
uint8_t maxTorque = 255;
uint8_t torqueDiv = 255;
uint8_t stepMode = 4;
uint8_t isGain = 3;   // 3

////////////////////////////

volatile uint8_t mcStatus=0;
volatile int16_t targetMotorSpeed = 0;
volatile uint8_t curTorque=0;

///////////////////////////////////////////////////


inline uint16_t calcPeriod() {
	// minimum steps per second using 1mhz clock and 16 bit counter: 16
	float ms = (float)abs(curMotorSpeed);
	if (ms<16.0f) ms=16.0f;
	return (1.0f/ms)*1000000;
}


inline uint8_t calcTorque() {
	uint16_t ms = abs(curMotorSpeed);
	uint16_t torque = holdingTorque;
	if (ms>16) {
		torque = (ms/torqueDiv)+minTorque;
		if (torque>maxTorque) torque=maxTorque;
	}
	return torque;
}


inline void initTimerA() {

	TIMER_A_clearTimerInterruptFlag(TIMER_A0_BASE);
    TIMER_A_configureUpMode(
    	TIMER_A0_BASE,
        TIMER_A_CLOCKSOURCE_ACLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        accelDiv,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
        );

    TIMER_A_startCounter(
		TIMER_A0_BASE,
		TIMER_A_UP_MODE
		);

}

inline void initTimerB() {

	TIMER_B_clearTimerInterruptFlag(TIMER_B0_BASE);
    TIMER_B_configureUpMode(
    	TIMER_B0_BASE,
        TIMER_B_CLOCKSOURCE_SMCLK,
        TIMER_B_CLOCKSOURCE_DIVIDER_24,
        calcPeriod(),
        TIMER_B_TBIE_INTERRUPT_DISABLE,
        TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE,
        TIMER_B_DO_CLEAR
        );


    TIMER_B_startCounter(
		TIMER_B0_BASE,
		TIMER_B_UP_MODE
		);

}


inline void initSpi() {

    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN1);

    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P3, GPIO_PIN0 + GPIO_PIN2 );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P3, GPIO_PIN1 );

 //   GPIO_setAsInputPinWithPullDownresistor( GPIO_PORT_P3, GPIO_PIN1 );

    //Initialize Master
    uint8_t initStat = USCI_B_SPI_masterInit(USCI_B0_BASE,
        USCI_B_SPI_CLOCKSOURCE_SMCLK,
        UCS_getSMCLK(),
        SPICLK,
        USCI_B_SPI_MSB_FIRST,
        USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,//USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
        USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
        );

//    if (initStat==STATUS_FAIL)  //  TODO: handle error

    //Enable SPI module
    USCI_B_SPI_enable(USCI_B0_BASE);

    //Wait for slave to initialize
    __delay_cycles(100);

}

inline void sendSpiData(uint8_t hi, uint8_t lo) {
//	_disable_interrupts();
    while (!(HWREG8(USCI_B0_BASE+OFS_UCBxIFG)&UCTXIFG));
    P8OUT |= BIT1;
    USCI_B_SPI_transmitData(USCI_B0_BASE, hi);
    while (!(HWREG8(USCI_B0_BASE+OFS_UCBxIFG)&UCTXIFG));
    USCI_B_SPI_transmitData(USCI_B0_BASE, lo);
    while (!(HWREG8(USCI_B0_BASE+OFS_UCBxIFG)&UCTXIFG));
    __delay_cycles(180);
    P8OUT &= ~BIT1;
//    _enable_interrupts();

    if (hi==0b11110000) mcStatus=UCB0RXBUF;

}


inline void sendCtrl() {
	uint8_t ctrlLo = stepMode<<3;

	if (curTorque>0) ctrlLo|=0b00000001;  // enable

	uint8_t ctrlHi = isGain|0b00001110;
	//                        daaaddii

	sendSpiData(ctrlHi, ctrlLo);
}


inline void setupPorts() {

	initPorts();           // Config GPIOS for low-power (output low)

	GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);   // STEP

	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN0);  // STALL
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2); // FAULT

	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
}

inline void resetMotorController() {
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6);  // SLEEP
	GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);  //  RESET
	__delay_cycles(2500000);
	GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);  //  RESET

	sendSpiData(0b01110000, 0b00000000);  // clear STATUS
	//         daaaxxxx    ssvppcct

	sendSpiData(0b00110001, 0b10000000);  // BLANK
	//         daaaxxxe    tttttttt

	sendSpiData(0b01000100, 0b00010000);  // DECAY
	//         daaaxmmm    tttttttt

	sendSpiData(0b01100000, 0b01011111);  // DRIVE
	//         daaaiiii    ttttggoo

	curTorque=holdingTorque;
	sendCtrl();
}

void main(void)	{													//////////////////                 ======== main ========
	WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

	PMM_setVCore(PMM_CORE_LEVEL_2);    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2

	setupPorts();
	initClocks(24000000);   // Config clocks. MCLK=SMCLK=FLL=24MHz; ACLK=REFO=32kHz
	initSpi();
	initUsb();
	initTimerA();
	initTimerB();

	resetMotorController();

	_enable_interrupts();


	while (1) {
		__bis_SR_register(LPM0_bits + GIE);

		if (timerWakeUp) {

			// process acceleration
			int32_t delta = (int32_t)targetMotorSpeed-(int32_t)curMotorSpeed;
			int32_t step = 0;
			if (delta>0) {
				if (delta<accelStep) step=delta;
				else step=accelStep;
			}
			else if (delta<0) {
				if (delta>-step) step=delta;
				else step=-accelStep;
			}
			if (step!=0) {

				// set motor speed
				curMotorSpeed+=step;
				TB0CCR0=calcPeriod();

				// set direction pin
				if (curMotorSpeed>0) DIR_PORT |= DIR_PIN;
				else DIR_PORT &= ~DIR_PIN;

				// process dynamic torque
				curTorque = calcTorque();
				if (curTorque!=prevTorque) {
					if (curTorque==0||prevTorque==0) {
						sendCtrl();
					}
					prevTorque = curTorque;
					sendSpiData(0b00010001, curTorque);  // TORQUE
					//         daaaxeee    tttttttt
				}
			}

			timerWakeUp=0;
		}

		if (handleUsb()) {
			// TODO: only on status cmd
			sendSpiData(0b11110000, 0b00000000);  //  read SATAUS
		}
	}
}






#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
	timerWakeUp=1;
	__bic_SR_register_on_exit(CPUOFF);
}



#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR (void)
{

	if (abs(curMotorSpeed)>15) {
		STEP_PORT |= STEP_PIN;			// STEP high
		__delay_cycles(48);  	// stay high for ~2uS
		STEP_PORT &= ~STEP_PIN;			// STEP low
	}

}




//if (targetMotorSpeed>100) P1OUT ^= BIT0;    ///////   TEST
//P1OUT |= BIT0;
//P1OUT &= ~BIT0;
