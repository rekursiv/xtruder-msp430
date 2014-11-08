
#include "driverlib.h"
#include "hal.h"

#include "usb_handler.h"

#include "iodef_test.h"  //   TEST
//#include "iodef_prod.h"

#define TIMER_A_PERIOD 3200


volatile int16_t stepCount=0;
volatile int16_t stepCountMin=0;
volatile int16_t stepCountMax=0;


inline void initTimer() {
	TIMER_A_clearTimerInterruptFlag(TIMER_A0_BASE);
    TIMER_A_configureUpMode(
    	TIMER_A0_BASE,
        TIMER_A_CLOCKSOURCE_ACLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        TIMER_A_PERIOD,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
        );
    TIMER_A_startCounter(
		TIMER_A0_BASE,
		TIMER_A_UP_MODE
		);
}

inline void initQuadratureInputs() {
	GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN4|GPIO_PIN5);
	GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4|GPIO_PIN5);
	if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4)) {
		GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
	} else {
		GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN4, GPIO_LOW_TO_HIGH_TRANSITION);
	}
	if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5)) {
		GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
	} else {
		GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION);
	}
	GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4|GPIO_PIN5);
}

void main(void)	{													//////////////////                 ======== main ========
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer
    PMM_setVCore(PMM_CORE_LEVEL_2);    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2
    initPorts();           // Config GPIOS for low-power (output low)
    initQuadratureInputs();
    GPIO_setAsInputPinWithPullUpresistor(ZERO_BTN_PORT, ZERO_BTN_PIN);  // init button
    initClocks(25000000);   // Config clocks. MCLK=SMCLK=FLL=25MHz; ACLK=REFO=32kHz
    initTimer();
    initUsb();
    _enable_interrupts();

    while (1) {
        __bis_SR_register(LPM0_bits + GIE);
        handleUsb();
    }
}


#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
	if ((P1IFG&QA_INPUT)==QA_INPUT) {
		if ((P1IES&QA_INPUT)==0) {
			if ((P1IN&QB_INPUT)==QB_INPUT) --stepCount; else ++stepCount;
			P1IES |= QA_INPUT;	// select falling edge
			QA_LED_PORT |= QA_LED_PIN;		// yellow LED on
		} else {
			if ((P1IN&QB_INPUT)==QB_INPUT) ++stepCount; else --stepCount;
			P1IES &= ~QA_INPUT;	// select rising edge
			QA_LED_PORT &= ~QA_LED_PIN;		// yellow LED off
		}
	} else if ((P1IFG&QB_INPUT)==QB_INPUT) {
		if ((P1IES&QB_INPUT)==0) {
			if ((P1IN&QA_INPUT)==QA_INPUT) ++stepCount; else --stepCount;
			P1IES |= QB_INPUT;	// select falling edge
			QB_LED_PORT |= QB_LED_PIN;		// red LED on
		} else {
			if ((P1IN&QA_INPUT)==QA_INPUT) --stepCount; else ++stepCount;
			P1IES &= ~QB_INPUT;	// select rising edge
			QB_LED_PORT &= ~QB_LED_PIN;		// red LED off
		}
	}
	if (stepCountMax<stepCount) stepCountMax=stepCount;
	else if (stepCountMin>stepCount) stepCountMin=stepCount;
	P1IFG = 0;  //  clear interrupt flag for port 1
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
	if ((ZERO_BTN_REG&ZERO_BTN_PIN)==0) {   //  zero btn
		stepCount=0;
		stepCountMin=0;
		stepCountMax=0;
	}
	_enable_interrupts();
	if (stepCount==0) {
		ZERO_LED_PORT |= ZERO_LED_PIN;
	} else {
		ZERO_LED_PORT &= ~ZERO_LED_PIN;
	}
}

