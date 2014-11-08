

#include "driverlib.h"
#include "hal.h"

#include "usb_handler.h"


volatile int16_t adcValue[4];


inline void initAdc() {

    //Configure internal reference
    while (REF_ACTIVE == REF_isRefGenBusy(REF_BASE));    //  if ref generator busy, WAIT
    REF_setReferenceVoltage(REF_BASE, REF_VREF2_5V);
    REF_enableReferenceVoltage(REF_BASE);

    //Enable A/D channel inputs
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3);

    //Initialize the ADC12_A Module
    /*
     * Base address of ADC12_A Module
     * Use internal ADC12_A bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC12_A_init(ADC12_A_BASE, ADC12_A_SAMPLEHOLDSOURCE_SC, ADC12_A_CLOCKSOURCE_ADC12OSC, ADC12_A_CLOCKDIVIDER_1);

    ADC12_A_enable(ADC12_A_BASE);

    /*
     * Base address of ADC12_A Module
     * For memory buffers 0-7 sample/hold for 256 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Enable Multiple Sampling
     */
    ADC12_A_setupSamplingTimer(ADC12_A_BASE, ADC12_A_CYCLEHOLD_256_CYCLES, ADC12_A_CYCLEHOLD_4_CYCLES, ADC12_A_MULTIPLESAMPLESENABLE);

    //Configure Memory Buffers
    ADC12_A_memoryConfigure(ADC12_A_BASE, ADC12_A_MEMORY_0, ADC12_A_INPUT_A0, ADC12_A_VREFPOS_INT, ADC12_A_VREFNEG_AVSS, ADC12_A_NOTENDOFSEQUENCE);
    ADC12_A_memoryConfigure(ADC12_A_BASE, ADC12_A_MEMORY_1, ADC12_A_INPUT_A1, ADC12_A_VREFPOS_INT, ADC12_A_VREFNEG_AVSS, ADC12_A_NOTENDOFSEQUENCE);
    ADC12_A_memoryConfigure(ADC12_A_BASE, ADC12_A_MEMORY_2, ADC12_A_INPUT_A2, ADC12_A_VREFPOS_INT, ADC12_A_VREFNEG_AVSS, ADC12_A_NOTENDOFSEQUENCE);
    ADC12_A_memoryConfigure(ADC12_A_BASE, ADC12_A_MEMORY_3, ADC12_A_INPUT_A3, ADC12_A_VREFPOS_INT, ADC12_A_VREFNEG_AVSS, ADC12_A_ENDOFSEQUENCE);

    //Enable memory buffer 3 interrupt
    ADC12_A_clearInterrupt(ADC12_A_BASE, ADC12IFG3);
    ADC12_A_enableInterrupt(ADC12_A_BASE, ADC12IE3);

}


void main(void)	{													//////////////////                 ======== main ========
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer
    PMM_setVCore(PMM_CORE_LEVEL_2);    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2
    initPorts();           // Config GPIOS for low-power (output low)
    initClocks(25000000);   // Config clocks. MCLK=SMCLK=FLL=25MHz; ACLK=REFO=32kHz
    initAdc();
    initUsb();
    _enable_interrupts();

    while (1) {
        __bis_SR_register(LPM0_bits + GIE);
        ADC12_A_startConversion(ADC12_A_BASE, ADC12_A_MEMORY_0, ADC12_A_REPEATED_SEQOFCHANNELS);
        handleUsb();
    }
}


#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR(void) {
        if (__even_in_range(ADC12IV, 34) == ADC12IV_ADC12IFG3) {
        	adcValue[0] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_0);
        	adcValue[1] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_1);
        	adcValue[2] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_2);
        	adcValue[3] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_3);
        }
}
