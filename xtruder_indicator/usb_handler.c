
#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 // USB-specific functions
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_app/usbConstructs.h"

#include "hal.h"

#include "usb_handler.h"


#define OUT_PKT_TYPE_INFO 0

#define BUFFER_SIZE 64
uint8_t dataBuffer[BUFFER_SIZE];

// Global flags set by events
volatile uint8_t bHIDDataReceived_event = FALSE;  // Flag set by event handler to indicate data has been received into USB buffer

volatile int16_t stepCountCopy;
volatile int16_t stepCountMinCopy;
volatile int16_t stepCountMaxCopy;


inline void initUsb() {
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect
}



inline void readData() {
	uint8_t msgLen=0;
	while (USBHID_bytesInUSBBuffer(HID0_INTFNUM)) {
		msgLen = hidReceiveDataInBuffer((uint8_t*)dataBuffer, BUFFER_SIZE, HID0_INTFNUM);
		if (msgLen>0) {
			if (dataBuffer[0]==1) {  // zero
				_disable_interrupts();
				stepCount=0;
				stepCountMin=0;
				stepCountMax=0;
				_enable_interrupts();
			}
		}
	}
}

inline void writeData() {
	// atomic data copy
	stepCountCopy=stepCount;
	stepCountMinCopy=stepCountMin;
	stepCountMaxCopy=stepCountMax;

	// reset min max
	stepCountMin=stepCount;
	stepCountMax=stepCount;

	// setup packet
	uint8_t msgLen=7;
	dataBuffer[0] = OUT_PKT_TYPE_INFO;

	// copy data
	dataBuffer[1] = (stepCountCopy >> 8) & 0xFF;
	dataBuffer[2] = stepCountCopy & 0xFF;
	dataBuffer[3] = (stepCountMinCopy >> 8) & 0xFF;
	dataBuffer[4] = stepCountMinCopy & 0xFF;
	dataBuffer[5] = (stepCountMaxCopy >> 8) & 0xFF;
	dataBuffer[6] = stepCountMaxCopy & 0xFF;

	// send
	hidSendDataInBackground((uint8_t*)dataBuffer, msgLen, HID0_INTFNUM, 0);
}


// returns 1 upon a successful read and write operation
// returns 0 if nothing happened (due to no message being available, device not connected, error, etc.)
inline uint8_t handleUsb() {
	if (USB_connectionState()==ST_ENUM_ACTIVE) {   //  device is enumerated on the USB host
		if (bHIDDataReceived_event) {
			bHIDDataReceived_event = FALSE;
			readData();
			writeData();
			return 1;
		}
	}
	return 0;
}



#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
{
	switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
	{
	case SYSUNIV_OFIFG:
		UCS_clearFaultFlag(UCS_XT2OFFG);
		UCS_clearFaultFlag(UCS_DCOFFG);
		SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
		break;
	case SYSUNIV_BUSIFG:
		// If the CPU accesses USB memory while the USB module is
		// suspended, a "bus error" can occur.  This generates an NMI.  If
		// USB is automatically disconnecting in your software, set a
		// breakpoint here and see if execution hits it.  See the
		// Programmer's Guide for more information.
		SYSBERRIV = 0; //clear bus error flag
		USB_disable(); //Disable
	}
}

