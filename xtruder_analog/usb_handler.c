
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


inline void initUsb() {
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect
}


inline int readData() {
	uint8_t bytesRcvd = USBHID_bytesInUSBBuffer(HID0_INTFNUM);
	if (bytesRcvd>0) {
		USBHID_rejectData(HID0_INTFNUM);   //  incoming pkt is simple "ping" no need to decode it
	}
	return bytesRcvd;
}

inline int writeData() {

	// check for error or send in progress
	// only continue if USBHID_intfStatus() returns 0 (all clear)
    uint16_t bytesSent, bytesReceived;
	if (USBHID_intfStatus(HID0_INTFNUM, &bytesSent, &bytesReceived)!=0) return bytesSent;

	// setup packet
	uint8_t msgLen=9;
	dataBuffer[0] = OUT_PKT_TYPE_INFO;

	// copy data
	dataBuffer[1] = (adcValue[0] >> 8) & 0xFF;
	dataBuffer[2] = adcValue[0] & 0xFF;

	dataBuffer[3] = (adcValue[1] >> 8) & 0xFF;
	dataBuffer[4] = adcValue[1] & 0xFF;

	dataBuffer[5] = (adcValue[2] >> 8) & 0xFF;
	dataBuffer[6] = adcValue[2] & 0xFF;

	dataBuffer[7] = (adcValue[3] >> 8) & 0xFF;
	dataBuffer[8] = adcValue[3] & 0xFF;

	// send
	if (USBHID_sendData((uint8_t*)dataBuffer, msgLen, HID0_INTFNUM)!=kUSBHID_sendStarted) {
		USBHID_abortSend(&bytesSent, HID0_INTFNUM);
	}

	return bytesSent;

}


// returns 1 upon a successful read and write operation
// returns 0 if nothing happened (due to no message being available, device not connected, error, etc.)
inline uint8_t handleUsb() {
	if (USB_connectionState()==ST_ENUM_ACTIVE) {   //  device is enumerated on the USB host
		if (bHIDDataReceived_event){
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

