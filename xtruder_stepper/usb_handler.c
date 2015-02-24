
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
uint8_t dataBufferIn[BUFFER_SIZE];
uint8_t dataBufferOut[BUFFER_SIZE];

volatile int16_t posCountCopy;
volatile int16_t targetMotorSpeedIn;

// Global flags set by events
volatile uint8_t bHIDDataReceived_event = FALSE;  // Flag set by event handler to indicate data has been received into USB buffer


inline void initUsb() {
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect
}


inline int readData() {

	uint8_t bytesRcvd = USBHID_bytesInUSBBuffer(HID0_INTFNUM);
	if (bytesRcvd>BUFFER_SIZE) {
		USBHID_rejectData(HID0_INTFNUM);
	} else if (bytesRcvd>0) {
		if (USBHID_receiveData(dataBufferIn, bytesRcvd, HID0_INTFNUM) == kUSBHID_busNotAvailable) {
			uint16_t rcvLen;
			USBHID_abortReceive(&rcvLen, HID0_INTFNUM);
			bytesRcvd = (uint8_t)rcvLen;
		} else {
			curCmd=dataBufferIn[0];
			if (curCmd==1&&bytesRcvd==15) {
				stepMode=dataBufferIn[1];
				isGain=dataBufferIn[2];
				holdingTorque=dataBufferIn[3];
				minTorque=dataBufferIn[4];
				maxTorque=dataBufferIn[5];
				torqueDiv=dataBufferIn[6];
				accelDiv=dataBufferIn[7];
				accelStep=dataBufferIn[8];
				loPos=(dataBufferIn[9]<<8)|dataBufferIn[10];
				hiPos=(dataBufferIn[11]<<8)|dataBufferIn[12];
				posCountDiv=(dataBufferIn[13]<<8)|dataBufferIn[14];
			} else if (curCmd==2&&bytesRcvd==3) {
				targetMotorSpeedIn=(dataBufferIn[1]<<8)|dataBufferIn[2];

				if (posCountDiv!=0) {  // maintain the current direction in case of "flip flop mode"   TODO: fix this ugly hack
					if (targetMotorSpeed>0) targetMotorSpeed=abs(targetMotorSpeedIn);
					else if (targetMotorSpeed<0) targetMotorSpeed=-abs(targetMotorSpeedIn);
					else {
						curStepCount=0;
						curPosCount=0;
						homingState=2;
						targetMotorSpeed=targetMotorSpeedIn;
					}
				} else {
					targetMotorSpeed=targetMotorSpeedIn;
				}
			}
		}
	}
	return bytesRcvd;
}


inline int writeData() {

	// check for error or send in progress
	// only continue if USBHID_intfStatus() returns 0 (all clear)
    uint16_t bytesSent, bytesReceived;
	if (USBHID_intfStatus(HID0_INTFNUM, &bytesSent, &bytesReceived)!=0) return bytesSent;

	// atomic data copy
	posCountCopy = curPosCount;

	// setup packet
	uint8_t msgLen=7;
	dataBufferOut[0] = OUT_PKT_TYPE_INFO;
	dataBufferOut[1] = (curMotorSpeed >> 8) & 0xFF;
	dataBufferOut[2] = curMotorSpeed & 0xFF;
	dataBufferOut[3] = curTorque;
	dataBufferOut[4] = mcStatus;
	dataBufferOut[5] = (posCountCopy >> 8) & 0xFF;
	dataBufferOut[6] = posCountCopy & 0xFF;

	// send
	if (USBHID_sendData((uint8_t*)dataBufferOut, msgLen, HID0_INTFNUM)!=kUSBHID_sendStarted) {
		USBHID_abortSend(&bytesSent, HID0_INTFNUM);
	}

	return bytesSent;

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



