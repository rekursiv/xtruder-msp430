

#ifndef USB_H_
#define USB_H_



////////////  CONFIG   /////////

extern volatile uint8_t stepMode;
extern volatile uint8_t isGain;
extern volatile uint8_t holdingTorque;
extern volatile uint8_t minTorque;
extern volatile uint8_t maxTorque;
extern volatile uint8_t torqueDiv;
extern volatile uint8_t accelDiv;
extern volatile uint8_t accelStep;

extern volatile uint16_t loPos;
extern volatile uint16_t hiPos;
extern volatile uint16_t posCountDiv;

////////////////////////////

extern volatile uint8_t mcStatus;
extern volatile int16_t targetMotorSpeed;
extern volatile uint8_t curTorque;
extern volatile uint8_t curCmd;
extern volatile int16_t curMotorSpeed;

extern volatile uint16_t curPosCount;

///////////////////////////////////////////////////


extern volatile uint16_t curStepCount;     //  FIXME:  currently "flip-flop mode" must be homed every time it is started
extern volatile uint8_t homingState;

////////////

inline void initUsb();
inline uint8_t handleUsb();


#endif /* USB_H_ */
