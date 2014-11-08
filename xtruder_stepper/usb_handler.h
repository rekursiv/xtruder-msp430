

#ifndef USB_H_
#define USB_H_

extern volatile int16_t targetMotorSpeed;
extern volatile int16_t curMotorSpeed;
extern volatile uint8_t curTorque;

extern volatile uint8_t mcStatus;


inline void initUsb();
inline uint8_t handleUsb();



#endif /* USB_H_ */
