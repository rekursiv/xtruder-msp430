

#ifndef USB_H_
#define USB_H_

extern volatile int16_t stepCount;
extern volatile int16_t stepCountMin;
extern volatile int16_t stepCountMax;

inline void initUsb();
inline uint8_t handleUsb();



#endif /* USB_H_ */
