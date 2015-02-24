
// IO for "production" module (with stepper controller booster pack plugged in)


#define STEP_PIN BIT2
#define STEP_PORT P4OUT

#define DIR_PIN BIT1
#define DIR_PORT P4OUT

// homing switch
#define HOME_PIN BIT0
#define HOME_REG P6IN
#define HOME_PORT GPIO_PORT_P6
