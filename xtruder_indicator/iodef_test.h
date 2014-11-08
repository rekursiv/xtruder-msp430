
//   IO using Launchpad LEDs/buttons (for testing)

// port 1
#define QA_INPUT BIT4
#define QB_INPUT BIT5


#define ZERO_BTN_PIN BIT1
#define ZERO_BTN_REG P1IN
#define ZERO_BTN_PORT GPIO_PORT_P1

// red LED
#define QA_LED_PIN BIT0
#define QA_LED_PORT P1OUT

// external LED
#define QB_LED_PIN BIT5
#define QB_LED_PORT P2OUT

// green LED
#define ZERO_LED_PIN BIT7
#define ZERO_LED_PORT P4OUT

