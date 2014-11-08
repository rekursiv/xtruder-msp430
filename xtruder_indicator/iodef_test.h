
//   IO using Launchpad LEDs/buttons (for testing)

#define QA_INPUT BIT4  // port 1
#define QB_INPUT BIT5  // port 1

#define ZERO_BTN_PIN BIT1
#define ZERO_BTN_REG P1IN
#define ZERO_BTN_PORT GPIO_PORT_P1

#define QA_LED_PIN BIT0		// red LED
#define QA_LED_PORT P1OUT

#define QB_LED_PIN BIT5		// external LED
#define QB_LED_PORT P2OUT

#define ZERO_LED_PIN BIT7	// green LED
#define ZERO_LED_PORT P4OUT


