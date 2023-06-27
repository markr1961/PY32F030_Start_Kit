// pin-port defines

#ifndef _PIN_PORT_H
#define _PIN_PORT_H

#define LED_PIN     GPIO_PIN_11
#define LED_PORT    GPIOA
#define USER_LED    LED_PORT, LED_PIN

#define BUTTON_PIN  GPIO_PIN_12
#define BUTTON_PORT GPIOA
#define USER_BUTTON BUTTON_PORT, BUTTON_PIN

#define JOYSTICK_X_PIN  LL_GPIO_PIN_4
#define JOYSTICK_Y_PIN  LL_GPIO_PIN_5
#define JOYSTCK_PORT    GPIOA


#endif