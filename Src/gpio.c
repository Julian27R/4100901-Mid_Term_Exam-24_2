#include "gpio.h"
#include "rcc.h"

#include "systick.h"

#define EXTI_BASE 0x40010400          
#define EXTI ((EXTI_t *)EXTI_BASE)    // Puntero a la estructura EXTI

#define EXTI15_10_IRQn 40
#define NVIC_ISER1 ((uint32_t *)(0xE000E104)) // NVIC Interrupt Set-Enable Register


#define SYSCFG_BASE 0x40010000
#define SYSCFG ((SYSCFG_t *)SYSCFG_BASE)

//Definiciones de la GPIO
#define GPIOA ((GPIO_t *)0x48000000) // Base address of GPIOA
#define GPIOC ((GPIO_t *)0x48000800) // Base address of GPIOC

// Definición de pines
#define LED_PIN 5      // LED conectado al pin 5 de GPIOA
#define LED1_PIN 1     // LED2 conectado al pin 1 de GPIOA
#define BUTTON_PIN 13  // Botón conectado al pin 13 de GPIOC

// Macros para el primer botón y LED
#define BUTTON_IS_PRESSED()    (!(GPIOC->IDR & (1 << BUTTON_PIN)))    // Lee el estado del botón - retorna true si está presionado                                                                   // (lógica negativa debido al pull-up)
#define BUTTON_IS_RELEASED()   (GPIOC->IDR & (1 << BUTTON_PIN))       // Lee el estado del botón - retorna true si está liberado
#define TOGGLE_LED()           (GPIOA->ODR ^= (1 << LED_PIN))         // Conmuta el estado del LED (encendido/apagado)  
#define TOGGLE_LED1()           (GPIOA->ODR ^= (1 << BUTTON_PIN))    // Conmuta el estado del LED1 (encendido/apagado)

// Variable global para almacenar el estado de presión del botón
volatile uint8_t button_pressed = 0;  // 0: no presionado, otro valor: presionado

void configure_gpio_for_usart(void)
{
    // // Habilita el reloj para GPIOA
    *RCC_AHB2ENR |= (1 << 0);

    // Configure PA2 (TX) as alternate function
    GPIOA->MODER &= ~(3U << (2 * 2)); // Clear mode bits for PA2
    GPIOA->MODER |= (2U << (2 * 2));  // Set alternate function mode for PA2

    // Configure PA3 (RX) as alternate function
    GPIOA->MODER &= ~(3U << (3 * 2)); // Clear mode bits for PA3
    GPIOA->MODER |= (2U << (3 * 2));  // Set alternate function mode for PA3

    // Set alternate function to AF7 for PA2 and PA3
    GPIOA->AFR[0] &= ~(0xF << (4 * 2)); // Clear AFR bits for PA2
    GPIOA->AFR[0] |= (7U << (4 * 2));   // Set AFR to AF7 for PA2
    GPIOA->AFR[0] &= ~(0xF << (4 * 3)); // Clear AFR bits for PA3
    GPIOA->AFR[0] |= (7U << (4 * 3));   // Set AFR to AF7 for PA3

    // Configure PA2 and PA3 as very high speed
    GPIOA->OSPEEDR |= (3U << (2 * 2)); // Very high speed for PA2
    GPIOA->OSPEEDR |= (3U << (3 * 2)); // Very high speed for PA3

    // Configure PA2 and PA3 as no pull-up, no pull-down
    GPIOA->PUPDR &= ~(3U << (2 * 2)); // No pull-up, no pull-down for PA2
    GPIOA->PUPDR &= ~(3U << (3 * 2)); // No pull-up, no pull-down for PA3
}

// Función para inicializar un pin específico de un puerto GPIO con un modo determinado
void init_gpio_pin(GPIO_t *GPIOx, uint8_t pin, uint8_t mode)
{
    // Limpia (pone en 0) los dos bits del registro MODER correspondientes al pin indicado
    // Cada pin tiene 2 bits en el registro MODER que determinan su modo (entrada, salida, etc.)
    GPIOx->MODER &= ~(0x3 << (pin * 2));
    
    // Establece el nuevo modo del pin configurando los 2 bits en la posición correspondiente
    GPIOx->MODER |= (mode << (pin * 2));
}

void configure_gpio(void)
{
// Primero se habilitar el reloj para los puertos GPIOA y GPIOC
*RCC_AHB2ENR |= (1 << 0) | (1 << 2);

// Se habilitar el reloj para SYSCFG
*RCC_APB2ENR |= (1 << 0); 

// Configurar el registro SYSCFG EXTICR para asignar EXTI13 al pin PC13
SYSCFG->EXTICR[3] &= ~(0xF << 4); // Limpia los bits asociados a EXTI13
SYSCFG->EXTICR[3] |= (0x2 << 4);  // Asigna el EXTI13 al puerto C

// Configurar el registro SYSCFG EXTICR para asignar EXTI0 a PC0 y EXTI1 a PC1
SYSCFG->EXTICR[0] &= ~(0xF << 0); // Limpia los bits asociados a EXTI0
SYSCFG->EXTICR[0] |= (0x2 << 0);  // Asigna el EXTI0 al puerto C
SYSCFG->EXTICR[0] &= ~(0xF << 4); // Limpia los bits asociados a EXTI1
SYSCFG->EXTICR[0] |= (0x2 << 4);  // Asigna el EXTI1 al puerto C

// Configurar EXTI13 para dispararse con el flanco de bajada
EXTI->FTSR1 |= (1 << BUTTON_PIN);  // Habilitar flanco de bajada
EXTI->RTSR1 &= ~(1 << BUTTON_PIN); // Deshabilitar flanco de subida

// Desenmascarar la línea EXTI13 para permitir interrupciones
EXTI->IMR1 |= (1 << BUTTON_PIN);

// Configurar el pin del LED como salida
init_gpio_pin(GPIOA, LED_PIN, 0x1); // Configura el LED como salida
init_gpio_pin(GPIOA, LED1_PIN, 0x1); // Configura LED1 como salida

// Configurar los pines de los botones como entrada
init_gpio_pin(GPIOC, BUTTON_PIN, 0x0);  // Configura el botón como entrada

// Habilitar interrupción EXTI15_10 en el controlador de interrupciones (NVIC)
*NVIC_ISER1 |= (1 << (EXTI15_10_IRQn - 32));

// Configurar GPIO para comunicación USART (función personalizada)
configure_gpio_for_usart();

}

// Emula el comportamiento de la puerta
void gpio_set_door_led_state(uint8_t state) {
    if (state) {
        GPIOA->ODR |= (1 << 4); // encender LED estado puerta
    } else {
        GPIOA->ODR &= ~(1 << 4); // apagar LED estado puerta
    }
}

void gpio_toggle_heartbeat_led(void) {
    GPIOA->ODR ^= (1 << 5);
}

//volatile uint8_t button_pressed = 0; // Flag to indicate button press
uint8_t button_driver_get_event(void)
{
    return button_pressed;
}

uint32_t b1_tick = 0;
void detect_button_press(void)
{
    if (systick_GetTick() - b1_tick < 50) {
        return; // Ignore bounces of less than 50 ms
    } else if (systick_GetTick() - b1_tick > 500) {
        button_pressed = 1; // single press
    } else {
        button_pressed = 2; // double press
    }

    b1_tick = systick_GetTick();
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1 << BUTTON_PIN)) {
        EXTI->PR1 = (1 << BUTTON_PIN); // Clear pending bit
        detect_button_press();
    }
}