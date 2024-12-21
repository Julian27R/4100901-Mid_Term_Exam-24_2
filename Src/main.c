// Access Control System Implementation

#include "gpio.h"
#include "systick.h"
#include "uart.h"
#include "rcc.h"

#define TEMP_UNLOCK_DURATION 5000 // Duration in ms for temporary unlock

typedef enum {
    LOCKED,   //ESTADO DE BLOQUEO 
    TEMP_UNLOCK,  //BLOQUEO TEMPORAL
    PERM_UNLOCK   //DESBLOQUEO PERMANENTE
} DoorState_t;

//Variables Locales
DoorState_t current_state = LOCKED; // Estado inicial: bloqueado
uint32_t unlock_timer = 0;      // Temporizador para rastrear el desbloqueo temporal

void run_state_machine(void) {
    switch (current_state) {
        case LOCKED:
            // No periodic action in locked state
            break;
        case TEMP_UNLOCK:
            if (systick_GetTick() - unlock_timer >= TEMP_UNLOCK_DURATION) {
                gpio_set_door_led_state(0); // Turn off door state LED
                current_state = LOCKED;
            }
            break;
        case PERM_UNLOCK:
            // No periodic action in permanent unlock state
            break;
    }
}

void handle_event(uint8_t event) {
    if (event == 1) { // Pulsacion simple
        gpio_set_door_led_state(1); // Turn on door state LED
        current_state = TEMP_UNLOCK;
        unlock_timer = systick_GetTick();
    } else if (event == 2) { // Double button press
        gpio_set_door_led_state(1); // Turn on door state LED
        current_state = PERM_UNLOCK;
    } else if (event == 'O') { // UART OPEN command
        gpio_set_door_led_state(1); // Turn on door state LED
        current_state = TEMP_UNLOCK;
        usart2_send_string("DOOR OPEN!\r\n");  //Enviara un mensaje que dirá Puerta Abierta
        unlock_timer = systick_GetTick();
    } else if (event == 'C') { // UART CLOSE command
        gpio_set_door_led_state(0); // Turn off door state LED
        current_state = LOCKED;
        usart2_send_string("DOOR CLOSE!\r\n");  //Enviara un mensaje que dirá puerta cerrada
    }
}

int main(void) {
    //Inilizaciones
    configure_systick_and_start();
    configure_gpio();
    usart2_init();

    usart2_send_string("System Initialized\r\n");

    uint32_t heartbeat_tick = 0;
    while (1) {
        if (systick_GetTick() - heartbeat_tick >= 500) {
            heartbeat_tick = systick_GetTick();
            gpio_toggle_heartbeat_led();
        }

        uint8_t button_pressed = button_driver_get_event();
        if (button_pressed != 0) {
            handle_event(button_pressed);
            button_pressed = 0;
        }

        uint8_t rx_byte = usart2_get_command();
        if (rx_byte != 0) {
            handle_event(rx_byte);
            rx_byte = 0;
        }

        run_state_machine();
    }
}
