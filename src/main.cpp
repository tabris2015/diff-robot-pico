#include "pico/stdlib.h"
#include <stdlib.h>
#include "stdio.h"
#include "pico_explorer.hpp"
#include "button.hpp"
#include "analog.hpp"
#include "encoder.hpp"
#include "pid.hpp"

#define UART_ID uart0

#define COUNTS_PER_REV 1496

using namespace motor;
using namespace encoder;

static constexpr pin_pair MOTOR_I_PINS{8, 9};
static constexpr pin_pair MOTOR_D_PINS{7, 6};

const uint UPDATES = 100;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;

Motor motor_i(MOTOR_I_PINS);
Motor motor_d(MOTOR_D_PINS);
//Analog pot(PicoExplorer::ADC2_PIN);
Encoder enc_i(
        pio0,
        0,
        {16, 17},
        PIN_UNUSED,
    REVERSED_DIR,
COUNTS_PER_REV,
true
);
Encoder enc_d(
        pio0,
        1,
        {19, 18},
        PIN_UNUSED,
        REVERSED_DIR,
        COUNTS_PER_REV,
        true
        );

constexpr float KP = 0.25f;
constexpr float KI = 0.1f;
constexpr float KD = 0.001f;
PID pid_i = PID(KP, KI, KD, UPDATE_RATE);
PID pid_d = PID(KP, KI, KD, UPDATE_RATE);
float past_speed_i = 0;
float past_speed_d = 0;

float left = 0.0f;
float right = 0.0f;

// uart
char in_buffer[100];
uint16_t char_idx = 0;

void failRoutine()
{
    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(1500);
    }
}

bool timerCallback(repeating_timer_t * rt)
{
    Encoder::Capture capture_i = enc_i.capture();
    Encoder::Capture capture_d = enc_d.capture();

    pid_i.setpoint = left;
    pid_d.setpoint = right;

    float last_speed_i = capture_i.radians_per_second();
    float last_speed_d = capture_d.radians_per_second();

    float output_i = pid_i.calculate(last_speed_i, last_speed_i - past_speed_i);
    float output_d = pid_d.calculate(last_speed_d, last_speed_d - past_speed_d);

    past_speed_i = last_speed_i;
    past_speed_d = last_speed_d;

    motor_i.speed(output_i);
    motor_d.speed(output_d);
    return true;
}


int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    stdio_init_all();
    motor_i.init();
    motor_d.init();
    enc_i.init();
    enc_d.init();
    uint32_t i = 0;
    bool state = true;

    motor_i.speed(0.0f);
    motor_d.speed(0.0f);
    sleep_ms(2000);

    //
    int ch;
    int ch_idx;
    int value1, value2;
    float value3;
    char* ch_ptr;
    char* ch_ptr2;
    char* ch_ptr3;
    //
    repeating_timer_t timer;
    if(!add_repeating_timer_ms(-UPDATE_RATE * 1000.0f, timerCallback, NULL, &timer))
    {
        failRoutine();
    }
    //

    while(true) {
        ch = getchar_timeout_us(0);
        while(ch != PICO_ERROR_TIMEOUT)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, true);
//            printf(" %c ", ch);
            putchar(ch);
            in_buffer[ch_idx++] = ch;
            if(ch == '/')
            {
                in_buffer[ch_idx] = 0;      // end of string
//                printf("\nreceived: %s\n", in_buffer);
                ch_idx = 0;
                float aux_left = strtof(in_buffer, &ch_ptr);
                float aux_right = strtof(ch_ptr+1, &ch_ptr2);
                left = aux_left;
                right = aux_right;
//                printState(linear, angular, robot.getState(), robot.getOdometry());
                break;
            }

            ch = getchar_timeout_us(0);
        }
        gpio_put(PICO_DEFAULT_LED_PIN, false);

    }

}
