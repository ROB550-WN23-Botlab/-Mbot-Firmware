#include <stdio.h>
#include <stdint.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#define INT_16_MAX 32768
#define ENCODER_RESOLUTION 48.0
#define GEAR_RATIO 20.4
#define TIMESTEP_S 1.5
#define NUM_POINTS 25
#define WHEEL_RADIUS 0.04
#define PI 3.1405926

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RESOLUTION));

void blink();

int main() {
    const float I_conversion_factor = 2 * 3.3f / (1 << 12);
    const float RPM_conversion_factor = 60.0 / (GEAR_RATIO * TIMESTEP_S * ENCODER_RESOLUTION);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    rc_motor_init();
    rc_encoder_init();
    blink();
    sleep_ms(10000);
    printf("\nTesting motor 1...\n");
    int32_t d = 0;
    int encoder_reading;
    float current_reading;
    float wheel_speed;
    printf("\nDuty\tSpeed\tCurrent\n");
    adc_select_input(0);
    for (; d < INT_16_MAX; d += INT_16_MAX/NUM_POINTS) {
        rc_motor_set(1, d);
        encoder_reading = -rc_encoder_read_delta(1);
        wheel_speed = RPM_conversion_factor * encoder_reading;
        current_reading = 0.0;
        for(int i=0; i<10; i++){
            current_reading += I_conversion_factor * adc_read()/10;
        }
        printf("%f\t%f\t%f\n", (float)d/(float)INT_16_MAX, wheel_speed, current_reading);
        sleep_ms(1000*TIMESTEP_S);
    }
    rc_motor_set(1, 0);
    d = 0;
    sleep_ms(3000);
    for (; d < INT_16_MAX; d += INT_16_MAX/NUM_POINTS) {
        rc_motor_set(1, -d);
        encoder_reading = -rc_encoder_read_delta(1);
        wheel_speed = RPM_conversion_factor * encoder_reading;
        current_reading = 0.0;
        for(int i=0; i<10; i++){
            current_reading += I_conversion_factor * adc_read()/10;
        }
        printf("%f\t%f\t%f\n", (float)d/(float)INT_16_MAX, wheel_speed, current_reading);
        sleep_ms(1000*TIMESTEP_S);
    }

    rc_motor_set(1, 0);
    d = 0;
    sleep_ms(3000);



    adc_select_input(2);
    printf("\nTesting motor 3...\n");
    printf("\nDuty\tSpeed\tCurrent\n");
    for (; d < INT_16_MAX; d += INT_16_MAX/NUM_POINTS) {
        rc_motor_set(3, d);
        encoder_reading = -rc_encoder_read_delta(3);
        wheel_speed = RPM_conversion_factor * encoder_reading;
        current_reading = 0.0;
        for(int i=0; i<10; i++){
            current_reading += I_conversion_factor * adc_read()/10;
        }
        printf("%f\t%f\t%f\n", (float)d/(float)INT_16_MAX, wheel_speed, current_reading);
        sleep_ms(1000*TIMESTEP_S);
    }


    rc_motor_set(3, 0);
    d = 0;
    sleep_ms(3000);


    for (; d < INT_16_MAX; d += INT_16_MAX/NUM_POINTS) {
        rc_motor_set(3, -d);
        encoder_reading = -rc_encoder_read_delta(3);
        wheel_speed = RPM_conversion_factor * encoder_reading;
        current_reading = 0.0;
        for(int i=0; i<10; i++){
            current_reading += I_conversion_factor * adc_read()/10;
        }
        printf("%f\t%f\t%f\n", (float)d/(float)INT_16_MAX, wheel_speed, current_reading);
        sleep_ms(1000*TIMESTEP_S);
    }
    
    blink();
    printf("\nDone!\n");

    while(1)
    {rc_motor_set(1, INT_16_MAX);
    rc_motor_set(3, INT_16_MAX);
    sleep_ms(1000*10);
    rc_motor_set(1, 0);
    rc_motor_set(3, 0);
    sleep_ms(1000*10);}

    
    rc_motor_cleanup(); 
    blink();
    return 0;
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
