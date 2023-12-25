#include <atmel_start.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Parameters
#define N_MODE_ITEMS 6  // N Levels detected to extract a mode
#define N_LEVELS_ITEMS 32  // N levels detected after mode to get average
#define SAMPLING_RESOLUTION 50  // micro seconds
#define PWM_COUNT_BASE 22
#define PWM_COUNT_TOP 38
uint32_t led_count_limit = 500000 / SAMPLING_RESOLUTION; // for 0,5 seconds blink led
uint16_t duty_levels[17] = {
	// Duty levels according to a 16bit register where min value is 0x0000 and max is 0xffff
	// Also level 0x7fff led power is similar to 0xffff so in practice 0x7fff is taken as max value
	0,
	3,
	6,
	11,
	21,
	39,
	72,
	133,
	245,
	453,
	835,
	1539,
	2838,
	5232,
	9644,
	17779,
	32774,
	};

// Function declarations
void set_duty_cycle(uint8_t pwm_count);
uint16_t most_common_value(uint16_t vect[], uint8_t dim);
uint16_t avg_duty_value(uint16_t vect[], uint8_t dim);

// Global variables
bool pending_pwm_process = false;
uint16_t last_levels_detected[N_MODE_ITEMS] = {0};
uint16_t levels_after_mode[N_LEVELS_ITEMS] = {0};
uint8_t levels_detected_storage_index = 0;
uint8_t levels_after_mode_storage_index = 0;

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	TCA0.SINGLE.PERBUF = 0x0fff;
	
	PWM_OUTPUT_init();
	PWM_OUTPUT_load_duty_cycle_ch2(0x0000);
	PWM_OUTPUT_enable_output_ch2();
	
	uint8_t pwm_count = 0;
	uint32_t led_count = 0;
	
	// PWM duty cycle is set in a 16 bit register so 0xffff is max and 0x0000 is min
	bool pwm_input = false;
	
	// Loop windows with predefined resolution
	while (1) {
        
		_delay_us(SAMPLING_RESOLUTION);
		led_count++;
		
		pwm_input = PA1_get_level();
		
		// Handling PWM input, counting and setting PMW output accordingly
		if(pwm_input) {
			pending_pwm_process = true;
			pwm_count++;
		}
			
		else {
			set_duty_cycle(pwm_count);
			pwm_count = 0;	
		}
		
		// Handling LED blink
		// 100us loop so for 0.5 sec blink a 5000 factor is needed.
		if(led_count > led_count_limit) {
			led_count = 0;
			PA3_toggle_level();
		}
	}
}

// PWM duty cycle is used in 1100us to 1900us range for 0% to 100% led power respectively.
// Also, sampling period of 100us on main method must be considered.
void set_duty_cycle(uint8_t pwm_count) {
	
	if(!pending_pwm_process) return;
	
	// Handling pwm_input cycle
	// duty cycle factor is used to multiply the max duty cycle register value of 0xffff
	// so for 50% factor 50 factor is used and duty cycle register is stored with value 0xffff*50/100
	uint16_t duty_cycle_level_index = 0;
	
	// pwm_count_range for 50us sampling window is 22 to 38 so deducting 22 base we have a 0 to 16 range
	// we then multiply by 100 for a 0 to 160 range and divide by 16 for a max resolution 0 to 100 range with 17 possible steps
	uint8_t base = PWM_COUNT_BASE;
	uint8_t max_value = PWM_COUNT_TOP;

	if(pwm_count < base) duty_cycle_level_index = 0;
	else if(pwm_count > max_value) duty_cycle_level_index = max_value - base;
	else {
		duty_cycle_level_index = pwm_count - base;
	}
	
	// Appending to array
	last_levels_detected[levels_detected_storage_index] = duty_levels[duty_cycle_level_index];
	levels_detected_storage_index++;
	if(levels_detected_storage_index > N_MODE_ITEMS - 1) levels_detected_storage_index = 0;
	
	// Appending mode value once in a while to array
	
	if (levels_detected_storage_index == N_MODE_ITEMS - 1) {
		uint16_t duty_cycle_mode = most_common_value(last_levels_detected, N_MODE_ITEMS);
		
		// Appending mode to array to extract average
		levels_after_mode[levels_after_mode_storage_index] = duty_cycle_mode;
		levels_after_mode_storage_index++;
		if(levels_after_mode_storage_index > N_LEVELS_ITEMS - 1) levels_after_mode_storage_index = 0;
	}
		
	// Applying duty cycle to led once in a while
	if (levels_after_mode_storage_index == N_LEVELS_ITEMS - 1) {
		
		uint16_t duty_cycle_mode_averaged = avg_duty_value(levels_after_mode, N_LEVELS_ITEMS);
		uint16_t duty_cycle_register_value = duty_cycle_mode_averaged;
		
		PWM_OUTPUT_load_duty_cycle_ch2(duty_cycle_register_value);
	}
	
	pending_pwm_process = false;
}

uint16_t most_common_value(uint16_t vect[], uint8_t dim) {
	size_t i, j, count;
    size_t most = 0;
	uint16_t temp = 0;
	uint16_t elem = 0;

    for(i = 0; i < dim; i++) {
        temp = vect[i];
        count = 1;
        for(j = i + 1; j < dim; j++) {
            if(vect[j] == temp) {
                count++;
            }
        }
        if (most < count) {
            most = count;
            elem = vect[i];
        }
    }
    return elem;
}

uint16_t avg_duty_value(uint16_t vect[], uint8_t dim) {
	
	uint32_t sum = 0;
	
	for (uint8_t i = 0; i < dim; i++) {
		sum = sum + vect[i];
	}
	
	return sum/dim;
}
