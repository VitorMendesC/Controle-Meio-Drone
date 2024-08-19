#include <stdio.h>
#include <stdint.h>

#define INTEGRAL_MAXIMUM_VALUE 100
#define TS 20E-3

float ADC_to_rad(uint32_t ADC_value){
	return 0.0005*ADC_value - 1.1874;
}

float trapezoid_area(float anterior_value, float current_value){
	return TS*((anterior_value+current_value)/2);
}

float clip_integral(float int_value){
	if(int_value > INTEGRAL_MAXIMUM_VALUE){
		int_value = INTEGRAL_MAXIMUM_VALUE;
	}
	if(int_value < -INTEGRAL_MAXIMUM_VALUE){
		int_value = -INTEGRAL_MAXIMUM_VALUE;
	}

	return int_value;

}

int main(int argc, int ** kargs){
    int ADC = 2500;
    float cv = 9.;
    float lv = 10.0;
    float sum = 0;
    float integral = -90;

    sum = trapezoid_area(lv, cv);
    integral = clip_integral(integral);

    printf("\nIntegral: %.1f\n", integral);
    printf("Trapezoid Area: %.3f\n", sum);
    printf("RAD: %.4f\n", ADC_to_rad(ADC));
    printf("\n");
    return 0;
}