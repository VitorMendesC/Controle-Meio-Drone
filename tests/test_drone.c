#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define TS 20E-3
#define MAX_SLOPE_RAD_MS 0.0001

struct r_data{
    float slope_rad_ms;
    float start_pos;
    float set_point;

    uint32_t t0, t1, t2, t3, t4, t5;
};
typedef struct r_data route_data;

void set_route_struct(route_data *d, float start_pos, float set_point, uint32_t time_constant_rad_ms){
    float slope_rad_ms;

    if(set_point-start_pos >= 0.){
        slope_rad_ms = MAX_SLOPE_RAD_MS;
    }
    else{
        slope_rad_ms = -MAX_SLOPE_RAD_MS;
    }

    d->slope_rad_ms = slope_rad_ms;
    d->start_pos = start_pos;
    d->set_point = set_point;

    //d->t0 = HAL_GetTick();                        //UNCOMMENT ON CUBEIDE
    d->t0=0.;
    d->t1 = (uint32_t) ((set_point-start_pos)/slope_rad_ms);
    d->t2 = d->t1 + time_constant_rad_ms;
    d->t3 = d->t2 + 2*d->t1;
    d->t4 = d->t3 + time_constant_rad_ms;
    d->t5 = d->t4 + d->t1;
    return;
}

float route_planner(route_data *d, uint32_t t){
    //uint32_t t = HAL_GetTick() - d->t0;           //UNCOMMENT ON CUBEIDE
    float relative_set_point = 0;

    if(t >= 0 && t < d->t1){
        relative_set_point = d->start_pos + d->slope_rad_ms*t;
    }
    else if(t >= d->t1 && t <d->t2){
        relative_set_point = d->set_point;
    }
    else if(t >= d->t2 && t < d->t3){
        relative_set_point = d->set_point - d->slope_rad_ms*(t - d->t2);
    }
    else if (t >= d->t3 && t < d->t4){
        relative_set_point = 2*d->start_pos - d->set_point;
    }
    else if (t >= d->t4 && t < d->t5){
        relative_set_point = 2*d->start_pos - d->set_point + d->slope_rad_ms*(t - d->t4);
    }
    else{
        relative_set_point = d->start_pos;
    }
    
    return relative_set_point;
}

float ADC_to_rad(uint32_t ADC_value){
	return 0.0005*ADC_value - 1.1874;
}

float pwm_to_f_right(float PWM){
    return 0.0301*PWM - 0.3834;
}

float pwm_to_f_left(float PWM){
    return 0.0316*PWM - 0.3992;
}

float f_to_pwm_rigth(float f){
    return -7.5039*pow(f,2) + 51.102*f + 7.8998;
}

float f_to_pwm_left(float f){
    return -6.7909*pow(f,2) + 48.729*f + 7.6053;
}

float clip_float(float int_value, float min, float max){
	if(int_value > max){
		int_value = max;
	}
	if(int_value < min){
		int_value = min;
	}

	return int_value;
}


uint8_t pData[16];
uint8_t first_run = 1;
route_data d;
float angle_rad_setpoint = 10;
float angle_rad = 0;
uint32_t t1, t2, t3;
FILE *fptr;

int main(int argc, int ** kargs){
    int ADC = 2500;
    float error = -10.;
    route_data route_d;

    fptr = fopen("route.txt", "w");


    printf("\n");
    sprintf(pData, "E%% = %f \r\n", f_to_pwm_rigth(0.9711));
    printf("%s", pData);
    printf("PWM[%%] = %.4f%%;    F[N] = %.4f\n",45., pwm_to_f_right(45.));
    printf("PWM[%%] = %.4f%%;    F[N] = %.4f\n",40., pwm_to_f_left(40.));
    printf("PWM[%%] = %.4f%%;    F[N] = %.4f\n",f_to_pwm_rigth(0.9711), 0.9711);
    printf("PWM[%%] = %.4f%%;    F[N] = %.4f\n",f_to_pwm_rigth(0.8648), 0.8648);
    printf("RAD = %.4f\n", ADC_to_rad(ADC));
    printf("\n");

    set_route_struct(&route_d, 0, 0.5, 2000);
    float sp = 0;
    for(float t = 0.; t<40; t= t + 0.01){
        sp = route_planner(&route_d, t*1000);
        fprintf(fptr, "\n%f    %f", t, sp);
    }

    fclose(fptr);
    return 0;
}