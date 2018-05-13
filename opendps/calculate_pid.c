#include "tick.h"

#define constrain(amt, low, high)                                              \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

int16_t previous_voltage_error = 0;
int16_t voltage_integral = 0 ;
int16_t target_voltage = 0 ;
int16_t current_voltage = 0;
int required_DAC_value = 0;

uint64_t previous_ticks = 0;

//this should all be moved to the global config file
#define V_P_CONST 100
#define V_I_CONST 100
#define V_D_CONST 100

#define WINDUP_CONSTRAINTS 500
#define ALLOWABLE_OVERSHOOT 1
//...once I find it.

void pid_update_voltages(){
  uint16_t i_out_raw, v_in_raw, v_out_raw;
  hw_get_adc_values(&i_out_raw, &v_in_raw, &v_out_raw);
  (void) i_out_raw;
  (void) v_in_raw;
  current_voltage = pwrctl_calc_vout(v_out_raw);
}

void set_target_pid_voltage(int local_target_voltage){
  local_target_voltage = target_voltage;
}

int get_voltage_pid(){
  return required_DAC_value;
}

int process_voltage_pid(){ //units: millivolts.
  dbg_printf("current_voltage: %i\r\n",current_voltage);
  dbg_printf("target_voltage: %i\r\n",target_voltage);
  float timestep = 1.0/(get_ticks()-previous_ticks);
  previous_ticks = get_ticks(); //NOT OVERFLOW SAFE! I gotta fix that.

  int16_t error = current_voltage-target_voltage;
  voltage_integral = voltage_integral + error*timestep;
  // int16_t derivative = (error - previous_voltage_error)/timestep;

  required_DAC_value = V_P_CONST*error + V_I_CONST*voltage_integral;

  // previous_voltage_error = error;
  //
  // if(current_voltage > target_voltage+ALLOWABLE_OVERSHOOT){ //we really, really don't want the voltage to go
  //                                                           //much above the target. Proper PID tuning should mostly deal with that
  //                                                           //but just in case...
  //   voltage_integral = 0;
  // }
  //
  // voltage_integral = constrain(voltage_integral,-WINDUP_CONSTRAINTS,WINDUP_CONSTRAINTS);
  //
  // required_DAC_value = constrain(required_DAC_value,0,4096);
  return required_DAC_value;
}
