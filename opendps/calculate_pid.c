#include "tick.h"
#include <dac.h>

#define constrain(amt, low, high)                                              \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

int16_t previous_voltage_error = 0;
int16_t voltage_integral = 0 ;
int16_t target_voltage = 0 ;
int16_t current_voltage = 0;
uint16_t required_DAC_value = 0;

uint64_t previous_ticks = 0;

//this should all be moved to the global config file
#define V_OFFSET_CONST 0.8
#define V_P_CONST 0.6
#define V_I_CONST 1
#define V_D_CONST 0.2//0.2

#define WINDUP_CONSTRAINTS 5000
#define ALLOWABLE_OVERSHOOT 100
//...once I find it.

void pid_update_voltages(){
  uint16_t i_out_raw, v_in_raw, v_out_raw;
  hw_get_adc_values(&i_out_raw, &v_in_raw, &v_out_raw);
  (void) i_out_raw;
  (void) v_in_raw;
  current_voltage = pwrctl_calc_vout(v_out_raw);
}

void set_target_pid_voltage(int new_target_voltage){
  dbg_printf("voltage set to: %u\r\n",new_target_voltage);
  if(new_target_voltage != target_voltage){
    voltage_integral = 0;
    target_voltage = new_target_voltage;
  }
}

int process_voltage_pid(){ //units: millivolts.

  float timestep = 1.0/(get_ticks()-previous_ticks);
  previous_ticks = get_ticks(); //NOT OVERFLOW SAFE! I gotta fix that.

  int16_t error = target_voltage-current_voltage;

  voltage_integral = voltage_integral + error*timestep;
  int16_t derivative = (error - previous_voltage_error)/timestep;

  required_DAC_value = V_OFFSET_CONST*target_voltage + V_P_CONST*error + V_I_CONST*voltage_integral + V_D_CONST*derivative;

  previous_voltage_error = error;

  // if(current_voltage > target_voltage+ALLOWABLE_OVERSHOOT){ //we really, really don't want the voltage to go
  //                                                           //much above the target. Proper PID tuning should mostly deal with that
  //                                                           //but just in case...
  //   voltage_integral = 0;
  // }

  voltage_integral = constrain(voltage_integral,-WINDUP_CONSTRAINTS,WINDUP_CONSTRAINTS);
  required_DAC_value = constrain(required_DAC_value,0,30000);

  dbg_printf("timestep: %u\r\n",(uint16_t)timestep*100.0);
  dbg_printf("error: %u\r\n",abs(error));
  dbg_printf("voltage_integral: %u\r\n",abs(voltage_integral));

  dbg_printf("current_voltage: %u\r\n",current_voltage);
  dbg_printf("target_voltage: %u\r\n",target_voltage);
  dbg_printf("required_DAC_value: %u\r\n",required_DAC_value);

  return required_DAC_value;
}

void pid_update_dac_value(){
  DAC_DHR12R1 = pwrctl_calc_vout_dac(required_DAC_value);
}
