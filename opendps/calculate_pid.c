#include "tick.h"

#define constrain(amt, low, high)                                              \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

float previous_voltage_error = 0;
float voltage_integral = 0 ;
uint64_t previous_ticks = 0;

//this should all be moved to the global config file
#define V_P_CONST 1
#define V_I_CONST 0
#define V_D_CONST 0

#define WINDUP_CONSTRAINTS 100 //not sure what the units of our input will be.
#define ALLOWABLE_OVERSHOOT 0.01
//...once I find it.

int process_voltage_pid(float current_voltage, float target_voltage){ //float probably isn't right.
  float timestep = 1.0/(get_ticks()-previous_ticks);
  previous_ticks = get_ticks(); //NOT OVERFLOW SAFE! I gotta fix that.

  float required_DAC_value = 0;

  float error = current_voltage-target_voltage;
  voltage_integral = voltage_integral + error*timestep;
  float derivative = (error - previous_voltage_error)/timestep;

  required_DAC_value = V_P_CONST*error + V_I_CONST*voltage_integral + V_D_CONST*derivative;

  previous_voltage_error = error;

  if(current_voltage > target_voltage+ALLOWABLE_OVERSHOOT){ //we really, really don't want the voltage to go
                                                            //much above the target. Proper PID tuning should mostly deal with that
                                                            //but just in case...
    voltage_integral = 0;
  }

  voltage_integral = constrain(voltage_integral,WINDUP_CONSTRAINTS,WINDUP_CONSTRAINTS);

  required_DAC_value = constrain(required_DAC_value,0,4096);
  return required_DAC_value;
}
