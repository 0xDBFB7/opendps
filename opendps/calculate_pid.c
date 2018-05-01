#define constrain(amt, low, high)                                              \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

float previous_voltage_error = 0;
float voltage_integral = 0 ;

//this should all be moved to the global config file
#define V_P_CONST 10
#define V_I_CONST 0.5
#define V_D_CONST 0.5

#define WINDUP_CONSTRAINTS 100 //not sure what the units of our input will be.
#define ALLOWABLE_OVERSHOOT 0.01
//...once I find it.

float process_voltage_pid(float current_voltage,float target_voltage){ //float probably isn't right.
  float timestep = 1.0/cycles_per_second; //I'm not yet sure what timers we have running on the DPS.

  float required_DAC_value = 0;

  float error = current_voltage-target_voltage;
  voltage_integral = voltage_integral + error*timestep;
  float derivative = (error - previous_error)/timestep;

  required_pump_duty = V_P_CONST*error + V_I_CONST*integral + V_D_CONST*derivative;

  previous_error = error;

  if(current_voltage > target_voltage+ALLOWABLE_OVERSHOOT){ //we really, really don't want the voltage to go
                                                            //much above the target. Proper PID tuning should mostly deal with that
                                                            //but just in case...
    voltage_integral = 0;
  }

  voltage_integral = constrain(voltage_integral,WINDUP_CONSTRAINTS,WINDUP_CONSTRAINTS);

  required_DAC_value = constrain(required_DAC_value,0,200);
  return required_DAC_value;
}
