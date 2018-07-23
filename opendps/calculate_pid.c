#include "tick.h"
#include <dac.h>

#define constrain(amt, low, high)                                              \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

int16_t previous_voltage_error = 0;
int16_t voltage_integral = 0;
int16_t target_voltage = 0;
int16_t current_voltage = 0;


uint16_t current_amps = 0; //you'll notice I'm using "current", "amps", and "I" in varying places.
                           // it's cuz I'm an idiot.
                           // should probably fix that.

                           // the variable names, that is.
bool current_mode = 0;

int16_t previous_current_error = 0;
int16_t current_integral = 0;
int16_t target_current = 0;

uint16_t required_DAC_value = 0;

uint64_t previous_ticks = 0;

int pid_just_reset = 1;


void dummy_debug(const char *fmt, ...);

#define pid_debug dbg_printf
// #define pid_debug dummy_debug

//this should all be moved to the global config file
#define V_OFFSET_CONST_DEFAULT 0.95 // this offsets the whole pid on startup
                                    // to get closer to the desired point
                                    // in order for us to save integral windup
                                    // for better response
                                    // All the control systems engineers are probably
                                    // killing themselves laughing at how stupid this is.
                                    // But it seems to work okay.
                                    // burma shave
#define V_P_CONST_DEFAULT 0.2
#define V_I_CONST_DEFAULT 0.15
#define V_D_CONST_DEFAULT 0.1//0.2

#define C_P_CONST_DEFAULT 0.3
#define C_I_CONST_DEFAULT 1
#define C_D_CONST_DEFAULT 0.1//0.2

#define WINDUP_CONSTRAINTS_DEFAULT 30000
#define ALLOWABLE_OVERSHOOT_DEFAULT 100
//...once I find it.

float V_OFFSET_CONST = V_OFFSET_CONST_DEFAULT;
float V_P_CONST = V_P_CONST_DEFAULT;
float V_I_CONST = V_I_CONST_DEFAULT;
float V_D_CONST = V_D_CONST_DEFAULT;

float C_P_CONST = C_P_CONST_DEFAULT;
float C_I_CONST = C_I_CONST_DEFAULT;
float C_D_CONST = C_D_CONST_DEFAULT;

uint16_t WINDUP_CONSTRAINTS = WINDUP_CONSTRAINTS_DEFAULT;
uint16_t ALLOWABLE_OVERSHOOT = ALLOWABLE_OVERSHOOT_DEFAULT;


void update_pid_tuning(uint16_t new_V_P_CONST,uint16_t new_V_I_CONST,uint16_t new_V_D_CONST){
  V_P_CONST = new_V_P_CONST/100.0;
  V_I_CONST = new_V_I_CONST/100.0;
  V_D_CONST = new_V_D_CONST/100.0;
}

void pid_update_voltages(){
  uint16_t i_out_raw, v_in_raw, v_out_raw;
  hw_get_adc_values(&i_out_raw, &v_in_raw, &v_out_raw);
  current_amps = pwrctl_calc_iout(i_out_raw);
  (void) v_in_raw;
  current_voltage = pwrctl_calc_vout(v_out_raw);
}

void reset_pid(){
  voltage_integral = 0;
  current_integral = 0;
  pid_update_voltages();
  previous_ticks = get_ticks();
}

void set_target_pid_voltage(int new_target_voltage){
  pid_debug("voltage set to: %u\r\n",new_target_voltage);
  if(new_target_voltage != target_voltage){
    voltage_integral = 0;
    target_voltage = new_target_voltage;
  }
}


void set_target_pid_current(int new_target_current){
  pid_debug("target_current: %u\r\n",new_target_current);
  if(new_target_current != target_current){
    voltage_integral = 0;
    target_current = new_target_current;
  }
}

int process_pid_algorithms(){ //units: millivolts.

  float timestep = 1.0/(get_ticks()-previous_ticks);
  previous_ticks = get_ticks(); //NOT OVERFLOW SAFE! I gotta fix that.

  /////////////////VOLTAGE//////////////
  int16_t voltage_error = target_voltage-current_voltage;

  int16_t voltage_derivative = (voltage_error - previous_voltage_error)/timestep;
  /////////////////Amps//////////////
  int16_t current_error = target_current-current_amps;
  current_integral = current_integral + current_error*timestep;
  int16_t current_derivative = (current_error - previous_current_error)/timestep;
  if(pwrctl_vout_enabled()){
    voltage_integral = voltage_integral + voltage_error*timestep;
  }
  else{
    voltage_integral = 0;
    previous_voltage_error = current_error;
  }
  /////////////////CC/CV HYSTERESIS/////
  if(current_amps > target_current){
    current_mode = 1;
  }
  if(current_mode && voltage_error < 100){
    current_mode = 0;
  }
  ///////////////
  if(current_voltage > target_voltage+ALLOWABLE_OVERSHOOT && voltage_integral > 100){ //we really, really don't want the voltage to go
                                                            //much above the target. Proper PID tuning should mostly deal with that
                                                            //but just in case...
    voltage_integral -= 50;
  }

  if(current_mode){
    required_DAC_value = V_OFFSET_CONST*target_voltage + V_P_CONST*current_error + V_I_CONST*current_integral + V_D_CONST*current_derivative;
  }
  else{
    required_DAC_value = V_OFFSET_CONST*target_voltage + V_P_CONST*voltage_error + V_I_CONST*voltage_integral + V_D_CONST*voltage_derivative;
  }
  previous_voltage_error = voltage_error;


  voltage_integral = constrain(voltage_integral,-WINDUP_CONSTRAINTS,WINDUP_CONSTRAINTS);
  required_DAC_value = constrain(required_DAC_value,0,65000);

  pid_debug("timestep: %u\r\n",(uint16_t)timestep*100.0);
  pid_debug("error: %u\r\n",abs(voltage_error));
  pid_debug("voltage_integral: %u\r\n",abs(voltage_integral));

  pid_debug("current_voltage: %u\r\n",current_voltage);
  pid_debug("target_current: %u\r\n",target_current);
  pid_debug("target_voltage: %u\r\n",target_voltage);
  pid_debug("required_DAC_value: %u\r\n",required_DAC_value);
  pid_debug("current_mode: %u\r\n",current_mode);
  pid_debug("PID: %u, %u, %u", (int)(V_P_CONST*100),(int)(V_I_CONST*100),(int)(V_D_CONST*100));

  pid_just_reset = 0;

  return required_DAC_value;
}

void pid_update_dac_value(){
  DAC_DHR12R1 = pwrctl_calc_vout_dac(required_DAC_value);
}
