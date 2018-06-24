int process_voltage_pid(float current_voltage, float target_voltage);

extern float V_OFFSET_CONST;
extern float V_P_CONST;
extern float V_I_CONST;
extern float V_D_CONST;

extern float C_P_CONST;
extern float C_I_CONST;
extern float C_D_CONST;

extern uint16_t WINDUP_CONSTRAINTS;
extern uint16_t ALLOWABLE_OVERSHOOT;
