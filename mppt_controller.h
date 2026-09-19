#ifndef MPPT_CONTROLLER_H_
#define MPPT_CONTROLLER_H_

#include "F28x_Project.h"

/* --- SENSOR CALIBRATION MATRIX --- */
// V_SCALE = (3.3V / 65535) * (220.0V / 3.3V)
#define MPPT_V_SCALE          0.003357065f  
// I_SCALE = (3.3V / 65535) / 0.040 V/A
#define MPPT_I_SCALE          0.001258870f  
#define MPPT_I_OFFSET_V       1.65f

/* --- EPWM TIME-BASE RESOLUTION CONSTRAINTS (12-bit / 20kHz TBPRD) --- */
#define MPPT_PWM_MAX          4095
#define MPPT_MIN_DUTY         410   // 10% lower limit
#define MPPT_MAX_DUTY         3685  // 90% upper limit
#define MPPT_SLEW_STEP        15    // Max dDuty per iteration

/* --- OPERATION AND LIMIT THRESHOLDS --- */
#define O_VOLT_CRITICAL       220.0f
#define O_CURR_CRITICAL       30.0f
#define RECOVERY_VOLT_HYST    165.0f
#define TURBINE_CUT_IN_V      40.0f

// Filter coefficient for rectified 3-phase slot ripple mitigation
#define SIG_FILTER_ALPHA      0.12f 

// Execution timing divisor (assumes background call pace matches scheduler)
#define MPPT_EXEC_TICKS       120   

typedef enum {
    SYS_STATE_BOOT = 0,
    SYS_STATE_TRACKING,
    SYS_STATE_FAULT_BRAKE
} tMpptState;

/* --- FUNCTION DECLARATIONS --- */
void MPPT_Init(void);
void MPPT_Execute_Background(void);
interrupt void MPPT_Adc_Isr(void);

#endif /* MPPT_CONTROLLER_H_ */
