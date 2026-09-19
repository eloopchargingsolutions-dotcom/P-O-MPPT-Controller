#include "mppt_controller.h"

/* --- LOCAL PERSISTENT REGS --- */
static tMpptState eMpptState = SYS_STATE_BOOT;
static uint16_t u16TickScheduler = 0;

static float fV_Gen = 0.0f;
static float fI_Gen = 0.0f;
static float fP_Prev = 0.0f;
static float fV_Prev = 0.0f;
static int32_t s32CurrentDuty = 2048;

/**
 * @brief Configuration of hardware protection registers and initial hardware states
 */
void MPPT_Init(void) {
    EALLOW;
    // Bind hardware Trip-Zone: Force EPWM1A output LOW immediately when TZ One-Shot trips
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LOW; 
    
    // Configure GPIO7 as output for external hardware dump-load relay
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1; 
    EDIS;

    eMpptState = SYS_STATE_BOOT;
    s32CurrentDuty = MPPT_MIN_DUTY;
    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)s32CurrentDuty;
}

/**
 * @brief High-priority ADC End-of-Conversion Interrupt. Handles physical calculations and limits.
 */
interrupt void MPPT_Adc_Isr(void) {
    float fV_RawSample;
    float fI_RawSample;
    float fI_Volts;

    // Direct register extraction from C2000 hardware channels
    fV_RawSample = (float)AdcaResultRegs.ADCRESULT0 * MPPT_V_SCALE;
    
    fI_Volts = ((float)AdcbResultRegs.ADCRESULT0 * (3.3f / 65535.0f)) - MPPT_I_OFFSET_V;
    fI_RawSample = fI_Volts / 0.040f;
    if (fI_RawSample < 0.0f) {
        fI_RawSample = 0.0f;
    }

    // Dynamic smoothing of high frequency commutation ripples
    fV_Gen = (SIG_FILTER_ALPHA * fV_RawSample) + ((1.0f - SIG_FILTER_ALPHA) * fV_Gen);
    fI_Gen = (SIG_FILTER_ALPHA * fI_RawSample) + ((1.0f - SIG_FILTER_ALPHA) * fI_Gen);

    // Hardware Over-Limit Interlock Protection Check
    if ((fV_Gen >= O_VOLT_CRITICAL) || (fI_Gen >= O_CURR_CRITICAL)) {
        eMpptState = SYS_STATE_FAULT_BRAKE;
        EALLOW;
        EPwm1Regs.TZFRC.bit.OST = 1; // Assert instantaneous hardware-level One-Shot Trip zone override
        EDIS;
    }

    // Acknowledge interrupt to clear TI PIE block
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/**
 * @brief Background task execution. Contains timed state machine implementation.
 */
void MPPT_Execute_Background(void) {
    float fP_Now;
    float fdP;
    float fdV;
    int32_t s32Dir = 0;

    switch (eMpptState) {
        
        case SYS_STATE_BOOT:
            GpioDataRegs.GPACLEAR.bit.GPIO7 = 1; 
            s32CurrentDuty = MPPT_MIN_DUTY;
            EPwm1Regs.CMPA.bit.CMPA = (uint16_t)s32CurrentDuty;

            if (fV_Gen >= TURBINE_CUT_IN_V) {
                eMpptState = SYS_STATE_TRACKING;
                u16TickScheduler = 0;
            }
            break;

        case SYS_STATE_TRACKING:
            u16TickScheduler++;
            if (u16TickScheduler < MPPT_EXEC_TICKS) {
                return; // Pacing constraint: Yield processing execution window to accommodate rotor inertia
            }
            u16TickScheduler = 0;

            fP_Now = fV_Gen * fI_Gen;
            fdP = fP_Now - fP_Prev;
            fdV = fV_Gen - fV_Prev;

            if (fP_Now > 10.0f) {
                if (fdP > 4.0f) {
                    // Buck-Boost Mapping Direction: Increasing duty reduces generator terminal voltage
                    s32Dir = (fdV > 0.0f) ? 1 : -1;
                } 
                else if (fdP < -4.0f) {
                    s32Dir = (fdV > 0.0f) ? -1 : 1;
                }
                s32CurrentDuty += (s32Dir * MPPT_SLEW_STEP);
            } 
            else {
                s32CurrentDuty -= 10; // Drop load if wind drops out completely to ease blade aerodynamic restart
                if (fV_Gen < (TURBINE_CUT_IN_V - 5.0f)) {
                    eMpptState = SYS_STATE_BOOT;
                }
            }

            // Boundary clamping controls
            if (s32CurrentDuty > MPPT_MAX_DUTY) s32CurrentDuty = MPPT_MAX_DUTY;
            if (s32CurrentDuty < MPPT_MIN_DUTY) s32CurrentDuty = MPPT_MIN_DUTY;

            EPwm1Regs.CMPA.bit.CMPA = (uint16_t)s32CurrentDuty;

            fP_Prev = fP_Now;
            fV_Prev = fV_Gen;
            break;

        case SYS_STATE_FAULT_BRAKE:
            GpioDataRegs.GPASET.bit.GPIO7 = 1; // Engage hardware dump-load braking resistor relay loop
            s32CurrentDuty = MPPT_MIN_DUTY;
            EPwm1Regs.CMPA.bit.CMPA = (uint16_t)s32CurrentDuty;

            // Enforce safe recovery hysteresis before resetting Trip Zone clears
            if (fV_Gen < RECOVERY_VOLT_HYST) {
                EALLOW;
                EPwm1Regs.TZCLR.bit.OST = 1; // Clear hardware one-shot trip flags safely
                EDIS;
                eMpptState = SYS_STATE_BOOT;
            }
            break;

        default:
            eMpptState = SYS_STATE_BOOT;
            break;
    }
}
