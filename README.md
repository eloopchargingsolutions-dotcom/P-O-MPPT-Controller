# P-O-MPPT-
Wind MPPT — Embedded P&O Controller (TI C2000)

Embedded implementation of a Perturb & Observe (P&O) MPPT controller for a
wind energy conversion system, targeting a TI C2000 (F28x series) DSP.
## How it works

- **`MPPT_Adc_Isr`** — high-priority ADC interrupt: reads raw voltage/current
  ADC channels, applies exponential smoothing (`SIG_FILTER_ALPHA`) to remove
  rectifier ripple, and checks hardware-level over-voltage/over-current
  interlocks that trip the PWM via the trip-zone (TZ) mechanism.
- **`MPPT_Execute_Background`** — timed state machine (`MPPT_EXEC_TICKS`
  paces execution relative to rotor inertia) implementing three states:
  - `SYS_STATE_BOOT` — holds minimum duty until generator voltage clears
    cut-in threshold
  - `SYS_STATE_TRACKING` — the P&O algorithm: compares power delta each
    cycle and steps duty cycle (`MPPT_SLEW_STEP`) toward the MPP
  - `SYS_STATE_FAULT_BRAKE` — engages dump-load relay and holds until
    voltage recovers below hysteresis threshold, then resets
