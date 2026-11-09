# Mamdani Fuzzy Logic Control for DC Motor Speed

This project builds and simulates a separately excited DC motor speed-control system. It compares a Mamdani fuzzy logic controller (FLC) with a conventional PID controller and open-loop voltage control under reference changes and a load-torque disturbance.

## Experiment design

- The motor starts with a 100 rad/s speed reference. A 1 N·m load is applied at 3 s, and the reference increases to 150 rad/s at 5 s.
- The FLC uses speed error and change in error as inputs, with seven triangular membership functions per input, 49 fuzzy rules, min–max inference, centroid defuzzification, and a saturated incremental voltage output.
- All three control strategies use the same plant and a 0.01 s sample time. The PID baseline uses `Kp = 2`, `Ki = 5`, and `Kd = 0.05`.

## Recorded results

| Measure | Mamdani FLC | PID | Open loop |
| --- | ---: | ---: | ---: |
| Rise time, 0–100 rad/s (s) | 0.500 | 0.120 | 0.260 |
| Settling time, 0–100 rad/s (s) | 0.970 | 1.050 | 0.710 |
| Overshoot, 0–100 rad/s (%) | 0.007 | 5.020 | 3.945 |
| Speed dip at load step (rad/s) | 3.937 | 3.768 | 6.580 |
| Steady-state error under load (rad/s) | 0.044 | 0.000 | 2.857 |
| Settling time, 100–150 rad/s (s) | 0.870 | 1.070 | — |
| Overshoot, 100–150 rad/s (%) | 0.306 | 4.530 | — |
| Integral absolute error, 0–8 s | 44.543 | 22.118 | 43.359 |

The FLC produces very low overshoot in both reference steps. The PID has the lowest reported integral absolute error and zero steady-state error under the load disturbance, while open-loop control is more affected by the disturbance. These results show the trade-offs of the three controllers for this simulated motor and parameter set.

## Requirements and run

Requires MATLAB R2018b or later, Simulink, Fuzzy Logic Toolbox, and Control System Toolbox.

From the repository root, run:

```matlab
build_Exp3_Simulink
```

The script creates or refreshes the Simulink model, runs the simulation, and exports the figures to `Exp3_Simulink/figures/`. The existing `.slx` model and `.fis` controller are included for inspection.

## Repository contents

- `build_Exp3_Simulink.m` — model construction, simulation, metrics, and figure export.
- `Exp3_Simulink/Exp3_Mamdani_FLC_DCmotor.slx` — Simulink model with FLC, PID, and open-loop paths.
- `Exp3_Simulink/DCmotorFLC.fis` — fuzzy inference system.
- `Exp3_Simulink/figures/` — membership functions, control surface, response plots, and model diagrams.

## Scope

The reported figures are simulation results; this project does not include hardware testing. No software license is specified in this repository.
