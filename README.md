# ⚙️ Oh Mighty Stewart!
### Gough-Stewart Platform Ball-on-Plate Stabilization & Control Project

*I know, weird repo title. It's a reference to the Frostpunk 2 trailer.*

This repository contains the complete MATLAB physics engine, control algorithms, and visualization tools for a **6-DOF Gough-Stewart Platform** configured as a **Ball-on-Plate** stabilization system. The project focuses on designing robust control architectures capable of keeping a rolling ball centered under highly chaotic and realistic wind disturbances.

---

## 🚀 Key Features & Run Modes
The project is orchestrated via a single, interactive entry script: **`RUN_ME.m`**. Launching it in MATLAB opens a menu offering 7 distinct software modes:

1.  **`[1] PID Ball Balancing`**: Closed-loop interactive visual simulation of the platform. Supports swapping between **Classic PID** and **Non-Linear Adaptive PID (NLPID)**, as well as applying live chaotic, realistic, or combined wind.
2.  **`[2] Manual Control (Game Mode)`**: Interactive manual mode where the user controls the platform's pitch, roll, and heave in real-time using keyboard inputs (`W/S/A/D`, `Arrow Keys`, `R` to Reset) to keep the ball on the plate.
3.  **`[3] Wind Analyzer`**: Visual tool that plots and analyzes the active wind force vectors, showcasing the mathematical difference between persistent realistic wind and swirling chaotic wind.
4.  **`[4] Ziegler-Nichols Auto-Tuner`**: Closed-loop automated heuristical tuner. It dampens the integrator/derivative gains and increases proportional gain until the system reaches marginal stability (sustained oscillations) to calculate the Ultimate Gain ($K_u = 2.823$) and Ultimate Period ($T_u = 0.707\text{ s}$).
5.  **`[5] Evolutionary Benchmark`**: Offline optimization and comparative benchmarking. Trains the **Island Model Genetic Algorithm** and **CMA-ES** over a deterministic robust fitness landscape to find optimal gains. Displays real-time training plots showing the best and average population scores.
6.  **`[6] Adaptive PID Benchmark`**: Validation benchmark comparing fixed analytical baseline gains against the adaptive Non-Linear PID under multiple wind seeds.
7.  **`[7] Ultimate Benchmark`**: Complete Monte Carlo stress-testing protocol evaluating all control architectures across **100 random seeds (300 simulation runs)** to rank overall performance.

---


## 💾 Automatic Gain Overwrite Feature
The software features a fully parametric, disk-updating gain loader (**`utils/load_config.m`**). 

Whenever you launch any simulation or benchmark, the program automatically reads your active physical constants (`delay_sec`, `slew_rate_deg`, `c_roll`) from `config.txt` and **calculates the mathematically optimal analytical $K_p$, $K_i$, and $K_d$ values on the fly**. If the parameters have changed, it **automatically overwrites the lines inside `config.txt` on your disk**, ensuring your configuration is always in a perfectly validated and stable state!

---

## 🛠️ Installation & Getting Started

### Prerequisites
*   MATLAB (R2018a or newer recommended).
*   Simulink (Optional, only required if running Simulink integration tools in `utils/`).
*   No external toolboxes are strictly required; the physics engine, GA, and visualization tools are written in native MATLAB scripting.

### Quick Start
1.  Clone the repository:
    ```bash
    git clone https://github.com/Sokisati/oh_mighty_stewart.git
    cd oh_mighty_stewart/StewartProject
    ```
2.  Open MATLAB, navigate to the `StewartProject` directory.
3.  Launch the menu:
    ```matlab
    RUN_ME
    ```
4.  Follow the interactive command-line prompts to launch simulations or benchmarks.

---

## ⚙️ Configuration (`config.txt`)
Physical dimensions, system constraints, wind blends, and optimization parameters can be modified directly in the central **`config.txt`** file. Major parameters include:

*   `R_base` / `R_top` : Plate radii [m]
*   `delay_sec` : Actuator loop delay / latency [seconds] (e.g. `0.04` for 40ms)
*   `slew_rate_deg` : Maximum platform speed limit [deg/s]
*   `c_roll` : Viscous rolling damping of the ball [1/s]
*   `ga_generations` / `ga_pop_size` : GA training parameters


