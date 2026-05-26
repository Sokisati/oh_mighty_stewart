# Stewart Platform - MATLAB Project

## Quick Start

1. Open MATLAB
2. Set **Current Folder** to the `StewartProject` directory
3. Open `RUN_ME.m` → press **F5** (or type `>> RUN_ME`)
4. Choose a run mode when prompted:
   - **[1]** → Fast MATLAB 3D animation (no Simscape needed)
   - **[2]** → Simscape Multibody model with Mechanics Explorer

---

## File Structure

```
StewartProject/
├── RUN_ME.m           <- START HERE
├── stewart_setup.m    <- Geometry, inverse kinematics, motion profile
├── stewart_sim.m      <- MATLAB 3D animation (Mode 1)
├── build_simulink.m   <- Simscape Multibody model builder (Mode 2)
└── README.md
```

---

## Platform Parameters

| Parameter | Value |
|-----------|-------|
| Base plate radius | 200 mm |
| Top plate radius | 120 mm |
| Home height | 200 mm |
| Piston outer diameter | 20 mm |
| Material | Steel (rho = 7850 kg/m^3) |
| Simulation duration | 10 s |

## Demo Motion Profile

The platform executes a combined motion:

| Motion | Amplitude | Frequency |
|--------|-----------|-----------|
| Heave (vertical) | ±25 mm | 0.40 Hz |
| Roll | ±6 deg | 0.35 Hz |
| Pitch | ±8 deg | 0.25 Hz |

---

## Inverse Kinematics

At each time step, given platform pose `[x, y, z, roll, pitch, yaw]`,
the six leg lengths are computed as:

```
L_k = | R * b_k + t - a_k |
```

Where:
- `R`   : ZYX Euler rotation matrix
- `b_k` : k-th top attachment point (body frame)
- `t`   : platform center position (world frame)
- `a_k` : k-th base attachment point (world frame)

---

## Requirements

- MATLAB R2020a or later (written targeting R2025b)
- **Mode 1:** Base MATLAB only (no additional toolboxes)
- **Mode 2:** Simscape + Simscape Multibody toolboxes
