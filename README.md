# Adaptive Active Suspension Simulation Framework

This project presents a modular MATLAB/Simulink simulation environment for evaluating various control strategies in active suspension systems. The study is grounded on a 2-DOF quarter-car model and investigates the performance of **PID**, **SMC**, **LQR**, and **H-Infinity** controllers against a **passive suspension baseline**. Emphasis is placed on surface adaptability, controller tuning, and the integration of modern AI-based perception and communication technologies.

---

## Objectives

- Evaluate multiple active control strategies for vehicle suspension.
- Compare system responses to both synthetic and real road profiles.
- Observe controller robustness under gain and speed variations.
- Develop a gain surface interpolation approach for adaptive control.
- Extend the simulation with AI-driven surface recognition and V2V communication.

---

## Methodology

### 1. Modeling

- The system is modeled as a **2-DOF quarter-car suspension** setup.
- Equations of motion are derived and transformed into the **frequency domain**.
- Implemented using **MATLAB/Simulink**, with state-space representation shared across all controllers.

### 2. Road Input Generation

- **Synthetic ISO profiles** are generated using power spectral density (PSD) standards to simulate class A-F roughness.
- **Real-world elevation data** is imported from the **OpenCRG** dataset.

### 3. Performance Evaluation

- The simulation plots **sprung and unsprung mass displacements** against road distance.
- **Fixed-speed, fixed-gain** tests compare baseline controller performance.
- Gain variation tests sweep each controller's gain from **0.1x to 10x** the base value to study deviations from the 1x benchmark.
- Speed sensitivity is evaluated from **10 km/h to 90 km/h**, holding gain constant.

### 4. Adaptive Gain Interpolation

- A **3D gain surface** is constructed using:
  - Speed
  - Vertical displacement
  - Controller gain
- Using **bilinear interpolation**, the controller retrieves optimal gains for the given operating condition.
- This forms the basis of the **proposed adaptive gain strategy**.

### 5. AI Integration (Future Work)

- A neural network-based **image recognition module** identifies the road type and condition.
- **Vehicle-to-vehicle (V2V)** communication allows compatible vehicles to share road quality data.
- Combined with vehicle dynamics and slope detection, the system provides **real-time adaptive gain values** to the controller.
- This culminates in a **robust and intelligent active suspension setup** tailored to real-world driving scenarios.

---

## Project Structure

```plaintext
simulation/
├── algo/                  # State-space computation
├── controller/            # All controller implementations (PID, SMC, etc.)
├── init/                  # Parameter and gain initializers
├── scripts/               # Batch figure conversion or automation tools
├── tests/
│   ├── iso_gain_sweep/    # ISO-based gain variation tests
│   ├── crg_tests/         # Road profile tests (OpenCRG API)
│   ├── gain_interp/       # Gain Surface Bilinear Interpolation tests
│   └── tests_common/      # Reusable test benches for gain/speed interpolation
│       ├──controller/     # Controller comparison (fixed gain, speed)
│       ├──gain_compare/   # Gain Comparison for controller (fixed vs adaptive)
│       ├──gain_sweep/     # Gain Sweep performance for controller (fixed speed)
│       └──speed_sweep/    # Speed Sweep performance for controller (fixed gain)
└── utils/                 # Metrics, plotting, road profile generation, etc.
```

---

## Results
This section presents various simulation results, highlighting different stages of performance evaluation for the active suspension system.

### 1. Road Profile and Suspension Displacements (from `tests_common/controller`)
These plots illustrate the controller responses to varied road conditions by showing the road elevation input and the corresponding sprung and unsprung mass displacements. This includes both real-world data from the **OpenCRG** dataset and synthetic **ISO-8608** profiles.

| 1.1. Road Profile - CRG Country Road & Displacements | 1.2. Road Profile - ISO Class F & Displacements |
| :----------------------------------------------: | :----------------------------------------: |
| ![Simulation - CRG_country_road](simulation/figures/svg/tests_common/controller/crg_sim_q_car/Simulation%20-%20CRG_country_road.svg) | ![ISO Class F](simulation/figures/svg/tests_common/controller/iso_sim_q_car/ISO%20Class%20F.svg) |
| Fig: The plot shows the road elevation data for a country road profile (top), followed by the sprung mass displacement (middle) and unsprung mass displacement (bottom) over a distance of 1000m. The multiple lines in the displacement plots represent different control strategies or variations. | Fig: This figure displays a synthetic ISO Class F road profile (top) and the resulting sprung (middle) and unsprung (bottom) mass displacements. ISO Class F represents a very rough road, allowing for the evaluation of controller performance under severe conditions. |

### 2. Gain Sweep Performance (from `tests_common/gain_sweep`)
These tests evaluate how robustly the controllers perform under varying gain values. The gain is swept from **0.1x to 10x** the base value to study deviations from the **1x benchmark**, covering both **ISO-based** and **CRG-based** scenarios.

| 2.1. Gain Sweep - H-infinity - CRG Country Road | 2.2. Gain Sweep - PID - ISO-F |
| :----------------------------------------------: | :-----------------------------: |
| ![Gain Sweep - Hinf - CRG_country_road](simulation/figures/svg/tests_common/gain_sweep/crg_gain_sweep/CRG_country_road/Gain%20Sweep%20-%20Hinf%20-%20CRG_country_road.svg) | ![Gain Sweep - PID - ISO-F](simulation/figures/svg/tests_common/gain_sweep/iso_gain_sweep/PID/Gain%20Sweep%20-%20PID%20-%20ISO-F.svg) |
| Fig: This graph illustrates the performance of the H-infinity controller across a range of gain variations when subjected to the CRG country road profile. The road profile is shown on top, with the sprung and unsprung mass displacements below, showing how the controller reacts to changes in its internal gains. | Fig: This plot shows the PID controller's performance under various gain settings against an ISO-F road profile. The top subplot shows the road profile, and the bottom two display the sprung and unsprung mass displacements, indicating the stability and effectiveness of the PID controller as gains are swept. |

### 3. Speed Sweep Performance (from `tests_common/speed_sweep`)
These tests assess the controllers' robustness to varying vehicle speeds, ranging from **10 km/h to 90 km/h**, while keeping the controller gain constant. This helps in understanding the system's adaptability to different driving conditions across both **CRG** and **ISO** road types.

| 3.1. Speed Sweep - SMC - CRG Country Road | 3.2. Speed Sweep - SMC - ISO-D |
| :----------------------------------------------: | :----------------------------: |
| ![Speed Sweep - SMC - CRG_country_road](simulation/figures/svg/tests_common/speed_sweep/crg_speed_sweep/CRG_country_road/Speed%20Sweep%20-%20SMC%20-%20CRG_country_road.svg) | ![Speed Sweep - SMC - D](simulation/figures/svg/tests_common/speed_sweep/iso_speed_sweep/D/Speed%20Sweep%20-%20SMC%20-%20D.svg) |
| Fig: The performance of the Sliding Mode Controller (SMC) is shown here as vehicle speed is varied on the CRG country road. The road profile is consistent across speeds (top), but the sprung and unsprung mass displacements (middle and bottom) highlight how the SMC reacts differently at various speeds. | Fig: This figure demonstrates the SMC's performance across different speeds when encountering an ISO-D road profile. The top graph shows the ISO-D road, and the lower graphs illustrate how sprung and unsprung mass displacements change with speed, providing insight into the controller's robustness. |

### 4. Gain Interpolation Surfaces (PID Example - from `gain_interp`)
These 3D gain surfaces illustrate the adaptive control approach, where optimal gains for the PID controller are interpolated based on vehicle speed and vertical displacement. This demonstrates the framework's ability to adjust controller parameters in real-time for improved performance across varying operating conditions.

#### 4.1. PID Controller Gain Surfaces

| Kd vs Speed & Displacement (PID) | Ki vs Speed & Displacement (PID) | Kp vs Speed & Displacement (PID) |
| :----------------------------------------------: | :----------------------------------------------: | :----------------------------------------------: |
| ![Kd vs Speed & Displacement (PID)](simulation/figures/svg/gain_interp/gain_interp/PID/Kd%20vs%20Speed%20&%20Displacement%20(PID).svg) | ![Ki vs Speed & Displacement (PID)](simulation/figures/svg/gain_interp/gain_interp/PID/Ki%20vs%20Speed%20&%20Displacement%20(PID).svg) | ![Kp vs Speed & Displacement (PID)](simulation/figures/svg/gain_interp/gain_interp/PID/Kp%20vs%20Speed%20&%20Displacement%20(PID).svg) |
| Fig: Interpolated gain surface for the proportional derivative (Kd) parameter of the PID controller based on vehicle speed and vertical displacement. | Fig: Interpolated gain surface for the integral (Ki) parameter of the PID controller based on vehicle speed and vertical displacement. | Fig: Interpolated gain surface for the proportional (Kp) parameter of the PID controller based on vehicle speed and vertical displacement. |

### 5. Gain Comparison - Fixed vs Adaptive (from `tests_common/gain_compare`)
This section compares the performance of controllers using fixed gains versus those utilizing the adaptive gain interpolation strategy. These results highlight the advantages of adapting controller parameters dynamically based on road conditions and vehicle state.

| 5.1. Performance Comparison: PID - CRG Country Road (70 km/h) | 5.2. Performance Comparison: SMC - ISO-F (70 km/h) |
| :----------------------------------------------: | :----------------------------------------: |
| ![Gain Comparison - PID - CRG Country Road](simulation/figures/svg/tests_common/gain_compare/crg_gain_compare/CRG_country_road/PID/Gain%20Compare%20-%20PID%20-%20CRG_country_road%20-%2070.0%20kmph.svg) | ![Gain Comparison - SMC - ISO-F](simulation/figures/svg/tests_common/gain_compare/iso_gain_compare/F/SMC/Gain%20Compare%20-%20SMC%20-%20F%20-%2070.0%20kmph.svg) |
| Fig: This plot illustrates the gain comparison for the PID controller on a CRG Country Road profile at 70 km/h, showcasing the benefits of adaptive gain. | Fig: This plot illustrates the gain comparison for the SMC controller on an ISO-F road profile at 70 km/h, highlighting the improved performance with adaptive control. |