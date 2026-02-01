# SMASH 4.5 — FRC 9162

Codebase for **SMASH 4.5**, the development and training robot developed by **Team ALLMIGHT — 9162**.  
This version focuses on **software evolution, precision control, and advanced localization**, extending the foundations of SMASH 4.0.

---

## Robot Overview

- **Team:** ALLMIGHT — 9162  
- **Robot Name:** SMASH 4.5  
- **Season:** 2025 (Training & Development)  
- **Language:** Java  
- **Framework:** WPILib (Command-Based)  
- **Drivetrain:** Swerve (YAGSL)

---

## Highlights

### Control Architecture
- Command-based structure with swerve as default command  
- Cubic joystick scaling for improved low-speed precision  
- Hybrid control: manual driving with vision-assisted correction  
- Clear separation between driver, operator, and autonomous logic  

---

### Vision & Localization
- Dual **Limelight** setup (front and rear)  
- AprilTag detection for alignment and targeting  
- Vision-assisted lateral correction during approach  
- Designed to integrate vision data with odometry and pose estimation  

---

### Autonomous System
- **PathPlanner** with full **AutoBuilder** configuration   
- Independent PID tuning for translation and rotation  
- Automatic alliance-based path mirroring  

---

## Swerve & Odometry
- **YAGSL-based swerve implementation**  
- Continuous odometry updates in subsystem periodic loop  
- Heading and angular velocity correction enabled  

---

## Mechanisms & PID Control
- Explicit PID tuning for angulation mechanisms  
- Output limits and brake mode for stability  
- Position tolerance checks for repeatable behavior  

---

## Development Focus
- Improve pose estimation accuracy  
- Refine swerve odometry and PID tuning  
- Validate vision-assisted driving behavior  
- Establish a stable base for future competition robots  

---

Developed by the **Team ALLMIGHT — 9162 Software Team**
- **[Rafael Henritzi](https://github.com/henritzi)**  
- **[Niord Miranda](https://github.com/ProgramadorNiord)**
- **[João Santos](https://github.com/JoaoAntonio18)**
