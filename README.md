# FTC Team 24689 Acabots — Acalanes High School

![FTC](https://img.shields.io/badge/FIRST-Tech%20Challenge-red)
![Season](https://img.shields.io/badge/Season-2025--2026-blue)
![Language](https://img.shields.io/badge/Language-Java-orange)
![RoadRunner](https://img.shields.io/badge/Autonomous-RoadRunner-purple)
![FTC Dashboard](https://img.shields.io/badge/Debugging-FTC%20Dashboard-informational)
![Open Source](https://img.shields.io/badge/Open%20Source-Yes-brightgreen)
![CAD](https://img.shields.io/badge/CAD-Autodesk%20Fusion%20360-lightgrey)

## 2025–2026 Robot Code & Engineering Overview

Welcome!  
This repository contains the **open-source robot code** for **FTC Team 24689 Acabots** for the **2025–2026 season**.

---
## 📍 Team Info

**FTC Team:** 24689 Acabots  
**School:** Acalanes High School  
**Season:** 2025–2026

---

### 🔗 Engineering Resources
- **CAD Design:**  
  View our full, up-to-date robot CAD in your browser:  
  https://a360.co/4qYtIFu

---

## Robot Overview

### Key Features
- **2-Axis Turret**
  - ±200° rotation in under 1 second  
  - Servo-controlled launch angle  
  - Variable flywheel velocity for shooting from anywhere on the field  

- **High-Inertia Flywheel System**
  - Custom-designed flywheel  
  - Enables consistent, rapid shots with minimal recovery time  

- **Intake & Transfer System**
  - Polyurethane round belt intake and transfer  
  - Holds **3 artifacts simultaneously**  
  - Near-zero delay between intake and shooting  

- **Compact Mecanum Chassis**
  - ~17” wide  
  - Allows alliance partner to partially park during the Endgame  
  - GoBilda GripForce mecanum wheels for improved traction and control  

- **Extensive 3D Printing**
  - Structural and high-RPM components  
  - Reduced cost and lead time  
  - Rapid iteration during supply-chain delays  

---

## Software Highlights

### Odometry-Based Autonomous
- GoBilda odometry pods (X + Y)  
- Integrated with **RoadRunner**  
- ~¼” positional accuracy  
- Aggressive correction if bumped or blocked  
- Multiple autonomous variants depending on our alliance partner's capabilities

All autonomous routines end facing away from the driver station for a **seamless TeleOp transition**.

---

### Computer Vision–Assisted Aiming
Instead of relying solely on AprilTag visibility, we use:
- Robot position and heading (odometry)  
- Turret angle (gear-ratio scaled)  
- Camera heading = robot heading + turret angle  

This allows accurate aiming **even when the camera cannot see a tag**.

- The turret can continuously track the goal, correcting for position and adjusting the angle in real time

---

### TeleOp Driver Aids
- Field-centric driving using odometry  
- Throttle curves for smoother low-speed control  
- LED indicator shows when the flywheel is at the target RPM  
- Preset shooting configurations for multiple field locations  

---

## 🎮 Control Scheme (TeleOp)

| Control | Action |
|------|------|
| **A** | Toggle intake and transfer belts |
| **B** | Disable automatic aiming |
| **X** | Enable automatic aiming for the blue goal |
| **Y** | Enable automatic aiming for the red goal |
| **D-Pad** | Turret angle and velocity presets |
| **Right Button** | Turret kicker (feed artifacts) |
| **Left Button** | Spin up flywheel |
| **Left Joystick** | Robot rotation |
| **Right Joystick** | Field-centric strafe |
| **Left Trigger** | Rotate turret left |
| **Right Trigger** | Rotate turret right |
| **Back** | Reverse intake, belts, and kicker (jam clearing) |
| **Start** | Reinitialize odometry during TeleOp |

---

## Open Source Philosophy

We publicly release our robot code to:
- Support collaboration within the FTC community  
- Help newer teams learn advanced systems (odometry, turrets, vision)  
- Share our real-world engineering tradeoffs and design decisions  

We encourage teams to **fork, study, and adapt** anything useful.
