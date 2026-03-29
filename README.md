# PID Control System Simulator & Tuner
 
![Simulator](screenshot_pid.png)
 
## Overview
Interactive PID controller simulator with first-order and
second-order plant models. Visualizes step response, control
signal, PID component breakdown, and error. Includes
Ziegler-Nichols auto-tuning suggestions.
 
## Live Demo (HuggingFace Spaces)
**[Open Simulator]()**
 
## Features
- PID controller with anti-windup
- First-order (tau) and second-order (wn, zeta) plant models
- 4 real-time plots: step response, control signal, P/I/D, error
- Performance metrics: rise time, overshoot, settling time, IAE
- Ziegler-Nichols ultimate gain tuning method
- Disturbance injection at configurable time
- Built with **Gradio** on HuggingFace
 

## Author
**Oscar Vincent Dbritto**
