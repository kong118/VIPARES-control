# VIPARES-control
---
## Minimizing Jerk in Longitudinal Speed Control : PID vs. MPC
---
### Objectives
- Design and implement a PID controller for longitudinal speed regulation.
- Design and implement an MPC controller with jerk-related penalties or input-rate constraints.
- Simulate acceleration and deceleration scenarios (step input, S-curve, variable target speeds).
- Evaluate and compare controller performance using key metrics: jerk (RMS, peak), settling time,and energy consumption.
- Provide insights into trade-offs between controller simplicity (PID) and performance/constraint handling (MPC).
---
### Tools we need
- python
- matlab
  
---
### Step 1: Create a plant model
- plant.py
- plant.m
- jerk.py

### Step 2: create a controller
- PID  
  need to build: pid.py  
  
  what is PID:  
  https://www.ni.com/en/shop/labview/pid-theory-explained.html?srsltid=AfmBOor1eDSsFWreYQ3kElTcZXPkYeczanxj3OX8jKJXzBTlYpqStuDt
- MPC  
  need to build: mpc.py  
  
  what is MPC:  
  https://engineering.purdue.edu/~zak/Second_ed/MPC_handout.pdf


  
