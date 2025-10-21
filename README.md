Battery Consumption Analysis in Mobile Robotics
Overview

This project investigates how pulse-width modulation (PWM), voltage, and current interact to influence battery consumption and efficiency in mobile robotic systems. By simulating and analyzing these parameters, the goal is to understand how power usage patterns affect robot performance, endurance, and battery lifespan.

The research builds upon real-world robotic control scenarios where energy efficiency is a critical factor—particularly in autonomous systems where power management directly impacts operational time and system stability.

Objectives

    Model the relationship between PWM duty cycle and current draw in DC motor-driven systems.

    Analyze how varying load conditions affect power consumption.

    Quantify how battery discharge characteristics evolve under different motion profiles.

    Explore methods to optimize energy usage without compromising control accuracy or movement.

Methodology

  Simulation Setup:

    Developed in MATLAB/Simulink.

    Simulates a 4-wheel differential drive robot.

    Includes modules for motor control, battery monitoring, and drag force modeling.

    Measured Parameters:

    Input Voltage (V)

    PWM Duty Cycle (%)

    Battery State of Charge (SOC)

Data Analysis Tools:

    MATLAB scripts for plotting current vs. PWM.

    Google Sheets for long-term power trend analysis. 


Key Results (In Progress)

    Current increases non-linearly with PWM duty cycle due to dynamic motor resistance.

    Higher PWM frequencies yield smoother control but increase switching losses.

    Energy optimization is achievable through adaptive PWM tuning strategies.
