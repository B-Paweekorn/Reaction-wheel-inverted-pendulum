# Reaction Wheel Inverted Pendulum Simulation

This project simulates the dynamics of a Reaction Wheel Inverted Pendulum and includes controllers such as LQR and a Bang-bang controller.

#### This project is a part of FRA333 Robot Kinematics @ Institute of Field Robotics, King Mongkut’s University of Technology Thonburi

https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/47713359/a05cc066-3fb8-44ce-9fd9-84413e4f875e

<br>

## Overview

The Reaction Wheel Inverted Pendulum is a complex system that combines a pendulum with a reaction wheel, this simulation implements Bang-bang controller to breing the pendulum up, then use LQR to keep it stable.

### Features

- **LQR Controller:** Stabilize the system using a Linear Quadratic Regulator (LQR) controller.
  
- **Bang-Bang controller:** Implement a Bang-bang controller as a swing-up controller to bring the pendulum upright.

- **Realistic Dynamics:** The simulation incorporates realistic dynamics and parameters of a DC motor and an inverted pendulum.

<br>

## Getting Started

### Prerequisites

Ensure you have the following dependencies installed:

- `numpy`
- `pygame`
- `matplotlib`
- `PyQt5`
- `pygetwindow`
- `pyaudio`
- `threading`
- `control` (install via `pip install control`)

### Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum.git
cd Reaction-wheel-inverted-pendulum
pip install -r requirements.txt
```

<br>

## Usage

![Image of the program](https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/47713359/e2ff45d4-bfb2-4831-9ec9-334b88f8ff77)

**Inject** - Type the amount of torque into the box and click `INJECT` or ENTER key to inject a disturbance to the system

**Reset** - Click `RESET` to reset the system

**Plotting** - Click on the pendulum plot or the plot window to plot the data

<br>

## Methodology
### System Modeling

The simulation involves modeling the dynamics of a Reaction Wheel Inverted Pendulum (RWIP) system. The key components include:

***Reaction Wheel Inverted Pendulum Dynamics***
<br>
<img src="https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/122732439/6c59b7a7-6aa7-4a73-b353-9e242e6aae1c" width="480">
#### RWIP Parameter
- `L1` - Pendulum length from orgin to center of mass
- `L2` - Pendulum length
- `m1` - Mass of pendulum
- `m2` - Mass of fly wheel
- `θp` - Angle of pendulum
- `θr` - Angle of fly wheel
- `I1` - Innertia moment of pendulum
- `I2` - Innertia moment fly wheel and Innertia moment of motor
- `g` - Gravitational acceleration
- `Tr` - Torque apply by DC motor
#### Kinetic Energy
```math
\begin{equation}
K=\frac{1}{2}(m_{1}L_{1}^{2}+m_{2}L_{2}^{2}+I_{1}+I_{2})\dot{θ_p}^{2}+I_{2}\dot{θ_p}\dot{θ_r}+\frac{1}{2}I_{2}\dot{θ_r}^{2}
\end{equation}
```
#### Potential Energy
```math
\begin{equation}
V = (m_1L + m_2L)g\cosθ_p
\end{equation}
```
#### Lagrange Method
```math
\begin{equation}
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{θ_r}} \right) - \frac{\partial L}{\partial θ_r} = 0
\end{equation}
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;and
```math
\begin{equation}
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{θ_p}} \right) - \frac{\partial L}{\partial θ_p} = T_r
\end{equation}
```
Where Lagrangian (L) is
```math
\begin{equation}
L = K - V
\end{equation}
```
#### Mathematical equations of RWIP is described as
***DC Motor Dynamics***

<br>
<img src="https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/122732439/482f83f0-a4d7-4a70-b65e-006408f06a36" width="480">
<br>

#### Brushed DC Parameter

- `Vin` - Input Voltage
- `R` - Motor resistance
- `L` - Motor inductance
- `i` - Motor current
- `B` - Motor damped
- `J` - Motor rotor Innertia
- `ke` - Back EMF constant
- `kt` - Torque constant
#### Electrical part
```math
\begin{equation}
Vin = R i + L \frac{di}{dt} + k_e θ_r
\end{equation}
```
#### Mechanical Part
```math
\begin{equation}
T_{m} = B\omega_m + J\frac{dθ_r}{dt}
\end{equation}
```

```math
\begin{equation}
T_{m} = k_t i
\end{equation}
```
<br>

### Controllers

- **LQR Controller**: A linear quadratic regulator designed to stabilize the pendulum in the upright position.

    ### State space ###

    fff

- **Bang-bang Controller**: Used as a swing-up controller to bring the pendulum to an upright position.

    fff

- **Brake Controller**: Used as reduced energy of RWIP when RWIP have too much energy for stabilze
### Sound Generation

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; The simulation incorporates sound generation related to the speed of the reaction wheel. This feature adds an auditory element to the simulation, enhancing the user experience.

<br>

## Acknowledgments

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; This project is part of the coursework for FRA333 Robot Kinematics at the Institute of Field Robotics, King Mongkut’s University of Technology Thonburi. Special thanks to the course instructors for their guidance and support.

Feel free to explore, modify, and extend this project for educational and research purposes.

<br>

## Reference

- [Swing Up and Balancing of a Reaction Wheel Inverted Pendulum](http://ise.ait.ac.th/wp-content/uploads/sites/57/2020/12/Swing-Up-and-Balancing-of-a-Reaction-Wheel-Inverted-Pendulum.pdf)

- [Inverted Pendulum: State-Space Methods for Controller Design](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace)

