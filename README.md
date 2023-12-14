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

Edit the parameters in ```param.py```

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
- `Tm` - Torque apply by DC motor
- `Td` - Disturbance

#### Kinetic Energy

&emsp; $K = \frac{1}{2}(m_{1}L_{1}^{2}+m_{2}L_{2}^{2}+I_{1})\dot{θ_p}^{2} + \frac{1}{2}I_{2}(\dot{θ_p}+\dot{θ_r})^{2}$

&emsp; $K = \frac{1}{2}(m_{1}L_{1}^{2}+m_{2}L_{2}^{2}+I_{1}+I_{2})\dot{θ_p}^{2} + I_{2}\dot{θ_p}\dot{θ_r} + \frac{1}{2}I_{2}\dot{θ_r}^{2}$

#### Potential Energy

&emsp; $V = (m_1L + m_2L)g\cosθ_p$


#### Lagrange Method

&emsp; $\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{θ_r}} \right) - \frac{\partial L}{\partial θ_r} = 0$
&emsp;&emsp; and

&emsp; $\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{θ_p}} \right) - \frac{\partial L}{\partial θ_p} = T_m$

&emsp; Where Lagrangian (L) is

&emsp; $L = K - V$


#### Mathematical equations of RWIP is described as

&emsp; $\ddot{\theta_p} = \frac{m_{1}gL_{1}\sin\left(\theta_{p}\right)\ +\ m_{2}gL_{2}\sin\left(\theta_{p}\right)\ -\ T_{m} +\ T_{d}}{m_{1}L_{1}^{2}+m_{2}L_{2}+I_{1}}$

<br>

&emsp; $\ddot{\theta_r} = \frac{T_{r}}{I_{2}}\-\frac{m_{1}gL_{1}\ +\ m_{2}gL_{2}\ -\ T_{m} +\ T_{d}}{m_{1}L_{1}^{2}+m_{2}L_{2}+I_{1}}$

<br>

***DC Motor Dynamics***

<img src="https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/122732439/482f83f0-a4d7-4a70-b65e-006408f06a36" width="480">

<br>

#### Brushed DC Motor Parameter

- `Vin` - Input Voltage
- `R` - Motor resistance
- `L` - Motor inductance
- `i` - Motor current
- `B` - Motor damped
- `J` - Motor rotor Innertia
- `ke` - Back EMF constant
- `kt` - Torque constant

#### Electrical part

&emsp; $Vin = R i + L \frac{di}{dt} + k_e θ_r$

#### Mechanical Part

&emsp; $T_{m} = Bθ_r + J\frac{\dot{θ_r}}{dt}$

&emsp; $T_{m} = k_t i$
<br>

### Controllers

- **LQR Controller**: A linear quadratic regulator designed to stabilize the pendulum in the upright position.

    
    #### Linearization dynamics model ####
  
    - **Part RWIP**
 
      when sinθp -> 0 sinθp = θp
  
    &emsp;&emsp;&emsp; $\ddot{\theta_p} = \frac{m_{1}gL_{1}\theta_{p}\ +\ m_{2}gL_{2}\theta_{p}\ -\ T_{m} +\ T_{d}}{m_{1}L_{1}^{2}+m_{2}L_{2}+I_{1}}$

    &emsp;&emsp;&emsp; $\ddot{\theta_r} = \frac{T_{m}}{I_{2}}\-\frac{m_{1}gL_{1}\ +\ m_{2}gL_{2}\ -\ T_{m} +\ T_{d}}{m_{1}L_{1}^{2}+m_{2}L_{2}+I_{1}}$
  
    - **Part Motor**

    &nbsp;&nbsp;&nbsp;We can estimate that L << R
        
    &emsp;&emsp;&emsp; $Vin = R i + k_e θ_r$

    &emsp;&emsp;&emsp; $T_{m} = k_t i$

    #### State space ####
  
  The proceeding equations are valid around the operating point where θp = 0
  
&emsp;&emsp;&emsp;&emsp;&emsp;
![Matrix](https://latex.codecogs.com/svg.image?%5Cbegin%7Bbmatrix%7D%5Cdot%7B%5Ctheta_p%7D%5C%5C%5Cddot%7B%5Ctheta_p%7D%5C%5C%5Cdot%7B%5Ctheta_r%7D%5C%5C%5Cddot%7B%5Ctheta_r%7D%5Cend%7Bbmatrix%7D=%5Cbegin%7Bbmatrix%7D0&1&0&0%5C%5C%5Cfrac%7B(m_1L_1&plus;m_2L_2)g%7D%7Bm_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;J%7D&0&0&%5Cfrac%7Bk_tk_e%7D%7B(m_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;J)R%7D%5C%5C0&0&0&1%5C%5C-%5Cfrac%7B(m_1L_1&plus;m_2L_2)g%7D%7Bm_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;J%7D&0&0&-(%5Cfrac%7Bm_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;2J%7D%7B(m_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;J)J%7D)(%5Cfrac%7Bk_tk_e%7D%7BR%7D)%5C%5C%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7D%5Ctheta_p%5C%5C%5Cdot%7B%5Ctheta_p%7D%5C%5C%5Ctheta_r%5C%5C%5Cdot%7B%5Ctheta_r%7D%5Cend%7Bbmatrix%7D&plus;%5Cbegin%7Bbmatrix%7D0%5C%5C%5Cfrac%7Bk_t%7D%7B(m_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;J)R%7D%5C%5C0%5C%5C(%5Cfrac%7Bm_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;2J%7D%7B(m_1L_1%5E%7B2%7D&plus;m_2L_2%5E%7B2%7D&plus;J)J%7D)(%5Cfrac%7Bkt%7D%7BR%7D)%5Cend%7Bbmatrix%7DV_%7Bin%7D)

 - **PID Controller**:
     The stabilize controller to compare with LQR
    #### Transfer function ####
   
    &emsp;&emsp;&emsp; $\Large\frac{\theta_{p}(s)}{\tau_{m}(s)} = \frac{\frac{s}{-J - m_{2}l_{2}^{2}}}{s^{3} + \left(\frac{B+d_{p}}{J+m_{2}l_{2}^{2}}\right)s^{2} - \left(\frac{\left(m_{1}l_{1}+m_{2}l_{2}\right)g}{\left(J+m_{2}l_{2}^{2}\right)} - \frac{B \cdot d_{p}}{\left(J+m_{2}l_{2}^{2}\right)}\right)s - \frac{\left(m_{1}l_{1}+m_{2}l_{2}\right)Bg}{\left(J+m_{2}l_{2}^{2}\right)}}$

   #### Root Locus Design ####
   
     &emsp;&emsp;&emsp;To predict the system's characteristics as the gain (Kp) is adjusted and poles move, design the root locus.

&emsp;&emsp;&emsp; ![image](https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/122732439/a4bacd35-6c37-458c-ae57-cc66dece8c4e)

&emsp;&emsp;&emsp; Root Locus of the system. It has one zero and three poles

- **Bang-bang Controller**: 
The swing up control routine and the stabilizing control routine are switched between -15 to 15 degree

&nbsp;&nbsp; <img src="https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/122732439/79e6d6d3-9ab2-49eb-a58b-3ad10a96a96b" width="480">

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
