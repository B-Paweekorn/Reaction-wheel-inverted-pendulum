# Reaction Wheel Inverted Pendulum Simulation

This project simulates the dynamics of a Reaction Wheel Inverted Pendulum and includes controllers such as LQR and a Bang-bang controller.

#### This project is a part of FRA333 Robot Kinematics @ Institute of Field Robotics, King Mongkut’s University of Technology Thonburi

https://github.com/B-Paweekorn/Reaction-wheel-inverted-pendulum/assets/47713359/e4808677-a06a-4235-a8b2-1285d11bd78b

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
