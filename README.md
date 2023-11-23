# Reaction Wheel Inverted Pendulum Simulation

Welcome to the Reaction Wheel Inverted Pendulum simulation! This project simulates the dynamics of a Reaction Wheel Inverted Pendulum and includes controllers such as LQR and a bang-bang brake.

## Overview

The Reaction Wheel Inverted Pendulum is a complex system that combines a pendulum with a reaction wheel, and this simulation allows you to explore its behavior under different control strategies.

### Features

- **LQR Controller:** Stabilize the system using a Linear Quadratic Regulator (LQR) controller.
  
- **Bang-Bang Brake:** Implement a bang-bang brake as an alternative controller to influence the system's behavior.

- **Realistic Dynamics:** The simulation incorporates realistic dynamics and parameters for a more accurate representation.

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
git clone https://github.com/your-username/reaction-wheel-inverted-pendulum.git
cd reaction-wheel-inverted-pendulum
pip install -r requirements.txt
