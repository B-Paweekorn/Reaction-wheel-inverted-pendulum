import numpy as np
import pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN, KEYDOWN
import io
import sys
import math
import time

# ==========================================================================================
# ======================================= PARAMETER ========================================
# ==========================================================================================

# RWIP param
L1 = 0.11  # Length of Pendulum from origin to center of mass (m)
L2 = 0.18  # Length of Pendulum (m)
m1 = 0.96  # Mass of Pendulum (kg)
m2 = 0.35  # Mass of Wheel (kg)
I1 = 0.0212  # Innertia moment of Pendulum (Kg*m^2)
I2 = 0.0027  # Innertia moment of Wheel (Kg*m^2)
g = 9.81  # Gravitational acceleration
dp = 0.01  # RWIP damped
wheelradius = 0.05  # Radius of Reaction wheel

# Motor param
J = 0.0027  # Innertia Motor (Kgm^2)
Ng = 0.83  # Transmission ratio of DC Motor
ke = 3.69e-2  # Electrical constant of DC Motor (Vs/rad)
kt = ke * Ng  # Mechanical constant of DC Motor (Nm/A)
R = 2.85  # Resistor of DC Motor (Ohm)
L = 3.73e-4  # Inductance of DC Motor (Henry)
B = 3.85e-3  # Damped of DC Motor (Nn/v)

# ==========================================================================================
# ======================================= FUNCTION =========================================
# ==========================================================================================


def Forwardkinematics(q):
    x = L2 * math.sin(q)
    y = L2 * math.cos(q)
    return x, y


def PendulumEnergy(q, qp_d):
    K = (
        (0.5 * m1 * math.pow(qp_d * L1, 2))
        + (0.5 * m2 * math.pow(qp_d * L2, 2))
        + (0.5 * I2 * qp_d * qp_d)
    )  # Kinetic energy
    P = (m1 + m2) * g * L2 * math.cos(q)  # Potential energy
    return K + P


def MotorDynamics(Vin, dt):
    global qr_d, qr
    Tm = ((Vin - (qr_d * ke)) / R) * kt
    qr_dd = (Tm - B * qr_d) / J
    qr_d = qr_d + (qr_dd * dt)
    qr = qr + (qr_d * dt)
    return Tm


def RwipDynamics(q, Tr, Tp):
    qdd = (
        (m1 * g * L1 * math.sin(q)) + (m2 * g * L2 * math.sin(q)) - Tr + Tp - dp * qp_d
    ) / ((m1 * L1**2.0) + (m2 * L2) + I1)
    return qdd


def plot_figure(screen, qp, qp_d, qr_d, Tm, Vin, Tp, setpoint):
    x_offset = 240
    y_offset = 360
    multiplier = 800

    # Draw RWIP
    x, y = Forwardkinematics(qp)
    x, y = x_offset - x * multiplier, y_offset - y * multiplier
    pygame.draw.line(screen, BLACK, (x_offset, y_offset), (x, y), 3)

    # Draw wheel
    pygame.draw.circle(screen, GREY, (x, y), wheelradius * multiplier, 3)

    # Draw cross
    cross_length = wheelradius * multiplier
    cross_dx = cross_length * np.sin(qr)
    cross_dy = -cross_length * np.cos(qr)
    pygame.draw.line(
        screen, BLACK, (x - cross_dx, y - cross_dy), (x + cross_dx, y + cross_dy), 1
    )
    pygame.draw.line(
        screen, BLACK, (x - cross_dy, y + cross_dx), (x + cross_dy, y - cross_dx), 1
    )

    # Draw text
    font = pygame.font.Font(None, 20)
    texts = [
        f"Setpoint (deg): {round(np.rad2deg(setpoint), 2)}",
        f"Pendulum Angle (deg): {round(np.rad2deg(qp), 2)}",
        f"Pendulum Speed (deg/s) : {round(np.rad2deg(qp_d), 2)}",
        f"Controller Mode : {controller_mode}",
    ]
    for i, text in enumerate(texts):
        rendered_text = font.render(text, True, BLACK)
        screen.blit(rendered_text, (10, 80 + i * 20))
    texts = [
        f"Motorspeed (RPM): {round(qr_d * 60 / (math.pi * 2), 2)}",
        f"Apply Torque (Nm): {round(Tm, 2)}",
        f"Vin (V): {round(Vin, 2)}",
        f"Disturbance Torque (Tp): {round(Tp, 2)}",
    ]
    for i, text in enumerate(texts):
        rendered_text = font.render(text, True, BLACK)
        screen.blit(rendered_text, (300, 80 + i * 20))


# ==========================================================================================
# ======================================= MAIN LOGIC =======================================
# ==========================================================================================

pygame.init()

width, height = 480, 560
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Reaction Wheel Inverted Pendulum")

WHITE = (255, 255, 255)
GREY = (156, 156, 154)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

font = pygame.font.Font(None, 36)
clock = pygame.time.Clock()

qp = np.deg2rad(180)  # Initial pendulum angle
qp_d = 0.0  # Initial pendulum speed

qr = 0  # Initial reaction wheel angle
qr_d = 0  # Initial reaction wheel speed

Tm = 0  # Initial reaction wheel torque
Tp = 0  # Initial disturbance torque

dt = 1 / 10  # frequency (Hz)
reqE = (m1 + m2) * g * L2 * math.cos(0)

setpoint = 0

d_flag = 0

settled_flag = False
wait_flag = False

running = True
input_flag = False
input_string = ""

while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == MOUSEBUTTONDOWN:
            if 10 < event.pos[0] < 110 and 10 < event.pos[1] < 60:
                input_flag = True
            if 372 < event.pos[0] < 472 and 10 < event.pos[1] < 60:
                qp = np.deg2rad(180)
                qp_d = 0.0
                qr = 0
                qr_d = 0
                Tm = 0
                Tp = 0
                settled_flag = False
                wait_flag = False

        elif event.type == KEYDOWN:
            if event.key == pygame.K_BACKSPACE:
                input_string = input_string[:-1]
            else:
                input_string += event.unicode

    if input_flag == True:
        try:
            Tp = float(input_string)
        except:
            Tp = 0
            input_string = ""
        input_flag = False
    else:
        Tp = 0
    
    # Update setpoint
    setpoint_offset = (qp - math.pi) / (2 * math.pi)
    if setpoint_offset < 0:
        setpoint = (math.floor(setpoint_offset) + 1) * 2 * math.pi
    elif setpoint_offset > 0:
        setpoint = math.ceil(setpoint_offset) * 2 * math.pi
    
    E = PendulumEnergy(q=qp, qp_d=qp_d)

    if wait_flag:
        controller_mode = "brake"
        if abs(E) < 0.05:
            wait_flag = False
    elif abs(qp) % (2 * math.pi) <= np.deg2rad(20) or abs(qp) % (
        2 * math.pi
    ) >= np.deg2rad(340):
        settled_flag = True
        controller_mode = "PID"
    else:
        if settled_flag:
            wait_flag = True
            settled_flag = False
        if not wait_flag:
            controller_mode = "Bang-bang"

    if controller_mode == "PID":
        e = qp - setpoint
        Vin = e * 300 + 50 * qp_d
    elif controller_mode == "Bang-bang":
        if (qp_d < 0 and E < reqE) or (qp_d >= 0 and E >= reqE):
            Vin = 12
        elif (qp_d >= 0 and E < reqE) or (qp_d < 0 and E >= reqE):
            Vin = -12
    elif controller_mode == "brake":
        if (qp_d < 0 and E < reqE) or (qp_d >= 0 and E >= reqE):
            Vin = -12
        elif (qp_d >= 0 and E < reqE) or (qp_d < 0 and E >= reqE):
            Vin = 12
    else:
        Vin = 0

    if Vin > 24:
        Vin = 24
    elif Vin < -24:
        Vin = -24

    Tm = MotorDynamics(Vin, dt)

    qp_dd = RwipDynamics(qp, Tm, Tp)
    qp_d = qp_d + (qp_dd * dt)
    qp = qp + (qp_d * dt)

    # Draw background
    screen.fill(WHITE)

    # Draw grid
    for i in range(40, 480, 50):
        pygame.draw.line(screen, GREY, (i, 160), (i, 600), 1)
    for i in range(160, 560, 50):
        pygame.draw.line(screen, GREY, (0, i), (480, i), 1)

    # Draw figure
    plot_figure(screen, qp, qp_d, qr_d, Tm, Vin, Tp, setpoint)

    # Draw button
    pygame.draw.rect(screen, GREY, (10, 10, 100, 50))
    text = font.render("Inject", True, (0, 0, 0))
    screen.blit(text, (25, 22))

    pygame.draw.rect(screen, RED, (372, 10, 100, 50))
    text = font.render("RESET", True, (255, 255, 255))
    screen.blit(text, (383, 23))

    # Draw disturbance input field
    pygame.draw.rect(screen, GREY, (130, 10, 150, 50))
    pygame.draw.rect(screen, WHITE, (145, 20, 120, 30))
    text = font.render(input_string, True, (0, 0, 0))
    screen.blit(text, (150, 25))

    # calculate FPS and draw
    fps = clock.get_fps()
    pygame.display.set_caption(
        f"Reaction Wheel Inverted Pendulum (FPS: {round(fps, 2)}, dt: {round(dt, 4)})"
    )

    if fps:
        dt = 1 / fps

    pygame.display.flip()
    clock.tick(165)

pygame.quit()
sys.exit()
