# 4-in-1 Arduino Robot Car 🚗

A single Arduino sketch that turns one hand-built robot car into **four robots**. Flip one line of code, re-upload, and the same chassis behaves as an obstacle avoider, a line follower, a line follower that dodges obstacles, or a car that follows your hand.

<img width="788" height="578" alt="image" src="https://github.com/user-attachments/assets/cadfc3af-1d9d-4984-860e-8621f8b02352" />



## Modes

| Mode | What it does |
|------|--------------|
| **Obstacle Avoiding** | Drives forward; when the ultrasonic sensor sees something within 30 cm, it stops, reverses, scans left and right with the servo, and turns toward the more open side. |
| **Line Follower** | Uses the two bottom IR sensors to track a black line on a light surface. |
| **Line Follower + Obstacle Avoiding** | Follows the line, but if an object appears in the path it steers around it and rejoins the line. |
| **Object Following** | Uses the two top IR sensors + ultrasonic to keep a target (e.g. your hand) in front of it, following it as it moves. |

## Selecting a mode

Edit this one line near the top of the sketch and re-upload:

```cpp
#define ROBOT_CONTROL_MODE OBSTACLE_AVOIDING_MODE
```

Replace the value with any of:

- `OBSTACLE_AVOIDING_MODE`
- `LINE_FOLLOWER_MODE`
- `LINE_FOLLOWER_WITH_OBSTACLE_AVOIDING_MODE`
- `OBJECT_FOLLOWING_MODE`

## Hardware

- Arduino Uno / Nano (ATmega328P)
- L298N motor driver (or similar dual H-bridge)
- 2 × DC gear motors + wheels (differential drive)
- HC-SR04 ultrasonic sensor
- SG90 servo (pans the ultrasonic sensor)
- 4 × IR sensors — 2 mounted underneath (line following), 2 mounted on top/front (object following)
- Battery pack (e.g. 4 × AA or a Li-ion pack) for motors, plus power for the Arduino

## Pin connections

| Function | Pin |
|----------|-----|
| Servo signal | D3 |
| Right motor enable (PWM) | D5 |
| Left motor enable (PWM) | D6 |
| Right motor IN1 / IN2 | D7 / D8 |
| Left motor IN1 / IN2 | D9 / D10 |
| Ultrasonic TRIG / ECHO | D11 / D12 |
| Bottom IR — right / left | A0 / A1 |
| Top IR — left / right | A2 / A3 |

> IR sensor logic: `LOW` = detects black line / hand, `HIGH` = detects light surface / nothing.

## Software setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install these libraries via **Sketch → Include Library → Manage Libraries**:
   - **Servo** (bundled with the IDE)
   - **NewPing** by Tim Eckel
3. Open [`four_in_one_robot_car.ino`](four_in_one_robot_car.ino).
4. Set `ROBOT_CONTROL_MODE` to the mode you want.
5. Select your board and port, then **Upload**.

## Tuning

Each mode has speed and distance constants at the top of its function — tweak them for your motors, battery, and floor:

- `MAX_REGULAR_MOTOR_SPEED`, `MAX_MOTOR_TURN_SPEED` — how fast it drives and turns (0–255 PWM)
- `DISTANCE_TO_CHECK` — how close an obstacle must be before reacting (cm)
- `MIN_DISTANCE` / `MAX_DISTANCE` — the follow window for object-following mode

If turns are the wrong direction, swap a motor's IN1/IN2 wires or flip the pin assignments.
