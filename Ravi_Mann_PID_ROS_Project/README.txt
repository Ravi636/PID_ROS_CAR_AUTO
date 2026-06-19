This document provides pseudocode and function-level explanations for two control scripts:

- controlPD.py (PD controller)
- control.py (PID controller with anti-windup and error clamping)

---

controlPD.py

Overview:
Implements a Proportional–Derivative (PD) steering controller for an autonomous car using ROS. Subscribes to odometry and local plan topics, computes steering commands, and publishes to the drive topic.

Functions:

signal_handler(sig, frame)
  Functionality: Gracefully shuts down the node on interrupt (e.g., Ctrl+C), stops the car by publishing zero speed and steering.
  Inputs:
    - sig: Signal number
    - frame: Current stack frame

constrain(val, min_val, max_val)
  Functionality: Clamps val within [min_val, max_val].
  Inputs:
    - val (float): Value to clamp
    - min_val (float): Lower bound
    - max_val (float): Upper bound

odomCallback(msg)
  Functionality: Called on each odometry message. Computes heading error between current car orientation and path orientation, applies PD control law to determine steering, and publishes drive commands. Also checks for goal proximity to stop the car.
  Inputs:
    - msg (Odometry): Current pose and orientation of the car

pathCallback(msg)
  Functionality: Called on each local plan message. Extracts a midpoint orientation from the planned trajectory to set the desired heading (path_theta).
  Inputs:
    - msg (Path): Sequence of poses along the local plan

Main Execution
  - Initializes ROS node, sets up publisher (/drive) and subscribers (/odom, /move_base/.../local_plan), and enters spin loop.

Pseudocode:
BEGIN
  IMPORT ROS, Path, Odometry, AckermannDriveStamped, math, tf, sched, time, signal, sys

  // Parameters & State
  SET speed ← 0.35
  SET kp ← 3.0
  SET kd ← 0.2
  SET prev_error ← 0.0
  SET prev_time ← current_time()
  SET max_steering ← 0.43
  SET min_steering ← -0.43
  SET path_theta ← 0.0

  FUNCTION signal_handler(sig, frame):
    PRINT "Stopping car and exiting"
    UNSUBSCRIBE odom_sub, plan_sub
    REPEAT 4 TIMES:
      CREATE stop_msg with speed=0, steering=0
      PUBLISH stop_msg
    SLEEP 0.5
    EXIT

  FUNCTION constrain(val, min_val, max_val):
    RETURN clamp(val, min_val, max_val)

  FUNCTION odomCallback(msg):
    EXTRACT car_x, car_y, quaternion
    CONVERT quaternion → car_theta

    // PD control
    SET error ← wrap(path_theta − car_theta)
    SET now ← current_time()
    SET dt ← max(now − prev_time, ε)
    UPDATE prev_time
    SET derivative ← (error − prev_error)/dt
    UPDATE prev_error ← error
    SET steering ← kp*error + kd*derivative
    CLAMP steering between min_steering and max_steering

    PRINT debug info

    // Goal check
    COMPUTE distance_to_goal
    IF distance_to_goal < 0.4:
      PRINT "Reached Goal!"
      UNSUBSCRIBE odom_sub, plan_sub
      REPEAT 4 TIMES:
        CREATE stop_msg speed=0, steering=0
        PUBLISH stop_msg
    ELSE:
      CREATE drive_msg speed and steering
      PUBLISH drive_msg

  FUNCTION pathCallback(msg):
    IF msg.poses empty: RETURN
    SET idx ← floor(len(msg.poses)/2)
    EXTRACT quaternion at msg.poses[idx]
    CONVERT quaternion → path_theta

  // Main
  REGISTER signal_handler for SIGINT, SIGTERM
  INIT ROS node
  CREATE publisher '/drive'
  SUBSCRIBE '/odom' → odomCallback
  SUBSCRIBE '/move_base/.../local_plan' → pathCallback
  WHILE not shutdown: SPIN
END

---

controlPID.py

Overview:
Extends control.py to a full Proportional–Integral–Derivative (PID) controller. Adds integral action with anti-windup and clamps the heading error to prevent steering toward far-ahead targets.

Functions:

signal_handler(sig, frame)
  Functionality: Same graceful shutdown as in controlPD.py.
  Inputs:
    - sig: Signal number
    - frame: Stack frame

constrain(val, min_val, max_val)
  Functionality: Value clamping utility.
  Inputs:
    - val (float)
    - min_val (float)
    - max_val (float)

odomCallback(msg)
  Functionality: On each odometry update, computes PID control:
    1. Error: wrap difference between path_theta and car_theta.
    2. Clamp Error: restricts to ±max_error radians to avoid far-ahead steering.
    3. Integral: accumulates error over time with anti-windup (clamped to ±max_integral).
    4. Derivative: rate of error change.
    5. Steering: combine kp*error + ki*integral_error + kd*derivative, then clamp to steering limits.
    6. Checks goal proximity, stopping if within threshold.
  Inputs:
    - msg (Odometry)

pathCallback(msg)
  Functionality: Identical to controlPD.py—extracts midpoint orientation.
  Inputs:
    - msg (Path)

Main Execution
  - ROS node setup, publishers, and subscribers identical to controlPD.py.

Pseudocode:
BEGIN
  IMPORT ROS, Path, Odometry, AckermannDriveStamped, math, tf, sched, time, signal, sys

  // Parameters & State
  SET speed ← 0.65
  SET kp ← 3.2
  SET ki ← 0.05
  SET kd ← 0.2
  SET max_error ← 0.3
  SET max_integral ← 0.3
  SET max_steering ← 0.43
  SET min_steering ← -0.43
  SET prev_error ← 0.0
  SET integral_error ← 0.0
  SET prev_time ← current_time()
  SET path_theta ← 0.0

  FUNCTION signal_handler(sig, frame):
    // same as controlPD.py

  FUNCTION constrain(val, min_val, max_val):
    RETURN clamp(val, min_val, max_val)

  FUNCTION odomCallback(msg):
    EXTRACT car pose and convert to car_theta

    // PID control steps
    SET error ← wrap(path_theta - car_theta)
    CLAMP error to ±max_error
    SET now ← current_time(), dt ← max(now−prev_time, ε)
    UPDATE prev_time

    UPDATE integral_error = clamp(integral_error + error*dt, -max_integral, max_integral)
    CALCULATE derivative = (error - prev_error)/dt
    UPDATE prev_error = error

    CALCULATE steering = kp*error + ki*integral_error + kd*derivative
    CLAMP steering to [min_steering, max_steering]

    PRINT debug info

    // goal check
    IF distance_to_goal < threshold:
      STOP car
    ELSE:
      PUBLISH drive_msg

  FUNCTION pathCallback(msg):
    // same as controlPD.py

  // Main
  REGISTER signal handlers, INIT node, CREATE publisher, SUBSCRIBE topics, SPIN
END