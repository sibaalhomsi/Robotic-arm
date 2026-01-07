Robotic Arm Tuned Project (with servo mapping & testing tools)

This version includes:
- Jacobian IK (kinematics.py)
- Joint limits and workspace enforcement
- Servo mapping: per-joint direction and offset (SERVO_DIRECTION, SERVO_OFFSET)
- GUI controls to edit directions/offsets, send test angles, and record real endpoint to compare with simulation
- Arduino sketch to receive CSV angles and drive up to 4 servos

Quick start:
1. Install requirements: pip install numpy scipy matplotlib pyserial
2. Open project in VS Code and run python main.py
3. Use Simulation Mode for testing. Use 'Send Test Angles' to send a known set.
4. Tune SERVO_DIRECTION and SERVO_OFFSET from GUI until simulation and reality match.
