# BlueRobot Operator Guide

## Robot Selection

The robot automatically selects its configuration based on the **SmartDashboard "Robot" number**:
- **0 (default)**: Coral Robot Configuration
- **1**: Algae Robot Configuration

> **Important**: Set the robot number on SmartDashboard BEFORE enabling the robot.

---

## Controller Layout

### Common Controls (Both Robots)

**Left Joystick**: Drive Translation
- Forward/Back: Y-axis
- Left/Right: X-axis

**Right Joystick**: Rotation
- Left/Right: Rotate robot

**Left Bumper**: Reset field-centric heading (resets gyro orientation)

**Right Bumper**: Point wheels (hold and use left joystick to set direction)

**X Button**: Brake mode (locks wheels in X formation)

**Back + Y / Back + X**: SysId Dynamic tests (Forward/Reverse)

**Start + Y / Start + X**: SysId Quasistatic tests (Forward/Reverse)

---

## Coral Robot Controls

The Coral Robot includes a coral intake mechanism and an elevator.

### Coral Mechanism
- **Right Trigger**: Run coral intake forward (strength = trigger pressure)
- **Left Trigger**: Run coral intake in reverse (strength = trigger pressure)

The intake includes automatic coral detection:
- **S1 Sensor**: Detects coral entering mechanism
- **S2 Sensor**: Confirms coral is inside
- Intake will automatically stop when coral is fully inside

### Elevator Controls
- **POV Up**: Jog elevator down
- **POV Down**: Jog elevator up
- **POV Left/Right**: Stop elevator

**Preset Positions**:
- **Y Button**: Move to top position (-19.5 rotations)
- **B Button**: Move to L1 position (-8.5 rotations)
- **A Button**: Move to bottom position (-0.5 rotations)

---

## Algae Robot Controls

The Algae Robot uses a pneumatic system.

### Pneumatics Controls
- **A Button**: Set pneumatics forward
- **B Button**: Set pneumatics reverse
- **X Button**: Turn off pneumatics
- **Start Button**: Enable compressor
- **Y Button**: Disable compressor

---

## Vision System

Both robots include dual-camera AprilTag vision:
- **Front Camera**: `photonvision-front`
- **Rear Camera**: `photonvision-rear`

### Vision Telemetry (SmartDashboard)
- `Vision/Front Initialized`: Front camera status
- `Vision/Rear Initialized`: Rear camera status
- `Vision/Front/Has Targets`: Front camera sees tags
- `Vision/Rear/Has Targets`: Rear camera sees tags
- `Vision/Front/Using Measurement`: Front camera pose accepted
- `Vision/Rear/Using Measurement`: Rear camera pose accepted

Vision automatically updates robot pose when AprilTags are detected with high confidence.

---

## Autonomous Mode

### Auto Selection
1. Open **SmartDashboard**
2. Find **"Auto Selector"** chooser
3. Select desired auto routine:
   - **Do Nothing** (default)
   - **Drive to Tag 1**
   - **Drive to Nearest Tag**

### Field Configuration
Set the field mode in `VisionConstants.java`:
- `FIELD_MODE = FieldMode.REEFSCAPE_2025` (competition field)
- Other modes available for testing

---

## SmartDashboard Telemetry

### Drivetrain
- Velocity, position, and orientation data (via Telemetry logger)

### Elevator (Coral Robot)
- `Elevator Position`: Current encoder position
- `Target Position`: Target position for PID
- `Motor Output`: Current motor output
- `kP`, `kI`, `kD`: Live PID tuning values

### Coral Mechanism (Coral Robot)
- `Coral Motor Speed`: Manual override speed slider
- `Coral Motor Applied Output`: Current motor output
- Console logs S1/S2 sensor states

### Pneumatics (Algae Robot)
- Compressor and solenoid states visible in console

---

## Safety Features

The robot includes a **SafetyMonitor** subsystem:
- Monitors critical systems
- Logs safety events
- Auto-disables on faults (implementation-specific)

---

## Logging

All robot operations are logged via **WPILib DataLogManager**:
- Robot selection logged at startup
- Vision pose updates logged with tag count and coordinates
- Camera initialization status logged
- Access logs via Driver Station or log viewer

---

## Troubleshooting

### Vision Not Working
1. Check camera network connections
2. Verify PhotonVision is running on coprocessor
3. Check `Vision/Front Connected` and `Vision/Rear Connected` on SmartDashboard
4. Verify AprilTags are visible and field layout is correct

### Elevator Not Moving (Coral Robot)
1. Check limit switches aren't triggered
2. Verify soft limits (±500 rotations)
3. Tune PID values via SmartDashboard if needed
4. Check motor CAN ID (should be 1)

### Coral Mechanism Not Working (Coral Robot)
1. Check S1/S2 sensor readings in console
2. Verify motor CAN ID (should be 3)
3. Try manual override using SmartDashboard slider
4. Check CANdi device (CAN ID 7) is connected

### Robot Won't Drive
1. Verify joystick is connected (port 0)
2. Check for brake mode activation (press X to exit)
3. Verify swerve modules are initialized
4. Check slew rate limiters aren't too restrictive

### Pneumatics Not Responding (Algae Robot)
1. Check compressor is enabled
2. Verify pneumatic hub is connected
3. Check solenoid channels are correct
4. Ensure air pressure is adequate

---

## Pre-Match Checklist

- [ ] Set correct robot number on SmartDashboard (0=Coral, 1=Algae)
- [ ] Verify vision cameras are connected
- [ ] Select autonomous routine
- [ ] Test drive controls
- [ ] Test mechanism controls (elevator/pneumatics/coral)
- [ ] Verify field-centric reset works
- [ ] Check battery voltage
- [ ] Review logs for errors

---

## Competition Notes

- **Robot defaults to Coral configuration** if SmartDashboard value isn't set
- All commands respect WPILib command scheduler
- Vision measurements have quality filtering (ambiguity < threshold, distance limits)
- Field-centric driving is default; reset with Left Bumper
