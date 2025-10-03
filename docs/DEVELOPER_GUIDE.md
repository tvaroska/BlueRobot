# BlueRobot Developer Guide

## Architecture Overview

BlueRobot is a dual-configuration FRC robot supporting both **Coral** and **Algae** game piece handling. The codebase uses a shared base architecture with configuration-specific extensions.

### Key Components
- **WPILib Command-Based Framework**
- **CTRE Phoenix 6 Swerve Drivetrain**
- **PhotonVision AprilTag Localization**
- **REV Robotics Motor Controllers (SparkMax)**
- **Dual Robot Configuration Support**

---

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java                    # Main robot class, selects configuration
├── RobotContainer.java           # Base container with common subsystems
├── CoralRobotContainer.java      # Coral-specific configuration
├── AlgaeRobotContainer.java      # Algae-specific configuration
├── Constants.java                # Operator constants
├── RobotConstants.java           # Hardware constants per robot
├── VisionConstants.java          # Vision system configuration
├── Telemetry.java               # Telemetry logging
├── Sensitivity.java             # Input sensitivity transfer function
│
├── commands/
│   ├── CoralCommand.java        # Coral intake control
│   ├── DriveToAprilTag.java     # Vision-based navigation
│   └── AutoSelector.java        # Autonomous routine selector
│
├── subsystems/
│   ├── CommandSwerveDrivetrain.java  # Swerve drive implementation
│   ├── VisionSubsystem.java          # Dual-camera vision
│   ├── CoralMechanism.java           # Coral intake (Coral robot)
│   ├── ElevatorSubsystem.java        # Elevator (Coral robot)
│   ├── Pneumatics.java               # Pneumatic system (Algae robot)
│   ├── SafetyMonitor.java            # Safety monitoring
│   └── FieldConfiguration.java       # AprilTag field layout
│
└── generated/
    ├── TunerConstants.java       # CTRE Phoenix Tuner X generated
    └── ModuleConstants.java      # Swerve module configuration
```

---

## Robot Configuration System

### Selection Mechanism (`Robot.java:24-25`)
```java
double robot = SmartDashboard.getNumber("Robot", 0);
m_robotContainer = robot == 1 ? new AlgaeRobotContainer() : new CoralRobotContainer();
```

- **SmartDashboard value = 0**: Coral Robot (default)
- **SmartDashboard value = 1**: Algae Robot

### Hardware Differentiation (`RobotConstants.java`)
Each robot has unique swerve module constants:
- Different CAN IDs for motors and encoders
- Different encoder offsets for module calibration
- Module positions (x, y coordinates)

---

## Subsystem Details

### 1. Swerve Drivetrain (`CommandSwerveDrivetrain.java`)

**Technology**: CTRE Phoenix 6 SwerveDrivetrain API

**Features**:
- Field-centric and robot-centric drive modes
- SysId characterization support
- Vision measurement integration
- Brake/point wheel modes

**Key Methods**:
- `applyRequest()`: Execute swerve requests
- `addVisionMeasurement()`: Fuse vision pose estimates
- `seedFieldCentric()`: Reset field orientation

**Control Inputs** (`RobotContainer.java:105-123`):
- Slew rate limiters for smooth acceleration (rate = 3)
- 10% deadband on translation and rotation
- Open-loop voltage control

---

### 2. Vision Subsystem (`VisionSubsystem.java`)

**Dual-Camera System**:
- **Front Camera**: `photonvision-front`
- **Rear Camera**: `photonvision-rear`

**Pose Estimation Strategy**:
- Primary: `MULTI_TAG_PNP_ON_COPROCESSOR`
- Fallback: `LOWEST_AMBIGUITY`

**Quality Filtering** (`VisionSubsystem.java:237-260`):
```java
- Max ambiguity: VisionConstants.MAX_AMBIGUITY (single-tag only)
- Max distance: VisionConstants.MAX_VISION_DISTANCE
- Dynamic standard deviations based on:
  - Distance to target (increases uncertainty)
  - Number of tags (decreases uncertainty)
```

**Camera Transforms**:
- Defined in `VisionConstants.java`
- Separate transforms for front/rear cameras
- Position (x, y, z) and rotation (roll, pitch, yaw)

**API Methods**:
- `getNearestFrontTarget()`: Get best target from front camera
- `getFrontTargetById(int id)`: Find specific AprilTag
- `getFrontCameraResult()` / `getRearCameraResult()`: Raw pipeline results

---

### 3. Coral Mechanism (`CoralMechanism.java`)

**Hardware**:
- **Motor**: REV SparkMax (CAN ID 3, brushless)
- **Sensors**: CANdi device (CAN ID 7) with S1/S2 limit switches

**Coral Detection Logic** (`CoralMechanism.java:57-76`):
```java
S1: Detects coral entering
S2: Confirms coral inside
Auto-stops intake when coral fully detected (rising edge on S1)
```

**Control**:
- Speed scaling: 80% max (`setSpeed() * 0.8`)
- Automatic halt on detection when intake direction
- Manual override via SmartDashboard

---

### 4. Elevator Subsystem (`ElevatorSubsystem.java`)

**Hardware**:
- **Motor**: REV SparkMax (CAN ID 1, brushless)
- **Limit Switches**: Forward and reverse normally-open
- **Soft Limits**: ±500 encoder rotations

**PID Control** (`ElevatorSubsystem.java:27-29, 59-63`):
```java
kP = 0.05, kI = 0.0, kD = 0.0
Output range: -0.3 to 0.3
Live tuning via SmartDashboard
```

**Preset Positions**:
- Top: -19.5 rotations
- L1: -8.5 rotations
- Bottom: -0.5 rotations

**Control Modes**:
- Jogging: Open-loop ±0.2 speed
- Presets: Closed-loop position control

---

### 5. Pneumatics Subsystem (`Pneumatics.java`)

**Hardware**:
- PneumaticHub for compressor control
- DoubleSolenoid for actuator control

**States**:
- Forward / Reverse / Off
- Compressor enable/disable

---

### 6. Safety Monitor (`SafetyMonitor.java`)

**Purpose**: Monitor critical systems and log safety events

Implementation details vary by team requirements.

---

## Command Architecture

### CoralCommand (`CoralCommand.java`)
- Controls coral mechanism based on trigger input
- Supports forward/reverse operation
- Uses supplier pattern for trigger axis

### DriveToAprilTag (`DriveToAprilTag.java`)
- Vision-guided navigation to AprilTags
- Supports targeting specific tag IDs
- Uses PhotonVision for target acquisition

### AutoSelector (`AutoSelector.java`)
- Provides SendableChooser for autonomous selection
- Different options per robot configuration
- Options: Do Nothing, Drive to Tag 1, Drive to Nearest Tag

---

## Telemetry System

### Telemetry Class (`Telemetry.java`)
- Registers with drivetrain for periodic updates
- Logs velocities, positions, and odometry

### SmartDashboard Outputs
- Vision system status and estimates
- Elevator position and PID values
- Coral mechanism sensor states
- Motor outputs and applied speeds

### DataLogManager
- Comprehensive logging to WPILib data logs
- Robot initialization events
- Vision pose updates with tag counts
- Safety and error events

---

## Configuration Files

### `VisionConstants.java`
```java
- Camera names and network tables
- Camera mounting transforms (position + rotation)
- Measurement quality thresholds
- Standard deviations for sensor fusion
- Field layout mode selection
```

### `RobotConstants.java`
```java
- Per-robot swerve module constants:
  - Motor CAN IDs (steer, drive)
  - Encoder CAN IDs and offsets
  - Module positions (x, y)
  - Inversion flags
```

### `Constants.java`
```java
- Controller ports
- Sensitivity transfer function parameters
- Speed limits (translation and rotation)
```

### `TunerConstants.java` (Generated)
- CTRE Phoenix Tuner X generated constants
- Drivetrain physical parameters
- Swerve module configurations
- **Do not manually edit**

---

## Adding New Features

### Adding a New Subsystem
1. Create subsystem class extending `SubsystemBase`
2. Initialize in appropriate `RobotContainer` (base/Coral/Algae)
3. Configure bindings in `configureBindings()`
4. Add telemetry in `periodic()`

### Adding New Controls
1. Define binding in `CoralRobotContainer` or `AlgaeRobotContainer`
2. Use `joystick` (CommandXboxController, port 0)
3. Call `super.configureBindings()` first to preserve base controls

### Adding Autonomous Routines
1. Create command in `commands/` package
2. Add to `AutoSelector.initializeCoralAutos()` or `initializeAlgaeAutos()`
3. Use `chooser.addOption(name, command)`

---

## Vision System Configuration

### Camera Calibration
1. Use PhotonVision web interface for camera calibration
2. Update camera transforms in `VisionConstants.java`
3. Measure physical offsets from robot center accurately

### Field Layout
1. Set `FIELD_MODE` in `VisionConstants.java`
2. Options: `REEFSCAPE_2025`, others via `AprilTagFields` enum
3. Verify tag positions match actual field

### Tuning Vision Trust
Adjust in `VisionConstants.java`:
- `MAX_AMBIGUITY`: Lower = stricter single-tag filtering
- `MAX_VISION_DISTANCE`: Meters beyond which vision is ignored
- `VISION_MEASUREMENT_STD_DEVS`: Base uncertainty values
- `DISTANCE_WEIGHT`: How much distance increases uncertainty

---

## PID Tuning

### Elevator PID (Coral Robot)
1. Deploy code to robot
2. Open SmartDashboard
3. Adjust `kP`, `kI`, `kD` values
4. Observe `Elevator Position` vs `Target Position`
5. Update values in `ElevatorSubsystem.java` when satisfied

### Swerve Module PIDs
1. Use CTRE Phoenix Tuner X for swerve tuning
2. Regenerate `TunerConstants.java`
3. Redeploy code

---

## Testing Procedures

### SysId Characterization
**Dynamic Tests** (for kV, kA):
- Hold Back + Y: Forward dynamic
- Hold Back + X: Reverse dynamic

**Quasistatic Tests** (for kS):
- Hold Start + Y: Forward quasistatic
- Hold Start + X: Reverse quasistatic

Analyze logs with WPILib SysId tool.

### Vision Testing
1. Enable vision telemetry on SmartDashboard
2. Verify camera connections
3. Check `Has Targets` and `Using Measurement` booleans
4. Review estimated poses vs odometry
5. Test multi-tag vs single-tag scenarios

### Mechanism Testing
1. Use manual overrides (SmartDashboard sliders)
2. Monitor sensor states in console/dashboard
3. Test limit switches and soft limits
4. Verify PID setpoint tracking

---

## Build and Deployment

### Prerequisites
- WPILib VS Code with FRC extensions
- Phoenix Tuner X for CTRE devices
- PhotonVision running on coprocessor

### Build Commands
```bash
./gradlew build          # Build project
./gradlew deploy         # Deploy to robot
./gradlew simulateJava   # Run simulation
```

### Dependencies
- WPILib (wpilibj, wpimath, wpilibNewCommands)
- CTRE Phoenix 6 (Swerve, CANdi, TalonFX)
- REV Robotics (SparkMax)
- PhotonVision (PhotonLib)

---

## Common Issues and Solutions

### Vision Pose Jumps
- **Cause**: High ambiguity or bad tag detection
- **Fix**: Lower `MAX_AMBIGUITY` threshold or increase `VISION_MEASUREMENT_STD_DEVS`

### Elevator Oscillation
- **Cause**: High kP or low kD
- **Fix**: Reduce kP, add kD for damping

### Coral Mechanism Stops Prematurely
- **Cause**: S1/S2 sensor noise
- **Fix**: Add debouncing or adjust sensor logic in `CoralMechanism.java:60-66`

### Swerve Modules Not Aligned
- **Cause**: Incorrect encoder offsets
- **Fix**: Re-run Phoenix Tuner X calibration, update `RobotConstants.java`

### Robot Selects Wrong Configuration
- **Cause**: SmartDashboard value not set
- **Fix**: Set "Robot" number before enabling, or change default in `Robot.java:25`

---

## Code Style and Conventions

- **Command-based paradigm**: All robot actions are commands
- **Subsystems own hardware**: Motors, sensors encapsulated in subsystems
- **Config inheritance**: Base `RobotContainer` → specific containers
- **Telemetry everywhere**: Log state changes and sensor readings
- **Null safety**: Vision subsystem gracefully handles camera failures
- **WPILib standards**: Follow official WPILib code style

---

## Contributing

1. Test changes on both Coral and Algae configurations
2. Verify vision system still functions after changes
3. Update telemetry for new features
4. Add autonomous options if applicable
5. Document hardware changes in `RobotConstants.java`
6. Log important state changes via `DataLogManager`

---

## Resources

- [WPILib Documentation](https://docs.wpilib.org)
- [CTRE Phoenix 6 Docs](https://v6.docs.ctr-electronics.com)
- [PhotonVision Docs](https://docs.photonvision.org)
- [REV Robotics Docs](https://docs.revrobotics.com)

---

## Contact

For questions about this codebase, contact the programming team lead.
