// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.generated.TunerConstants.ConstantCreator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.ModuleConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.CoralMechanism;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralCommand;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CANBus kCANBus = new CANBus("rio");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    protected final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain;// = TunerConstants.createDrivetrain();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final Sensitivity sensitivityPos = 
        new Sensitivity(OperatorConstants.Threshold, OperatorConstants.ZeroValue, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitX);

    //TODO Rot constants
    private final Sensitivity sensitivityRot =
        new Sensitivity(OperatorConstants.Threshold, OperatorConstants.ZeroValue, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitRot);

    public RobotContainer(ModuleConstants frontLeft, ModuleConstants frontRight, ModuleConstants backLeft, ModuleConstants backRight) {
        drivetrain = createDrivetrain(frontLeft, frontRight, backLeft, backRight);

        configureBindings();
    }

    protected SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModuleConstants(ModuleConstants constants) {
        return ConstantCreator.createModuleConstants(
            constants.steerMotorId, constants.driveMotorId, constants.encoderId,
            constants.encoderOffset,
            constants.xPos, constants.yPos,
            TunerConstants.kInvertLeftSide,
            constants.steerMotorInverted, constants.encoderInverted
        );
    }

    public CommandSwerveDrivetrain createDrivetrain(ModuleConstants frontLeft, ModuleConstants frontRight, ModuleConstants backLeft, ModuleConstants backRight) {
        return new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            createModuleConstants(frontLeft), createModuleConstants(frontRight),
            createModuleConstants(backLeft), createModuleConstants(backRight)
        );
    }

    protected void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(
                    MaxSpeed * m_xspeedLimiter.calculate(-joystick.getLeftY())
//                    MaxSpeed * sensitivityPos.transfer(-joystick.getLeftY())
//                        -joystick.getLeftY() * MaxSpeed
                    ) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed * m_yspeedLimiter.calculate(-joystick.getLeftY())
//                        MaxSpeed * sensitivityPos.transfer(-joystick.getLeftX())
//                        -joystick.getLeftX() * MaxSpeed
                    ) // Drive left with negative X (left)
                    .withRotationalRate(
                        MaxSpeed * m_rotLimiter.calculate(-joystick.getRightX())
//                        -joystick.getRightX() * MaxAngularRate
                    ) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
