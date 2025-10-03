// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Pneumatics;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralCommand;
import frc.robot.commands.AutoSelector;

public class AlgaeRobotContainer extends RobotContainer {
    private final Pneumatics pneumatics = new Pneumatics();

    public AlgaeRobotContainer() {
        super(RobotConstants.AlgaeRobot.FrontLeft, RobotConstants.AlgaeRobot.FrontRight,
            RobotConstants.AlgaeRobot.BackLeft, RobotConstants.AlgaeRobot.BackRight);
        configureBindings();
    }

    @Override
    protected void configureBindings() {
        super.configureBindings();

        joystick.a().onTrue(new InstantCommand(() -> pneumatics.setForward()));
        joystick.b().onTrue(new InstantCommand(() -> pneumatics.setReverse()));
        joystick.x().onTrue(new InstantCommand(() -> pneumatics.setOff()));
        joystick.start().onTrue(new InstantCommand(() -> pneumatics.enableCompressor()));
        joystick.y().onTrue(new InstantCommand(() -> pneumatics.disableCompressor()));
    }

    @Override
    public Command getAutonomousCommand() {
        return AutoSelector.initializeAlgaeAutos(drivetrain, getVisionSubsystem(), pneumatics);
    }
}
